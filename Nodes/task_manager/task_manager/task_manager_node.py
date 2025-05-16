#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
import tf2_ros  # 添加tf2导入
import rclpy.time
import rclpy.duration
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, Twist
from std_msgs.msg import Bool, String
from nav2_msgs.action import NavigateToPose
from robot_control_interfaces.action import ManipulateObject  # 假设的机器人操作接口
from object_detection_interfaces.msg import DetectionResult  # 假设的检测结果消息类型
import time
import enum

class TaskState(enum.Enum):
    INIT = 0
    NAVIGATING_TO_PICKUP = 1
    PICKUP_APPROACHING = 2
    PICKING = 3
    PICKED = 4
    NAVIGATING_TO_DROPOFF = 5
    DROPPING = 6
    NAVIGATE_HOME = 7
    FINISHED = 8

class TaskManagerNode(Node):
    def __init__(self):
        super().__init__('task_manager_node')
        self.get_logger().info('Task Manager Node 启动中...')
        
        # 创建回调组，允许并发执行回调
        self.callback_group = ReentrantCallbackGroup()
        
        self.state = TaskState.INIT
        
        # 定义目标位置
        self.pickup_pose = PoseStamped()
        self.pickup_pose.header.frame_id = 'map'
        self.pickup_pose.pose.position.x = 2.0  # 设置抓取点坐标 X
        self.pickup_pose.pose.position.y = 0.0  # 设置抓取点坐标 Y
        self.pickup_pose.pose.orientation.w = 1.0
        
        self.dropoff_pose = PoseStamped()
        self.dropoff_pose.header.frame_id = 'map'
        self.dropoff_pose.pose.position.x = 0.0  # 设置放置点坐标 X
        self.dropoff_pose.pose.position.y = 2.0  # 设置放置点坐标 Y
        self.dropoff_pose.pose.orientation.w = 1.0
        
        # 当前位置初始化
        self.init_pose = PoseWithCovarianceStamped()
        self.init_pose.header.frame_id = 'map'
        self.init_pose.pose.pose.position.x = 0.0  # 初始位置 X
        self.init_pose.pose.pose.position.y = 0.0  # 初始位置 Y
        self.init_pose.pose.pose.orientation.w = 1.0
        
        # 发布者
        self.initial_pose_pub = self.create_publisher(
            PoseWithCovarianceStamped, 
            '/initialpose', 
            10
        )
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.detect_control_pub = self.create_publisher(Bool, '/enable_detection', 10)
        
        # 订阅者
        self.detection_sub = self.create_subscription(
            DetectionResult,  # 自定义消息类型，包含检测结果和目标位置
            '/object_detection/result',
            self.detection_callback,
            10,
            callback_group=self.callback_group
        )
        
        # Action客户端
        self.nav_client = ActionClient(
            self, 
            NavigateToPose, 
            'navigate_to_pose', 
            callback_group=self.callback_group
        )
        
        # 机器人操作Action客户端（用于协调机械臂和机械爪）
        self.robot_control_client = ActionClient(
            self, 
            ManipulateObject, 
            'manipulate_object', 
            callback_group=self.callback_group
        )
        
        # 添加tf2相关导入和对象
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        
        # 等待Action服务器可用
        self.get_logger().info('等待导航服务器可用...')
        self.nav_client.wait_for_server()
        self.get_logger().info('等待机器人操作服务器可用...')
        self.robot_control_client.wait_for_server()
        self.get_logger().info('都可用！')
        
        # 状态变量
        self.navigation_succeeded = False
        self.object_detected = False
        self.object_position = None  # 存储检测到的物体位置
        self.object_picked = False
        self.all_objects_picked = False
        self.current_nav_goal = None
        self.nav_goal_handle = None  # 添加存储当前导航goal handle的变量
        
        # 初始化定时器，延迟启动任务
        self.timer = self.create_timer(2.0, self.initialize_task)
        self.get_logger().info('Task Manager Node 初始化完成')
    
    def initialize_task(self):
        self.timer.cancel()  # 关闭初始化定时器
        self.get_logger().info('初始化任务...')
        
        # 设置初始位置
        self.initial_pose_pub.publish(self.init_pose)
        self.get_logger().info('发布初始位置')
        
        # 延迟一会儿，确保位置已设置
        time.sleep(1.0)
        
        # 开始第一阶段：导航到抓取点
        self.navigate_to_pickup()
    
    def navigate_to_pickup(self):
        self.get_logger().info('导航到抓取点...')
        self.state = TaskState.NAVIGATING_TO_PICKUP
        
        # 启动物体检测
        self.enable_detection(True)
        
        # 发送导航目标
        self.navigate_to_pose(self.pickup_pose)
    
    def navigate_to_dropoff(self):
        self.get_logger().info('导航到放置点...')
        self.state = TaskState.NAVIGATING_TO_DROPOFF
        
        # 关闭物体检测
        self.enable_detection(False)
        
        # 发送导航目标
        self.navigate_to_pose(self.dropoff_pose)
        
    def navigate_to_pose(self, pose):
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = pose
        
        self.current_nav_goal = pose
        self.navigation_succeeded = False
        
        # 发送导航请求
        send_goal_future = self.nav_client.send_goal_async(
            goal_msg,
            feedback_callback=self.navigation_feedback_callback
        )
        send_goal_future.add_done_callback(self.navigation_response_callback)
    
    def navigation_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('导航目标被拒绝')
            return
        
        self.get_logger().info('导航目标已接受')
        self.nav_goal_handle = goal_handle  # 保存goal handle以便后续取消
        self.get_result_future = goal_handle.get_result_async()
        self.get_result_future.add_done_callback(self.navigation_result_callback)
    
    def navigation_feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        # 反馈包含以下信息:
        # - current_pose: 机器人当前位置 (geometry_msgs/PoseStamped)
        # - navigation_time: 已经导航的时间 (builtin_interfaces/Duration)
        # - estimated_time_remaining: 预计剩余的导航时间 (builtin_interfaces/Duration)
        # - number_of_recoveries: 恢复行为执行的次数 (int16)
        # - distance_remaining: 到达目标的剩余距离 (float32)
        
        # 示例: 打印剩余距离和预计到达时间
        self.get_logger().debug(
            f'距离目标点还有: {feedback.distance_remaining:.2f}米, '
            f'预计剩余时间: {feedback.estimated_time_remaining.sec}秒'
        )
        #有待修改：也许可以设置剩余距离小于阈值就判断达到成功，直接进入navigation_result_callback，status=4的流程
    
    def navigation_result_callback(self, future):
        status = future.result().status
        if status == 4:  # 4 表示成功
            self.navigation_succeeded = True
            self.get_logger().info('导航成功完成')
            
            if self.state == TaskState.NAVIGATING_TO_PICKUP:
                if not self.object_detected:
                    # 到达抓取点但没有检测到物体
                    self.get_logger().info('到达抓取点，但没有检测到物体，所有物体已取完')
                    self.all_objects_picked = True
                    # 任务完成，可以返回初始位置
                    self.state = TaskState.FINISHED
                    self.get_logger().info('任务完成')
                # 注意：如果已经检测到并抓取了物体，这个回调可能在那之后才触发
                # 此时我们已经在detection_callback中处理了相应逻辑
            
            elif self.state == TaskState.NAVIGATING_TO_DROPOFF:
                # 到达放置点，开始放下物体
                self.drop_object()
            
            elif self.state == TaskState.NAVIGATE_HOME:
                # 返回初始位置
                self.state = TaskState.INIT
                self.get_logger().info('已返回初始位置')
                # 可能需要重新启动整个流程
                #self.initialize_task()
        else:
            self.get_logger().error(f'导航失败，状态: {status}')
            # 处理导航失败的情况
    
    def detection_callback(self, msg):
        """处理来自视觉检测节点的检测结果"""
        # 根据视觉节点的输出处理结果
        if self.state == TaskState.NAVIGATING_TO_PICKUP:
            if msg.detected_objects > 0:  # 检测到至少一个物体
                self.get_logger().info(f'检测到物体: {msg.detected_objects}个')
                self.object_detected = True
                self.object_position = msg.object_position  # 保存物体位置
                
                # 取消当前导航目标
                self.cancel_navigation()
                
                # 切换到接近状态
                self.state = TaskState.PICKUP_APPROACHING
                
                # 直接进入抓取流程
                self.pick_object()
    
    def cancel_navigation(self):
        """取消当前导航目标"""
        self.get_logger().info('正在取消当前导航目标')
        

        
        # 1. 使用Nav2 Action API取消当前导航目标
        if self.nav_goal_handle is not None and not self.nav_goal_handle.is_active:
            self.get_logger().info('当前没有活跃的导航目标，无需取消')
            return
            
        if self.nav_goal_handle is not None:
            self.get_logger().info('发送取消导航目标请求...')
            # 发送取消请求
            cancel_future = self.nav_goal_handle.cancel_goal_async()
            cancel_future.add_done_callback(self.cancel_navigation_callback)
        else:
            self.get_logger().warn('无法取消导航：没有保存的导航目标句柄')
        # 2. 然后发送停止命令，立即停止机器人移动
        stop_msg = Twist()
        self.cmd_vel_pub.publish(stop_msg)
    
    def cancel_navigation_callback(self, future):
        """处理取消导航的结果"""
        cancel_response = future.result()
        if cancel_response.return_code == 1:  # 1表示成功取消
            self.get_logger().info('导航目标已成功取消')
        else:
            self.get_logger().warn(f'取消导航失败，返回码: {cancel_response.return_code}')
        
        # 重置导航目标句柄
        self.nav_goal_handle = None
    
    def pick_object(self):
        self.get_logger().info('开始抓取物体...')
        self.state = TaskState.PICKING
        
        # 停止机器人移动
        stop_msg = Twist()
        self.cmd_vel_pub.publish(stop_msg)
        
        # 创建抓取目标
        goal_msg = ManipulateObject.Goal()
        goal_msg.operation = ManipulateObject.Goal.OPERATION_PICK  # 抓取操作
        goal_msg.object_position = self.object_position  # 目标物体位置
        
        # 发送抓取请求
        send_goal_future = self.robot_control_client.send_goal_async(
            goal_msg,
            feedback_callback=self.robot_control_feedback_callback
        )
        send_goal_future.add_done_callback(self.robot_control_response_callback)
        #有待修改：也许抓取并不需要发送物体目标位置，robot_control_client也会订阅object_detect节点的位置消息，或者到时候可能会实时pid等
    
    def drop_object(self):
        self.get_logger().info('放下物体...')
        self.state = TaskState.DROPPING
        
        # 创建放置目标
        goal_msg = ManipulateObject.Goal()
        goal_msg.operation = ManipulateObject.Goal.OPERATION_DROP  # 放置操作
        
        # 发送放置请求
        send_goal_future = self.robot_control_client.send_goal_async(
            goal_msg,
            feedback_callback=self.robot_control_feedback_callback
        )
        send_goal_future.add_done_callback(self.robot_control_response_callback)
    
    def robot_control_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('机器人操作请求被拒绝')
            return
        
        self.get_logger().info('机器人操作请求已接受')
        self.get_result_future = goal_handle.get_result_async()
        self.get_result_future.add_done_callback(self.robot_control_result_callback)
    
    def robot_control_feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        # 处理机器人操作反馈
        status_texts = {
            0: "初始化",
            1: "调整中",
            2: "抓握中", 
            3: "抓握失败",
            4: "抓握成功"
        }
        status_text = status_texts.get(feedback.status, "未知状态")
        
        if feedback.status == 3:  # 抓握失败
            self.get_logger().info(f'机器人操作状态: {status_text}，正在重试 ({feedback.retry_count})')
        else:
            self.get_logger().info(f'机器人操作状态: {status_text}')
    
    def robot_control_result_callback(self, future):
        result = future.result().result
        
        if self.state == TaskState.PICKING:
            if result.success:
                self.get_logger().info('物体抓取成功')
                self.object_picked = True
                self.state = TaskState.PICKED
                
                # 抓取成功后，更新当前位置并导航到放置点
                self.update_current_position()
                self.navigate_to_dropoff()
            else:
                self.get_logger().error(f'物体抓取失败: {result.message}')  # 使用了message字段
        
        elif self.state == TaskState.DROPPING:
            if result.success:
                self.get_logger().info('物体放置成功')
                self.object_picked = False
                
                # 放置成功后回到抓取点继续循环
                self.navigate_to_pickup()
            else:
                self.get_logger().error(f'物体放置失败: {result.message}')  # 使用了message字段
    
    def update_current_position(self):
        """使用tf2获取机器人当前位置并更新位姿
        原因：Nav2启动AMCL定位节点，持续更新map→odom变换
        机器人驱动程序发布odom→base_link变换
        所有这些变换通过/tf话题发布
"""
        self.get_logger().info('更新机器人当前位置')
        
        try:
            # 获取机器人在地图坐标系中的当前位置
            # 假设机器人的基座坐标系是base_link，地图坐标系是map
            transform = self.tf_buffer.lookup_transform(
                'map',            # 目标坐标系
                'base_link',      # 源坐标系
                rclpy.time.Time(), # 使用最新的可用变换
                timeout=rclpy.duration.Duration(seconds=1.0) # 1秒超时
            )
            
            # 创建当前位置消息
            current_pose = PoseWithCovarianceStamped()
            current_pose.header.frame_id = 'map'
            current_pose.header.stamp = self.get_clock().now().to_msg()
            
            # 从tf变换中提取位置和方向
            current_pose.pose.pose.position.x = transform.transform.translation.x
            current_pose.pose.pose.position.y = transform.transform.translation.y
            current_pose.pose.pose.position.z = transform.transform.translation.z
            current_pose.pose.pose.orientation = transform.transform.rotation
            
            # 设置协方差矩阵(这里简单设置，实际应用可能需要调整)
            for i in range(36):
                if i % 7 == 0:  # 对角线元素
                    current_pose.pose.covariance[i] = 0.01
                else:
                    current_pose.pose.covariance[i] = 0.0
            
            # 发布更新后的位置
            self.initial_pose_pub.publish(current_pose)
            self.get_logger().info(f'位置已更新为: [{current_pose.pose.pose.position.x:.2f}, {current_pose.pose.pose.position.y:.2f}]')
            
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
            self.get_logger().error(f'无法获取机器人当前位置: {str(e)}')
            self.get_logger().warn('使用最后已知的导航目标位置作为备选')
            
            # 备选方案: 使用最后的导航目标
            current_pose = PoseWithCovarianceStamped()
            current_pose.header.frame_id = 'map'
            current_pose.header.stamp = self.get_clock().now().to_msg()
            current_pose.pose.pose = self.current_nav_goal.pose
            self.initial_pose_pub.publish(current_pose)
    
    def enable_detection(self, enable):
        # 启用或禁用物体检测
        msg = Bool()
        msg.data = enable
        self.detect_control_pub.publish(msg)
        self.get_logger().info(f'{"启用" if enable else "禁用"}物体检测')
        
        if enable:
            # 重置检测状态
            self.object_detected = False

def main(args=None):
    rclpy.init(args=args)
    node = TaskManagerNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('用户中断')
    finally:
        # 清理资源
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
