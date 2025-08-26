#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
import threading
import time
import math
import numpy as np
from enum import Enum
from typing import Optional, Tuple

# ROS2 消息类型
from std_msgs.msg import Bool
from geometry_msgs.msg import Twist, Pose, PoseStamped, TransformStamped
from tf2_ros import TransformListener, Buffer
from tf2_geometry_msgs import do_transform_pose
import tf2_ros

# 动作客户端
from rclpy.action import ActionClient

# 尝试导入机械臂和爪子控制接口
try:
    from arm_control_interfaces.action import MoveArm
    from claw_control_interfaces.action import MoveClaw
except ImportError:
    MoveArm = None
    MoveClaw = None
    print("警告: 机械臂或爪子控制接口不可用")

# ============ 宏定义参数 ============
# 运动控制参数
MAX_LINEAR_ACCELERATION = 0.5   # 最大线加速度 (m/s²)
MAX_ANGULAR_ACCELERATION = 1.0  # 最大角加速度 (rad/s²)
CMD_VEL_PUBLISH_RATE = 20.0     # cmd_vel发布频率 (Hz)

# 位置误差阈值
POSITION_TOLERANCE_XY = 0.1     # x,y位置误差容忍度 (m)
ORIENTATION_TOLERANCE = 0.1     # 角度误差容忍度 (rad)

# 物体检测与追踪参数
PIXEL_DEVIATION_THRESHOLD_X = 50  # 画面中心左右偏差阈值 (像素)
PIXEL_DEVIATION_THRESHOLD_Y = 30  # 画面中心上下偏差阈值 (像素)
OBJECT_SIZE_THRESHOLD = 1600      # 物体边长阈值 (像素)
OBJECT_LOST_COUNT_THRESHOLD = 5   # 物体丢失次数阈值

# 运动速度参数
SEARCH_SPEED_X = 0.2        # 搜索时x方向速度
APPROACH_SPEED_X = 0.1      # 接近时x方向速度
CORRECTION_SPEED_Y = 0.2    # y方向修正速度
RETREAT_SPEED_X = -0.2      # 后退速度

# 机械臂参数
ARM_ADJUSTMENT_STEP = 1.0   # 机械臂调整步长 (度)
ARM_MIN_ANGLE = -20.0       # 机械臂最小角度
ARM_MAX_ANGLE = 19.0        # 机械臂最大角度

# 目标位置参数
TARGET_X = 1.0              # 目标x坐标
TARGET_Y = -2.0             # 目标y坐标
TARGET_ORIENTATION = -math.pi/2  # 目标朝向

# 时间参数
SEARCH_TIMEOUT = 5.0        # 搜索超时时间 (秒)
RETREAT_DURATION = 2.0      # 后退持续时间 (秒)
GRAB_DELAY = 0.5           # 抓取延迟时间 (秒)
RELEASE_DELAY = 1.0        # 释放延迟时间 (秒)

class AutoTaskState(Enum):
    """自动化任务状态枚举"""
    IDLE = "idle"                           # 空闲状态
    MOVE_TO_ORIGIN = "move_to_origin"       # 移动到原点
    LOWER_ARM = "lower_arm"                 # 降低机械臂
    SEARCH_OBJECT = "search_object"         # 搜索物体
    APPROACH_OBJECT = "approach_object"     # 接近物体
    GRAB_OBJECT = "grab_object"             # 抓取物体
    RETREAT = "retreat"                     # 后退
    RAISE_ARM = "raise_arm"                 # 抬高机械臂
    MOVE_TO_TARGET = "move_to_target"       # 移动到目标点
    LOWER_ARM_TARGET = "lower_arm_target"   # 在目标点降低机械臂
    RELEASE_OBJECT = "release_object"       # 释放物体
    RAISE_ARM_FINAL = "raise_arm_final"     # 最后抬高机械臂
    RETREAT_FINAL = "retreat_final"         # 最后后退

class SmoothVelocityController:
    """平滑速度控制器"""
    
    def __init__(self, max_linear_acc: float, max_angular_acc: float, publish_rate: float):
        self.max_linear_acc = max_linear_acc
        self.max_angular_acc = max_angular_acc
        self.dt = 1.0 / publish_rate
        
        # 当前速度
        self.current_linear_x = 0.0
        self.current_linear_y = 0.0
        self.current_angular_z = 0.0
        
        # 目标速度
        self.target_linear_x = 0.0
        self.target_linear_y = 0.0
        self.target_angular_z = 0.0
        
        self.lock = threading.Lock()
    
    def set_target_velocity(self, linear_x: float, linear_y: float, angular_z: float):
        """设置目标速度"""
        with self.lock:
            self.target_linear_x = linear_x
            self.target_linear_y = linear_y
            self.target_angular_z = angular_z
    
    def update_and_get_velocity(self) -> Tuple[float, float, float]:
        """更新并获取当前速度"""
        with self.lock:
            # 计算速度差
            diff_linear_x = self.target_linear_x - self.current_linear_x
            diff_linear_y = self.target_linear_y - self.current_linear_y
            diff_angular_z = self.target_angular_z - self.current_angular_z
            
            # 应用加速度限制
            max_linear_change = self.max_linear_acc * self.dt
            max_angular_change = self.max_angular_acc * self.dt
            
            # 限制线速度变化
            if abs(diff_linear_x) > max_linear_change:
                diff_linear_x = max_linear_change * (1 if diff_linear_x > 0 else -1)
            if abs(diff_linear_y) > max_linear_change:
                diff_linear_y = max_linear_change * (1 if diff_linear_y > 0 else -1)
            
            # 限制角速度变化
            if abs(diff_angular_z) > max_angular_change:
                diff_angular_z = max_angular_change * (1 if diff_angular_z > 0 else -1)
            
            # 更新当前速度
            self.current_linear_x += diff_linear_x
            self.current_linear_y += diff_linear_y
            self.current_angular_z += diff_angular_z
            
            return self.current_linear_x, self.current_linear_y, self.current_angular_z

class AutoTaskNode(Node):
    """自动化任务节点"""
    
    def __init__(self):
        super().__init__('auto_task_node')
        
        # 初始化状态
        self.current_state = AutoTaskState.IDLE
        self.enabled = False
        self.state_start_time = 0.0
        
        # TF相关
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        # 速度控制器
        self.velocity_controller = SmoothVelocityController(
            MAX_LINEAR_ACCELERATION, 
            MAX_ANGULAR_ACCELERATION, 
            CMD_VEL_PUBLISH_RATE
        )
        
        # 机械臂状态
        self.current_arm_angle = 0.0
        
        # 物体检测状态
        self.object_detected = False
        self.object_dx = 0.0  # 物体相对画面中心的x偏差
        self.object_dy = 0.0  # 物体相对画面中心的y偏差
        self.object_size = 0.0
        self.object_lost_count = 0
        
        # 初始化订阅器
        self.enable_sub = self.create_subscription(
            Bool, 'enable_auto', self.enable_callback, 10)
        
        self.yolo_pose_sub = self.create_subscription(
            Pose, '/onecam_yolo/target_pose', self.yolo_pose_callback, 10)
        
        # 初始化发布器
        self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        
        # 初始化动作客户端
        self.init_action_clients()
        
        # 启动cmd_vel发布线程
        self.cmd_vel_thread = threading.Thread(target=self.cmd_vel_publisher_thread, daemon=True)
        self.cmd_vel_thread.start()
        
        # 启动状态机线程
        self.state_machine_thread = threading.Thread(target=self.state_machine_loop, daemon=True)
        self.state_machine_thread.start()
        
        self.get_logger().info('自动化任务节点已启动')
    
    def init_action_clients(self):
        """初始化动作客户端"""
        if MoveArm is not None:
            self.arm_action_client = ActionClient(self, MoveArm, 'move_arm')
        else:
            self.arm_action_client = None
            self.get_logger().warn('机械臂接口不可用')
        
        if MoveClaw is not None:
            self.claw_action_client = ActionClient(self, MoveClaw, 'move_claw')
        else:
            self.claw_action_client = None
            self.get_logger().warn('爪子接口不可用')
    
    def enable_callback(self, msg: Bool):
        """使能状态回调"""
        if msg.data and not self.enabled:
            self.get_logger().info('启动自动化任务')
            self.enabled = True
            self.change_state(AutoTaskState.MOVE_TO_ORIGIN)
        elif not msg.data and self.enabled:
            self.get_logger().info('停止自动化任务')
            self.enabled = False
            self.emergency_stop()
    
    def yolo_pose_callback(self, msg: Pose):
        """YOLO检测结果回调"""
        if msg.position.z > 0.0:  # z > 0 表示检测到物体
            self.object_detected = True
            self.object_dx = msg.position.x
            self.object_dy = msg.position.y
            self.object_size = msg.position.z
            self.object_lost_count = 0
        else:
            if self.object_detected:  # 之前检测到，现在丢失
                self.object_lost_count += 1
                if self.object_lost_count >= OBJECT_LOST_COUNT_THRESHOLD:
                    self.object_detected = False
    
    def get_current_pose(self) -> Optional[Tuple[float, float, float]]:
        """获取当前位置和朝向"""
        try:
            # 获取从map到base_link的变换
            transform = self.tf_buffer.lookup_transform(
                'map', 'base_link', rclpy.time.Time())
            
            x = transform.transform.translation.x
            y = transform.transform.translation.y
            
            # 从四元数计算yaw角
            qx = transform.transform.rotation.x
            qy = transform.transform.rotation.y
            qz = transform.transform.rotation.z
            qw = transform.transform.rotation.w
            
            yaw = math.atan2(2.0 * (qw * qz + qx * qy), 1.0 - 2.0 * (qy * qy + qz * qz))
            
            return x, y, yaw
            
        except Exception as e:
            self.get_logger().warn(f'无法获取当前位置: {e}')
            return None
    
    def send_arm_command(self, angle: float):
        """发送机械臂命令"""
        if self.arm_action_client is None:
            return
        
        angle = max(ARM_MIN_ANGLE, min(ARM_MAX_ANGLE, angle))
        self.current_arm_angle = angle
        
        if self.arm_action_client.wait_for_server(timeout_sec=1.0):
            goal_msg = MoveArm.Goal()
            goal_msg.pose = angle
            self.arm_action_client.send_goal_async(goal_msg)
            self.get_logger().debug(f'发送机械臂角度: {angle}°')
    
    def send_claw_command(self, command: int):
        """发送爪子命令 (0=抓取, 1=释放)"""
        if self.claw_action_client is None:
            return
        
        if self.claw_action_client.wait_for_server(timeout_sec=1.0):
            goal_msg = MoveClaw.Goal()
            goal_msg.command = command
            self.claw_action_client.send_goal_async(goal_msg)
            action = "抓取" if command == 0 else "释放"
            self.get_logger().debug(f'发送爪子命令: {action}')
    
    def change_state(self, new_state: AutoTaskState):
        """改变状态"""
        self.get_logger().info(f'状态变更: {self.current_state.value} -> {new_state.value}')
        self.current_state = new_state
        self.state_start_time = time.time()
    
    def emergency_stop(self):
        """紧急停止"""
        self.velocity_controller.set_target_velocity(0.0, 0.0, 0.0)
        self.send_claw_command(1)  # 释放爪子
        self.change_state(AutoTaskState.MOVE_TO_ORIGIN)
    
    def cmd_vel_publisher_thread(self):
        """cmd_vel发布线程"""
        rate = self.create_rate(CMD_VEL_PUBLISH_RATE)
        
        while rclpy.ok():
            try:
                linear_x, linear_y, angular_z = self.velocity_controller.update_and_get_velocity()
                
                twist = Twist()
                twist.linear.x = linear_x
                twist.linear.y = linear_y
                twist.angular.z = angular_z
                
                self.cmd_vel_pub.publish(twist)
                
                rate.sleep()
            except Exception as e:
                self.get_logger().error(f'cmd_vel发布线程错误: {e}')
                time.sleep(0.1)
    
    def state_machine_loop(self):
        """状态机主循环"""
        while rclpy.ok():
            try:
                if not self.enabled:
                    time.sleep(0.1)
                    continue
                
                self.execute_current_state()
                time.sleep(0.05)  # 20Hz状态机更新频率
                
            except Exception as e:
                self.get_logger().error(f'状态机错误: {e}')
                time.sleep(0.1)
    
    def execute_current_state(self):
        """执行当前状态"""
        current_time = time.time()
        elapsed_time = current_time - self.state_start_time
        
        if self.current_state == AutoTaskState.IDLE:
            self.velocity_controller.set_target_velocity(0.0, 0.0, 0.0)
            
        elif self.current_state == AutoTaskState.MOVE_TO_ORIGIN:
            self.execute_move_to_position(0.0, 0.0, 0.0)
            
        elif self.current_state == AutoTaskState.LOWER_ARM:
            self.send_arm_command(ARM_MIN_ANGLE)
            time.sleep(0.5)  # 等待机械臂到位
            self.change_state(AutoTaskState.SEARCH_OBJECT)
            
        elif self.current_state == AutoTaskState.SEARCH_OBJECT:
            if self.object_detected:
                self.change_state(AutoTaskState.APPROACH_OBJECT)
            elif elapsed_time > SEARCH_TIMEOUT:
                self.velocity_controller.set_target_velocity(0.0, 0.0, 0.0)
                # 继续等待检测，不改变状态
            else:
                self.velocity_controller.set_target_velocity(SEARCH_SPEED_X, 0.0, 0.0)
                
        elif self.current_state == AutoTaskState.APPROACH_OBJECT:
            self.execute_approach_object()
            
        elif self.current_state == AutoTaskState.GRAB_OBJECT:
            self.velocity_controller.set_target_velocity(0.0, 0.0, 0.0)
            self.send_claw_command(0)  # 抓取
            time.sleep(GRAB_DELAY)
            self.change_state(AutoTaskState.RETREAT)
            
        elif self.current_state == AutoTaskState.RETREAT:
            if elapsed_time < RETREAT_DURATION:
                self.velocity_controller.set_target_velocity(RETREAT_SPEED_X, 0.0, 0.0)
            else:
                self.velocity_controller.set_target_velocity(0.0, 0.0, 0.0)
                self.change_state(AutoTaskState.RAISE_ARM)
                
        elif self.current_state == AutoTaskState.RAISE_ARM:
            self.send_arm_command(ARM_MAX_ANGLE)
            time.sleep(0.5)
            self.change_state(AutoTaskState.MOVE_TO_TARGET)
            
        elif self.current_state == AutoTaskState.MOVE_TO_TARGET:
            self.execute_move_to_position(TARGET_X, TARGET_Y, TARGET_ORIENTATION)
            
        elif self.current_state == AutoTaskState.LOWER_ARM_TARGET:
            self.send_arm_command(ARM_MIN_ANGLE)
            time.sleep(0.5)
            self.change_state(AutoTaskState.RELEASE_OBJECT)
            
        elif self.current_state == AutoTaskState.RELEASE_OBJECT:
            self.send_claw_command(1)  # 释放
            time.sleep(RELEASE_DELAY)
            self.change_state(AutoTaskState.RAISE_ARM_FINAL)
            
        elif self.current_state == AutoTaskState.RAISE_ARM_FINAL:
            self.send_arm_command(ARM_MAX_ANGLE)
            time.sleep(0.5)
            self.change_state(AutoTaskState.RETREAT_FINAL)
            
        elif self.current_state == AutoTaskState.RETREAT_FINAL:
            if elapsed_time < RETREAT_DURATION:
                self.velocity_controller.set_target_velocity(RETREAT_SPEED_X, 0.0, 0.0)
            else:
                self.velocity_controller.set_target_velocity(0.0, 0.0, 0.0)
                # 重新开始循环
                self.send_claw_command(1)  # 确保爪子释放
                self.change_state(AutoTaskState.MOVE_TO_ORIGIN)
    
    def execute_move_to_position(self, target_x: float, target_y: float, target_yaw: float):
        """执行移动到指定位置"""
        current_pose = self.get_current_pose()
        if current_pose is None:
            return
        
        x, y, yaw = current_pose
        
        # 计算在map坐标系下的位置误差
        dx_map = target_x - x
        dy_map = target_y - y
        dyaw = target_yaw - yaw
        
        # 角度误差标准化到[-π, π]
        while dyaw > math.pi:
            dyaw -= 2 * math.pi
        while dyaw < -math.pi:
            dyaw += 2 * math.pi
        
        # 检查是否到达目标
        if (abs(dx_map) < POSITION_TOLERANCE_XY and 
            abs(dy_map) < POSITION_TOLERANCE_XY and 
            abs(dyaw) < ORIENTATION_TOLERANCE):
            
            self.velocity_controller.set_target_velocity(0.0, 0.0, 0.0)
            
            # 根据当前状态决定下一个状态
            if self.current_state == AutoTaskState.MOVE_TO_ORIGIN:
                self.send_claw_command(1)  # 释放爪子
                self.change_state(AutoTaskState.LOWER_ARM)
            elif self.current_state == AutoTaskState.MOVE_TO_TARGET:
                self.change_state(AutoTaskState.LOWER_ARM_TARGET)
            
            return
        
        # 将map坐标系下的位置误差转换到机器人本体坐标系
        # 机器人当前朝向为yaw，需要将全局误差旋转到机器人坐标系
        cos_yaw = math.cos(yaw)
        sin_yaw = math.sin(yaw)
        
        # 旋转变换矩阵：从map坐标系到base_link坐标系
        dx_robot = cos_yaw * dx_map + sin_yaw * dy_map   # 机器人前后方向(x)
        dy_robot = -sin_yaw * dx_map + cos_yaw * dy_map  # 机器人左右方向(y)
        
        # 计算控制速度（在机器人坐标系下）
        kp_linear = 1.0
        kp_angular = 2.0
        
        max_speed = 0.3
        
        # 在机器人坐标系下计算速度命令
        vel_x = np.clip(kp_linear * dx_robot, -max_speed, max_speed)    # 前后速度
        vel_y = np.clip(kp_linear * dy_robot, -max_speed, max_speed)    # 左右速度
        vel_yaw = np.clip(kp_angular * dyaw, -1.0, 1.0)                # 角速度
        
        self.velocity_controller.set_target_velocity(vel_x, vel_y, vel_yaw)
    
    def execute_approach_object(self):
        """执行接近物体"""
        if not self.object_detected:
            self.velocity_controller.set_target_velocity(0.0, 0.0, 0.0)
            self.change_state(AutoTaskState.SEARCH_OBJECT)
            return
        
        # 检查是否应该抓取
        if (self.object_size > OBJECT_SIZE_THRESHOLD or 
            self.object_lost_count >= OBJECT_LOST_COUNT_THRESHOLD):
            self.change_state(AutoTaskState.GRAB_OBJECT)
            return
        
        # 设置前进速度
        vel_x = APPROACH_SPEED_X
        
        # 根据水平偏差调整y方向速度
        if abs(self.object_dx) > PIXEL_DEVIATION_THRESHOLD_X:
            vel_y = CORRECTION_SPEED_Y if self.object_dx < 0 else -CORRECTION_SPEED_Y
        else:
            vel_y = 0.0
        
        # 根据垂直偏差调整机械臂
        if abs(self.object_dy) > PIXEL_DEVIATION_THRESHOLD_Y:
            if self.object_dy > 0:  # 物体在上方，降低机械臂
                new_angle = self.current_arm_angle - ARM_ADJUSTMENT_STEP
            else:  # 物体在下方，抬高机械臂
                new_angle = self.current_arm_angle + ARM_ADJUSTMENT_STEP
            self.send_arm_command(new_angle)
        
        self.velocity_controller.set_target_velocity(vel_x, vel_y, 0.0)

def main(args=None):
    rclpy.init(args=args)
    
    node = AutoTaskNode()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
