import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from geometry_msgs.msg import Twist, Pose, PoseWithCovarianceStamped
from std_msgs.msg import Bool
import threading
import math
import time
import asyncio
import wiringpi
from wiringpi import GPIO
from nav_msgs.msg import Odometry
# 新增导入
from geometry_msgs.msg import PoseWithCovarianceStamped

# === 宏定义 ===
INIT_X = 0.0
INIT_Y = 0.0
INIT_YAW = 0.0
TARGET_X = 2.8
TARGET_Y = 0.5
TARGET_RADIUS = 0.15
# === 初始化WiringPi ===
wiringpi.wiringPiSetup()
wiringpi.pinMode(7, GPIO.OUTPUT)
wiringpi.digitalWrite(7, GPIO.LOW)

class ControlCenterNode(Node):
    def __init__(self):
        super().__init__('control_center_node')
        # 全局变量
        self.vel_x = 0.0
        self.vel_y = 0.0
        self.vel_yaw = 0.0
        self.collision_x = 0.0
        self.collision_y = 0.0
        self.camera_yaw = 0.0
        self.enemy_lock = True
        self.hit_success = False
        self.amcl_pose = None
        self.odom_pose = None
        self.start_main_thread = False

        self.search_task = None
        self.hit_task = None
        self.move_task = None

        # 订阅
        self.create_subscription(Twist, '/collision_preventor_twist', self.collision_cb, 10)
        self.create_subscription(Bool, '/enemy_lock', self.enemy_lock_cb, 10)
        self.create_subscription(Twist, '/camera_tracking_twist', self.camera_cb, 10)
        self.create_subscription(Bool, '/hit_success', self.hit_success_cb, 10)
        self.create_subscription(PoseWithCovarianceStamped, '/amcl_pose', self.amcl_pose_cb, 10)
        # 新增/wheel_odom_poweron订阅
        self.create_subscription(Bool, '/wheel_odom_poweron', self.wheel_odom_poweron_cb, 10)

        # 发布
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.paper_enable_pub = self.create_publisher(Bool, '/paper_localizer_enable', 1)
        self.initial_pose_pub = self.create_publisher(PoseWithCovarianceStamped, '/initialpose', 1)
        # 新增/set_pose发布器
        self.set_pose_pub = self.create_publisher(PoseWithCovarianceStamped, '/set_pose', 1)

        # 线程
        self._lock = threading.Lock()
        self.running = True
        self.cmd_thread = threading.Thread(target=self.cmd_vel_loop, daemon=True)
        self.cmd_thread.start()

        # 主流程线程
        self.main_thread = threading.Thread(target=self.main_logic, daemon=True)
        self.main_thread.start()

        # Asyncio event loop for coroutines
        self.async_loop = asyncio.new_event_loop()
        self.async_thread = threading.Thread(target=self.async_loop.run_forever, daemon=True)
        self.async_thread.start()

    # --- 回调 ---
    def collision_cb(self, msg):
        with self._lock:
            self.collision_x = msg.linear.x
            self.collision_y = msg.linear.y

    def enemy_lock_cb(self, msg):
        with self._lock:
            self.enemy_lock = bool(msg.data)

    def camera_cb(self, msg):
        with self._lock:
            self.camera_yaw = msg.angular.z

    def hit_success_cb(self, msg):
        with self._lock:
            self.hit_success = bool(msg.data)

    def amcl_pose_cb(self, msg):
        with self._lock:
            self.amcl_pose = msg.pose.pose


    # 新增wheel_odom_poweron回调
    def wheel_odom_poweron_cb(self, msg):
        if msg.data and not self.start_main_thread:
            self.start_main_thread = True

    # --- cmd_vel 发布线程 ---
    def cmd_vel_loop(self):
        rate = 50  # Hz
        dt = 1.0 / rate
        while self.running:
            with self._lock:
                vx = self.vel_x + self.collision_x
                vy = self.vel_y + self.collision_y
                wz = self.vel_yaw + self.camera_yaw
            twist = Twist()
            twist.linear.x = vx
            twist.linear.y = vy
            twist.angular.z = wz
            self.cmd_vel_pub.publish(twist)
            time.sleep(dt)

    # --- 主流程 ---
    def main_logic(self):
        # 等待/wheel_odom_poweron为True
        while rclpy.ok() and not self.start_main_thread:
            time.sleep(0.05)
        self.get_logger().info("收到/wheel_odom_poweron, 开始主流程")

        # 设置amcl初始位姿（此处仅发布一次，实际应调用amcl的服务或topic）
        pose_msg = PoseWithCovarianceStamped()
        pose_msg.header.stamp = self.get_clock().now().to_msg()
        pose_msg.header.frame_id = "map"
        pose_msg.pose.pose.position.x = 0.18
        pose_msg.pose.pose.position.y = 0.18
        pose_msg.pose.pose.position.z = 0.0
        # yaw=0, quaternion: (x=0, y=0, z=0, w=1)
        pose_msg.pose.pose.orientation.x = 0.0
        pose_msg.pose.pose.orientation.y = 0.0
        pose_msg.pose.pose.orientation.z = 0.0
        pose_msg.pose.pose.orientation.w = 1.0
        # covariance: set small values for x, y, yaw
        pose_msg.pose.covariance[0] = 0.25  # x
        pose_msg.pose.covariance[7] = 0.25  # y
        pose_msg.pose.covariance[35] = 0.06853891945200942  # yaw (deg^2)
        self.initial_pose_pub.publish(pose_msg)
        self.get_logger().info("已发布amcl初始位姿")
        # 新增：同步设置robot_localization初始位置
        self.set_pose_pub.publish(pose_msg)
        self.get_logger().info("已发布/set_pose初始位姿")

        # 发布/paper_localizer_enable为True
        self.paper_enable_pub.publish(Bool(data=True))
        self.get_logger().info("已发布/paper_localizer_enable=True")

        # 启动移动到目标点的任务
        self.move_task = asyncio.run_coroutine_threadsafe(self.move_to_target_task(), self.async_loop)

        # 等待到达目标点
        while rclpy.ok():
            with self._lock:
                pose = self.amcl_pose
            if pose is not None:
                x = pose.position.x
                y = pose.position.y
                dist = math.hypot(x - TARGET_X, y - TARGET_Y)
                if dist < TARGET_RADIUS:
                    self.get_logger().info("已到达目标点")
                    if self.move_task and not self.move_task.done():
                        self.move_task.cancel()
                    with self._lock:
                        self.vel_x = 0.0
                        self.vel_y = 0.0
                        self.vel_yaw = 0.0
                    break
            time.sleep(0.05)

        # 进入循环流程
        while rclpy.ok():
            with self._lock:
                enemy_lock = self.enemy_lock
                hit_success = self.hit_success
            # 搜索任务控制
            if not enemy_lock and (self.search_task is None or self.search_task.done()):
                self.get_logger().info("启动找寻任务")
                self.search_task = asyncio.run_coroutine_threadsafe(self.search_task_func(), self.async_loop)
            if enemy_lock and self.search_task and not self.search_task.done():
                self.get_logger().info("终止找寻任务")
                self.search_task.cancel()
                with self._lock:
                    self.vel_x = 0.0
                    self.vel_y = 0.0
                    self.vel_yaw = 0.0
            # 击中提示任务
            if hit_success and (self.hit_task is None or self.hit_task.done()):
                self.get_logger().info("触发击中提示任务")
                self.hit_task = asyncio.run_coroutine_threadsafe(self.hit_success_task(), self.async_loop)
            time.sleep(0.05)

    # --- 协程任务 ---
    async def move_to_target_task(self):
        # 1s加速到2m/s, 1s减速到0, 每0.05s更新
        total_time = 2.0
        dt = 0.05
        steps = int(total_time / dt)
        for i in range(steps):
            t = i * dt
            if t < 1.0:
                speed = 2.0 * (t / 1.0)
            else:
                speed = 2.0 * (1.0 - (t - 1.0) / 1.0)
            # 方向
            with self._lock:
                pose = self.amcl_pose
            if pose is None:
                await asyncio.sleep(dt)
                continue
            x, y = pose.position.x, pose.position.y
            dx = TARGET_X - x
            dy = TARGET_Y - y
            dist = math.hypot(dx, dy)
            if dist < 1e-3:
                vx, vy = 0.0, 0.0
            else:
                # 机器人yaw
                q = pose.orientation
                yaw = math.atan2(2.0*(q.w*q.z + q.x*q.y), 1.0 - 2.0*(q.y*q.y + q.z*q.z))
                # 世界坐标速度
                vx_world = dx / dist * speed
                vy_world = dy / dist * speed
                # 转到机器人坐标
                vx = math.cos(-yaw) * vx_world - math.sin(-yaw) * vy_world
                vy = math.sin(-yaw) * vx_world + math.cos(-yaw) * vy_world
            with self._lock:
                self.vel_x = vx
                self.vel_y = vy
                self.vel_yaw = 0.0
            await asyncio.sleep(dt)
        # 结束后速度归零
        with self._lock:
            self.vel_x = 0.0
            self.vel_y = 0.0
            self.vel_yaw = 0.0

    async def search_task_func(self):
        # 每0.05s更新一次角速度，采用正弦函数
        dt = 0.05
        t = 0.0
        while True:
            yaw = 0.5 + 0.5 * math.sin(0.2 * t)
            with self._lock:
                self.vel_yaw = yaw
            await asyncio.sleep(dt)
            t += dt
        # 任务结束时归零（实际cancel时会跳出循环，已经在cancel后归零）
        # with self._lock:
        #     self.vel_yaw = 0.0

    async def hit_success_task(self):
        # 留白：击中提示任务逻辑
        subprocess.run([
            '/opt/opi_test/audio/sample_audio',
            'play',
            '2',
            '/home/HwHiAiUser/Downloads/1.pcm'
        ])
        wiringpi.digitalWrite(7, GPIO.HIGH)
        await asyncio.sleep(1.0)

    def destroy_node(self):
        self.running = False
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = ControlCenterNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
