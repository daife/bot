import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose
from std_msgs.msg import Bool
import numpy as np
import time

# ----------- 配置区 -----------
KALMAN_MODEL = 'constant_velocity'  # 可选: 'constant_velocity', 'constant_acceleration'
# KALMAN_MODEL: 选择卡尔曼滤波模型
#   - 'constant_velocity'：假设目标速度恒定，适合运动平稳的场景
#   - 'constant_acceleration'：假设目标加速度恒定，适合加减速明显的场景

PROCESS_NOISE = 1.0
# PROCESS_NOISE: 过程噪声（模型不确定性），数值越大，滤波器对模型预测越不信任，更依赖观测值
#   - 增大：滤波更灵敏，跟踪更快，但容易受噪声影响
#   - 减小：滤波更平滑，抗噪性强，但响应变慢

OBSERVATION_NOISE = 10.0
# OBSERVATION_NOISE: 观测噪声（测量误差），数值越大，滤波器对观测值越不信任，更依赖模型预测
#   - 增大：滤波更平滑，抗测量抖动，但可能滞后
#   - 减小：滤波更灵敏，跟踪更快，但容易受测量噪声影响

INITIAL_COVARIANCE = 100.0
# INITIAL_COVARIANCE: 初始协方差，表示初始状态的不确定性
#   - 一般设置较大，表示刚开始时对目标位置不确定

MAX_LOST_FRAMES = 10
# MAX_LOST_FRAMES: 最大允许丢帧数，超过后强制观测为0，避免滤波器漂移

DT = 1.0  # 时间间隔，单位: 帧
# DT: 每帧之间的时间间隔，通常为1（每次回调为一帧），如有特殊帧率可调整

class PaperCenterKalmanNode(Node):
    def __init__(self):
        super().__init__('paper_center_kalman_node')
        self.subscription = self.create_subscription(
            Pose,
            'paper_center_pose',
            self.pose_callback,
            10)
        self.publisher = self.create_publisher(Pose, 'paper_center_pose_smooth', 10)
        # 新增发布器
        self.enemy_lock_pub = self.create_publisher(Bool, '/enemy_lock', 10)
        self.hit_success_pub = self.create_publisher(Bool, '/hit_success', 10)
        self.lost_count = 0
        # 计时相关变量
        self.last_z1_time = None
        self.timer_started = False
        self.hit_success_sent = False
        self.init_kalman()

    def init_kalman(self):
        if KALMAN_MODEL == 'constant_velocity':
            # 状态: [x, y, vx, vy]
            self.A = np.array([[1, 0, DT, 0],
                               [0, 1, 0, DT],
                               [0, 0, 1, 0],
                               [0, 0, 0, 1]])
            self.H = np.array([[1, 0, 0, 0],
                               [0, 1, 0, 0]])
            self.Q = PROCESS_NOISE * np.eye(4)
            self.R = OBSERVATION_NOISE * np.eye(2)
            self.P = INITIAL_COVARIANCE * np.eye(4)
            self.x = np.zeros((4, 1))
        elif KALMAN_MODEL == 'constant_acceleration':
            # 状态: [x, y, vx, vy, ax, ay]
            self.A = np.array([
                [1, 0, DT, 0, 0.5*DT*DT, 0],
                [0, 1, 0, DT, 0, 0.5*DT*DT],
                [0, 0, 1, 0, DT, 0],
                [0, 0, 0, 1, 0, DT],
                [0, 0, 0, 0, 1, 0],
                [0, 0, 0, 0, 0, 1]
            ])
            self.H = np.array([[1, 0, 0, 0, 0, 0],
                               [0, 1, 0, 0, 0, 0]])
            self.Q = PROCESS_NOISE * np.eye(6)
            self.R = OBSERVATION_NOISE * np.eye(2)
            self.P = INITIAL_COVARIANCE * np.eye(6)
            self.x = np.zeros((6, 1))
        else:
            raise ValueError("KALMAN_MODEL must be 'constant_velocity' or 'constant_acceleration'")

    def pose_callback(self, msg):
        z = np.array([[msg.position.x], [msg.position.y]])
        detected = msg.position.z != -1.0
        # 连续丢帧计数，重新观测到目标时会重置
        if detected:
            self.lost_count = 0
        else:
            self.lost_count += 1

        # 预测
        self.x = self.A @ self.x
        self.P = self.A @ self.P @ self.A.T + self.Q

        # 更新
        if detected or self.lost_count > MAX_LOST_FRAMES:
            if not detected and self.lost_count > MAX_LOST_FRAMES:
                z = np.array([[0.0], [0.0]])
            y = z - self.H @ self.x
            S = self.H @ self.P @ self.H.T + self.R
            K = self.P @ self.H.T @ np.linalg.inv(S)
            self.x = self.x + K @ y
            self.P = (np.eye(self.P.shape[0]) - K @ self.H) @ self.P

        # 发布平滑后的结果
        smooth_pose = Pose()
        smooth_pose.position.x = float(self.x[0, 0])
        smooth_pose.position.y = float(self.x[1, 0])
        # 修改z值发布逻辑
        if not detected and self.lost_count > MAX_LOST_FRAMES:
            smooth_pose.position.z = -1.0  # 丢帧超过阈值，发布-1
            self.enemy_lock_pub.publish(Bool(data=False))
        else:
            smooth_pose.position.z = msg.position.z  # 维持原始值（0或1或其他）
            self.enemy_lock_pub.publish(Bool(data=True))
        self.publisher.publish(smooth_pose)

        # --- 击中计时逻辑 ---
        now = time.time()
        if msg.position.z == 1.0:
            if not self.timer_started:
                self.last_z1_time = now
                self.timer_started = True
                self.hit_success_sent = False
            else:
                # 已经计时，检查时间间隔
                if self.last_z1_time is not None and (now - self.last_z1_time) >= 2.0 and not self.hit_success_sent:
                    self.hit_success_pub.publish(Bool(data=True))
                    self.hit_success_sent = True
                self.last_z1_time = now  # 更新为最新的z=1时间
        elif msg.position.z == 0.0:
            # 重新开始计时
            self.timer_started = False
            self.last_z1_time = None
            self.hit_success_sent = False
        # z=-1不影响计时逻辑

def main(args=None):
    rclpy.init(args=args)
    node = PaperCenterKalmanNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
