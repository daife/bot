import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped, Pose
import numpy as np

class CollisionPreventorNode(Node):
    def __init__(self):
        super().__init__('collision_preventor_node')
        # 参数声明
        self.declare_parameter('wall_radius', 0.2)
        self.declare_parameter('mine_radius', 0.3)
        self.declare_parameter('wall_max_speed', 0.5)
        self.declare_parameter('mine_max_speed', 1.0)

        self.wall_radius = self.get_parameter('wall_radius').value
        self.mine_radius = self.get_parameter('mine_radius').value
        self.wall_max_speed = self.get_parameter('wall_max_speed').value
        self.mine_max_speed = self.get_parameter('mine_max_speed').value

        # 墙的边界
        self.wall_min_x = 0.0
        self.wall_max_x = 4.0
        self.wall_min_y = 0.0
        self.wall_max_y = 4.0

        # 地雷点
        self.mines = [(1.0, 2.0), (2.0, 2.0), (3.0, 2.0)]

        self.pose_sub = self.create_subscription(
            PoseWithCovarianceStamped, 'amcl_pose', self.pose_callback, 10)
        self.cmd_pub = self.create_publisher(Pose, 'collision_preventor_msg', 10)

    def pose_callback(self, msg):
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        # 机器人朝向
        q = msg.pose.pose.orientation
        theta = self.quaternion_to_yaw(q)

        # 检查墙
        wall_vec, wall_dist = self.check_wall(x, y)
        # 检查地雷
        mine_vec, mine_dist = self.check_mine(x, y)

        # 优先级：地雷 > 墙
        vx, vy = 0.0, 0.0
        if mine_vec is not None:
            speed = self.mine_max_speed * max(0.0, (self.mine_radius - mine_dist) / self.mine_radius)
            vx, vy = self.vec_to_body(mine_vec, theta, speed)
        elif wall_vec is not None:
            speed = self.wall_max_speed * max(0.0, (self.wall_radius - wall_dist) / self.wall_radius)
            vx, vy = self.vec_to_body(wall_vec, theta, speed)
        # 如果都不在警戒区，vx,vy都为0

        # 发布
        out = Pose()
        out.position.x = vx
        out.position.y = vy
        out.position.z = 0.0
        out.orientation.x = 0.0
        out.orientation.y = 0.0
        out.orientation.z = 0.0
        out.orientation.w = 1.0
        self.cmd_pub.publish(out)

    def check_wall(self, x, y):
        # 距离四条墙的最近距离和法向量
        dists = [
            (x - self.wall_min_x, np.array([1.0, 0.0])),  # 左
            (self.wall_max_x - x, np.array([-1.0, 0.0])), # 右
            (y - self.wall_min_y, np.array([0.0, 1.0])),  # 下
            (self.wall_max_y - y, np.array([0.0, -1.0]))  # 上
        ]
        min_dist, normal = min(dists, key=lambda d: d[0])
        if min_dist < self.wall_radius:
            return normal, min_dist
        return None, None

    def check_mine(self, x, y):
        for mx, my in self.mines:
            dx = x - mx
            dy = y - my
            dist = np.hypot(dx, dy)
            if dist < self.mine_radius:
                vec = np.array([dx, dy]) / (dist + 1e-6)
                return vec, dist
        return None, None

    def vec_to_body(self, vec, theta, speed):
        # 世界系向量转为机器人系速度
        # vec: 指向小车的单位向量
        # theta: 机器人朝向
        # speed: 标量
        # 先将vec旋转到机器人坐标系
        c, s = np.cos(-theta), np.sin(-theta)
        vx = vec[0] * c - vec[1] * s
        vy = vec[0] * s + vec[1] * c
        return vx * speed, vy * speed

    def quaternion_to_yaw(self, q):
        # 四元数转yaw
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        return np.arctan2(siny_cosp, cosy_cosp)

def main(args=None):
    rclpy.init(args=args)
    node = CollisionPreventorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
