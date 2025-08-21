import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool
import serial
import struct
import math
import sys
import signal

def quaternion_from_euler(roll, pitch, yaw):
    qx = math.sin(roll/2) * math.cos(pitch/2) * math.cos(yaw/2) - math.cos(roll/2) * math.sin(pitch/2) * math.sin(yaw/2)
    qy = math.cos(roll/2) * math.sin(pitch/2) * math.cos(yaw/2) + math.sin(roll/2) * math.cos(pitch/2) * math.sin(yaw/2)
    qz = math.cos(roll/2) * math.cos(pitch/2) * math.sin(yaw/2) - math.sin(roll/2) * math.sin(pitch/2) * math.cos(yaw/2)
    qw = math.cos(roll/2) * math.cos(pitch/2) * math.cos(yaw/2) + math.sin(roll/2) * math.sin(pitch/2) * math.sin(yaw/2)
    return [qx, qy, qz, qw]

class ChassisControlerAndPublisher(Node):
    def __init__(self):
        super().__init__('chassis_controlerAndpublisher')
        # ...参数声明与获取...
        self.declare_parameter('serial_device', '/dev/ttyAMA1')
        self.declare_parameter('baud_rate', 115200)
        self.declare_parameter('frame_id', 'odom')
        self.declare_parameter('child_frame_id', 'base_link')
        self.declare_parameter('publish_rate', 30.0)
        self.declare_parameter('max_linear_speed', 2.5)
        self.declare_parameter('max_angular_speed', 1.0)

        self.serial_device = self.get_parameter('serial_device').get_parameter_value().string_value
        self.baud_rate = self.get_parameter('baud_rate').get_parameter_value().integer_value
        self.frame_id = self.get_parameter('frame_id').get_parameter_value().string_value
        self.child_frame_id = self.get_parameter('child_frame_id').get_parameter_value().string_value
        self.publish_rate = self.get_parameter('publish_rate').get_parameter_value().double_value
        self.max_linear_speed = self.get_parameter('max_linear_speed').get_parameter_value().double_value
        self.max_angular_speed = self.get_parameter('max_angular_speed').get_parameter_value().double_value

        # 串口初始化
        try:
            self.serial_conn = serial.Serial(
                port=self.serial_device,
                baudrate=self.baud_rate,
                timeout=0.1
            )
            self.get_logger().info(f'Serial port {self.serial_device} opened successfully')
        except Exception as e:
            self.get_logger().error(f'Failed to open serial port: {e}')
            sys.exit(-1)

        # cmd_vel订阅
        self.subscription = self.create_subscription(
            Twist,
            'cmd_vel',
            self.cmd_vel_callback,
            50
        )

        # 里程计发布
        self.odom_pub = self.create_publisher(Odometry, '/wheel_odom', 10)
        self.poweron_pub = self.create_publisher(Bool, '/wheel_odom_poweron', 10)
        self.buffer = bytearray()
        timer_period = 1.0 / self.publish_rate
        self.timer = self.create_timer(timer_period, self.read_serial_data)
        self.last_data_valid = False

        # 看门狗
        self.last_cmd_time = self.get_clock().now()
        self.watchdog_timer = self.create_timer(0.5, self.watchdog_callback)

    def cmd_vel_callback(self, msg):
        self.last_cmd_time = self.get_clock().now()
        linear_x = max(min(msg.linear.x, self.max_linear_speed), -self.max_linear_speed)
        linear_y = max(min(msg.linear.y, self.max_linear_speed), -self.max_linear_speed)
        angular_z = max(min(msg.angular.z, self.max_angular_speed), -self.max_angular_speed)
        header = bytes([0xAA])
        data = struct.pack('<fff', linear_x, linear_y, angular_z)
        newline = b'\n'
        command = header + data + newline
        try:
            self.serial_conn.write(command)
            self.serial_conn.flush()
        except Exception as e:
            self.get_logger().error(f'Failed to send velocity command: {e}')

    def watchdog_callback(self):
        current_time = self.get_clock().now()
        if (current_time - self.last_cmd_time).nanoseconds > 1000000000:
            try:
                header = bytes([0xAA])
                data = struct.pack('<fff', 0.0, 0.0, 0.0)
                newline = b'\n'
                command = header + data + newline
                self.serial_conn.write(command)
                self.serial_conn.flush()
            except Exception as e:
                self.get_logger().error(f'Failed to send stop command: {e}')

    def read_serial_data(self):
        valid_data = False
        try:
            if self.serial_conn.in_waiting > 0:
                data = self.serial_conn.read(self.serial_conn.in_waiting)
                self.buffer.extend(data)
            while len(self.buffer) >= 26:
                header_index = self.buffer.find(0xAA)
                if header_index == -1:
                    self.buffer.clear()
                    break
                if header_index > 0:
                    self.buffer = self.buffer[header_index:]
                if len(self.buffer) < 26:
                    break
                if self.buffer[25] == 0x0A:
                    self.parse_odom_data(self.buffer[1:25])
                    valid_data = True
                    self.buffer = self.buffer[26:]
                else:
                    self.buffer = self.buffer[1:]
        except Exception as e:
            self.get_logger().error(f'Error reading serial data: {e}')
        # Publish poweron status and default odometry if no valid data
        if valid_data:
            self.last_data_valid = True
            poweron_msg = Bool()
            poweron_msg.data = True
            self.poweron_pub.publish(poweron_msg)
        else:
            self.last_data_valid = False
            poweron_msg = Bool()
            poweron_msg.data = False
            self.poweron_pub.publish(poweron_msg)
            # Publish default odometry
            current_time = self.get_clock().now()
            quat = quaternion_from_euler(0, 0, 0)
            odom_msg = Odometry()
            odom_msg.header.stamp = current_time.to_msg()
            odom_msg.header.frame_id = self.frame_id
            odom_msg.child_frame_id = self.child_frame_id
            odom_msg.pose.pose.position.x = 0.18
            odom_msg.pose.pose.position.y = 0.18
            odom_msg.pose.pose.position.z = 0.0
            odom_msg.pose.pose.orientation.x = quat[0]
            odom_msg.pose.pose.orientation.y = quat[1]
            odom_msg.pose.pose.orientation.z = quat[2]
            odom_msg.pose.pose.orientation.w = quat[3]
            odom_msg.twist.twist.linear.x = 0.0
            odom_msg.twist.twist.linear.y = 0.0
            odom_msg.twist.twist.linear.z = 0.0
            odom_msg.twist.twist.angular.x = 0.0
            odom_msg.twist.twist.angular.y = 0.0
            odom_msg.twist.twist.angular.z = 0.0
            odom_msg.pose.covariance[0] = 0.1
            odom_msg.pose.covariance[7] = 0.1
            odom_msg.pose.covariance[35] = 0.1
            odom_msg.twist.covariance[0] = 0.1
            odom_msg.twist.covariance[7] = 0.1
            odom_msg.twist.covariance[35] = 0.1
            self.odom_pub.publish(odom_msg)

    def parse_odom_data(self, data_bytes):
        try:
            odom_data = struct.unpack('<6f', data_bytes)
            x_pos = odom_data[0]
            y_pos = odom_data[1]
            x_vel = odom_data[2]
            y_vel = odom_data[3]
            angular_vel = odom_data[4]
            theta = odom_data[5]
            current_time = self.get_clock().now()
            quat = quaternion_from_euler(0, 0, theta)
            odom_msg = Odometry()
            odom_msg.header.stamp = current_time.to_msg()
            odom_msg.header.frame_id = self.frame_id
            odom_msg.child_frame_id = self.child_frame_id
            odom_msg.pose.pose.position.x = x_pos
            odom_msg.pose.pose.position.y = y_pos
            odom_msg.pose.pose.position.z = 0.0
            odom_msg.pose.pose.orientation.x = quat[0]
            odom_msg.pose.pose.orientation.y = quat[1]
            odom_msg.pose.pose.orientation.z = quat[2]
            odom_msg.pose.pose.orientation.w = quat[3]
            odom_msg.twist.twist.linear.x = x_vel
            odom_msg.twist.twist.linear.y = y_vel
            odom_msg.twist.twist.linear.z = 0.0
            odom_msg.twist.twist.angular.x = 0.0
            odom_msg.twist.twist.angular.y = 0.0
            odom_msg.twist.twist.angular.z = angular_vel
            odom_msg.pose.covariance[0] = 0.1
            odom_msg.pose.covariance[7] = 0.1
            odom_msg.pose.covariance[35] = 0.1
            odom_msg.twist.covariance[0] = 0.1
            odom_msg.twist.covariance[7] = 0.1
            odom_msg.twist.covariance[35] = 0.1
            self.odom_pub.publish(odom_msg)
        except Exception as e:
            self.get_logger().error(f'Error parsing odom data: {e}')

    def cleanup(self):
        self.get_logger().info('正在关闭底盘控制与里程计节点...')
        try:
            header = bytes([0xAA])
            data = struct.pack('<fff', 0.0, 0.0, 0.0)
            newline = b'\n'
            command = header + data + newline
            self.serial_conn.write(command)
            self.serial_conn.flush()
            if self.serial_conn.is_open:
                self.serial_conn.close()
        except Exception as e:
            self.get_logger().error(f'清理过程中出错: {e}')

def main(args=None):
    rclpy.init(args=args)
    node = ChassisControlerAndPublisher()
    def signal_handler(sig, frame):
        node.get_logger().info('收到关闭信号')
        node.cleanup()
        rclpy.shutdown()
        sys.exit(0)
    signal.signal(signal.SIGINT, signal_handler)
    signal.signal(signal.SIGTERM, signal_handler)
    try:
        rclpy.spin(node)
    except Exception as e:
        node.get_logger().error(f'发生错误: {e}')
    finally:
        node.cleanup()
        rclpy.shutdown()

if __name__ == '__main__':
    main()