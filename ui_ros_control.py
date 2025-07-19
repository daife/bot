import sys
import threading
import time
import subprocess
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose, Twist
from nav_msgs.msg import Odometry
from PyQt5 import QtWidgets, QtCore
import wiringpi
import serial
import struct

class RosDataSubscriber(Node):
    def __init__(self, update_callback):
        super().__init__('ui_ros_data_sub')
        self.update_callback = update_callback
        self.paper_pose = None
        self.odom_pose = None
        self.create_subscription(Pose, 'paper_center_pose', self.paper_cb, 10)
        self.create_subscription(Odometry, 'wheel_odom', self.odom_cb, 10)

    def paper_cb(self, msg):
        self.paper_pose = (msg.position.x, msg.position.y, msg.position.z)
        self.update_callback('paper', self.paper_pose)

    def odom_cb(self, msg):
        pos = msg.pose.pose.position
        self.odom_pose = (pos.x, pos.y, pos.z)
        self.update_callback('odom', self.odom_pose)

class MotorController:
    def __init__(self):
        self.yaw_pin = 19
        wiringpi.wiringPiSetup()
        wiringpi.pinMode(self.yaw_pin, wiringpi.GPIO.PWM_OUTPUT)
        wiringpi.pwmSetRange(self.yaw_pin, 3000000)
        # 初始化为7.5%占空比（1.5ms脉宽）
        pulse_ms = 1.5
        duty_cycle = pulse_ms / 20.0
        pwm_value = int(duty_cycle * 3000000)
        wiringpi.pwmWrite(self.yaw_pin, pwm_value)
        self.pitch_val = 0.0  # pitch值，模拟
        self.pitch_step = 0.1

    def set_yaw_duty(self, percent):
        # percent: 0~100
        pulse_ms = 1.5 + (percent - 7.5) / 2.5  # 7.5%为中位
        pulse_ms = max(1.0, min(2.0, pulse_ms))
        duty_cycle = pulse_ms / 20.0
        pwm_value = int(duty_cycle * 3000000)
        wiringpi.pwmWrite(self.yaw_pin, pwm_value)

    def yaw_left(self):
        self.set_yaw_duty(5.0)

    def yaw_right(self):
        self.set_yaw_duty(10.0)

    def yaw_stop(self):
        self.set_yaw_duty(7.5)

    def pitch_reset(self):
        # 点击回正时才启动串口，回正完成后立即关闭串口
        try:
            ser = serial.Serial('/dev/ttyAMA0', baudrate=115200, timeout=0.1)
            data = bytearray(14)
            data[0] = 0xAA
            struct.pack_into('<f', data, 1, float(0.0))
            data[13] = 0x0A
            ser.write(data)
            ser.close()
        except Exception as e:
            print(f"Pitch回正失败: {e}")

class UiMain(QtWidgets.QWidget):
    def __init__(self, hard_test_node):
        super().__init__()
        # 先初始化测试相关变量，确保后续方法调用不会报错
        self.hard_test_running = False
        self.hard_test_thread = None
        self.odom_x = 0.0
        self.odom_y = 0.0

        # 新增：主线程创建Node用于硬编码测试
        self.hard_test_node = hard_test_node
        self.hard_test_pub = self.hard_test_node.create_publisher(Twist, 'cmd_vel', 10)
        self.setWindowTitle("ROS2 控制面板")
        self.setGeometry(100, 100, 400, 300)
        self.motor = MotorController()
        self.camera_tracking_running = False
        self.camera_tracking_proc = None
        self.ros_thread = None
        self.subscriber = None

        layout = QtWidgets.QVBoxLayout()

        # chassis节点启动按钮
        self.btn_start_chassis = QtWidgets.QPushButton("打开 chassis_control 和 odom_publisher 节点")
        self.btn_start_chassis.clicked.connect(self.start_chassis_and_odom)
        layout.addWidget(self.btn_start_chassis)

        # camera_tracking控制
        self.btn_camera_tracking = QtWidgets.QPushButton("打开 camera_tracking 节点")
        self.btn_camera_tracking.clicked.connect(self.toggle_camera_tracking)
        layout.addWidget(self.btn_camera_tracking)

        # 管理员终端按钮
        self.btn_admin_terminal = QtWidgets.QPushButton("打开管理员终端")
        self.btn_admin_terminal.clicked.connect(self.open_admin_terminal)
        layout.addWidget(self.btn_admin_terminal)

        # 一键硬编码测试按钮
        self.btn_hard_test = QtWidgets.QPushButton("一键硬编码测试")
        self.btn_hard_test.clicked.connect(self.start_hard_test)
        layout.addWidget(self.btn_hard_test)

        # 数据显示
        self.label_paper = QtWidgets.QLabel("paper_center_pose: x= y= z=")
        layout.addWidget(self.label_paper)
        self.label_odom = QtWidgets.QLabel("wheel_odom: x= y= z=")
        layout.addWidget(self.label_odom)

        # 云台控制按钮
        self.btn_pitch_reset = QtWidgets.QPushButton("Pitch回正")
        self.btn_yaw_left = QtWidgets.QPushButton("Yaw 左")
        self.btn_yaw_right = QtWidgets.QPushButton("Yaw 右")
        self.btn_pitch_reset.clicked.connect(self.pitch_reset)
        self.btn_yaw_left.pressed.connect(self.yaw_left)
        self.btn_yaw_left.released.connect(self.yaw_stop)
        self.btn_yaw_right.pressed.connect(self.yaw_right)
        self.btn_yaw_right.released.connect(self.yaw_stop)
        layout.addWidget(self.btn_pitch_reset)
        layout.addWidget(self.btn_yaw_left)
        layout.addWidget(self.btn_yaw_right)

        # 新增：Yaw死区测试输入框和按钮
        hbox_deadzone = QtWidgets.QHBoxLayout()
        self.input_deadzone_time = QtWidgets.QLineEdit()
        self.input_deadzone_time.setPlaceholderText("持续时间(s)")
        self.input_deadzone_time.setFixedWidth(80)
        self.btn_test_yaw_deadzone = QtWidgets.QPushButton("测试Yaw死区")
        self.btn_test_yaw_deadzone.clicked.connect(self.test_yaw_deadzone)
        hbox_deadzone.addWidget(self.input_deadzone_time)
        hbox_deadzone.addWidget(self.btn_test_yaw_deadzone)
        layout.addLayout(hbox_deadzone)

        self.setLayout(layout)
        self.update_manual_buttons()
        self.start_ros_subscriber()

    def start_chassis_and_odom(self):
        def run_node(cmd, title):
            term_cmd = [
                'xfce4-terminal',
                '--hold',
                '--title', title,
                '--working-directory', '/home/HwHiAiUser/ros',
                '-e', f'sudo bash -c "source /home/HwHiAiUser/ros/install/setup.sh && {cmd}"'
            ]
            subprocess.Popen(term_cmd, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        threading.Thread(target=run_node, args=("ros2 run chassis_control_rclpy chassis_control_node", "chassis_control_node"), daemon=True).start()
        threading.Thread(target=run_node, args=("ros2 run odom_publisher odom_publisher_node", "odom_publisher_node"), daemon=True).start()

    def toggle_camera_tracking(self):
        if not self.camera_tracking_running:
            # 启动 camera_tracking 节点（新终端）
            term_cmd = [
                'xfce4-terminal',
                '--hold',
                '--title', 'camera_tracking_node',
                '--working-directory', '/home/HwHiAiUser/ros',
                '-e', 'sudo bash -c "source /home/HwHiAiUser/ros/install/setup.sh && ros2 run camera_tracking camera_tracking_node"'
            ]
            self.camera_tracking_proc = subprocess.Popen(term_cmd, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
            self.camera_tracking_running = True
            self.btn_camera_tracking.setText("关闭 camera_tracking 节点")
        else:
            # 强制关闭 camera_tracking 节点
            try:
                subprocess.run(['pkill', '-f', 'camera_tracking_node'], timeout=3)
                subprocess.run(['pkill', 'camera_tracking_node'], timeout=3)
                subprocess.run(['killall', 'camera_tracking_node'], timeout=3)
            except Exception as e:
                print(f"关闭camera_tracking失败: {e}")
            self.camera_tracking_running = False
            self.btn_camera_tracking.setText("打开 camera_tracking 节点")
        self.update_manual_buttons()

    def update_manual_buttons(self):
        enabled = not self.camera_tracking_running and not self.hard_test_running
        self.btn_pitch_reset.setEnabled(enabled)
        self.btn_yaw_left.setEnabled(enabled)
        self.btn_yaw_right.setEnabled(enabled)
        self.btn_hard_test.setEnabled(not self.camera_tracking_running and not self.hard_test_running)

    def pitch_reset(self):
        threading.Thread(target=self.motor.pitch_reset, daemon=True).start()

    def yaw_left(self):
        threading.Thread(target=self.motor.yaw_left, daemon=True).start()

    def yaw_right(self):
        threading.Thread(target=self.motor.yaw_right, daemon=True).start()

    def yaw_stop(self):
        threading.Thread(target=self.motor.yaw_stop, daemon=True).start()

    def test_yaw_deadzone(self):
        try:
            duration = float(self.input_deadzone_time.text())
            if duration <= 0:
                raise ValueError
        except Exception:
            QtWidgets.QMessageBox.warning(self, "输入错误", "请输入有效的持续时间（秒）")
            return

        def run_test():
            self.btn_test_yaw_deadzone.setEnabled(False)
            self.yaw_left()
            time.sleep(duration)
            self.yaw_stop()
            self.btn_test_yaw_deadzone.setEnabled(True)

        threading.Thread(target=run_test, daemon=True).start()

    def start_ros_subscriber(self):
        def ros_spin():
            # 不要再调用 rclpy.init()
            def update_callback(topic, value):
                if topic == 'paper':
                    self.label_paper.setText(f"paper_center_pose: x={value[0]:.2f} y={value[1]:.2f} z={value[2]:.2f}")
                elif topic == 'odom':
                    self.odom_x = value[0]
                    self.odom_y = value[1]
                    self.label_odom.setText(f"wheel_odom: x={value[0]:.2f} y={value[1]:.2f} z={value[2]:.2f}")
            self.subscriber = RosDataSubscriber(update_callback)
            while rclpy.ok():
                rclpy.spin_once(self.subscriber, timeout_sec=0.1)
            # 不要再调用 rclpy.shutdown()
        self.ros_thread = threading.Thread(target=ros_spin, daemon=True)
        self.ros_thread.start()

    def open_admin_terminal(self):
        try:
            terminal_cmd = [
                'xfce4-terminal',
                '--hold',
                '--title', 'ROS管理员终端',
                '--working-directory', '/home/HwHiAiUser/ros',
                '-e', 'sudo bash -c "cd /home/HwHiAiUser/ros && source install/setup.sh && echo \\"ROS环境已加载，准备测试\\" && bash"'
            ]
            subprocess.Popen(terminal_cmd, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        except Exception as e:
            QtWidgets.QMessageBox.warning(self, "错误", f"无法打开管理员终端: {e}")

    def start_hard_test(self):
        if self.camera_tracking_running or self.hard_test_running:
            QtWidgets.QMessageBox.warning(self, "提示", "请先关闭 camera_tracking 节点和其他测试")
            return
        self.hard_test_running = True
        self.update_manual_buttons()
        self.hard_test_thread = threading.Thread(target=self._hard_test_logic, daemon=True)
        self.hard_test_thread.start()

    def _hard_test_logic(self):
        # 不再调用 rclpy.init()/shutdown，只用主线程的 node
        pub = self.hard_test_pub
        opened_debug_nodes = False
        try:
            # 第一阶段：x前进到2.2
            while self.odom_x < 2.2 and self.hard_test_running:
                twist = Twist()
                twist.linear.x = 2.5
                pub.publish(twist)
                time.sleep(0.05)
            # 第二阶段：x后退到1.8
            while self.odom_x > 1.8 and self.hard_test_running:
                twist = Twist()
                twist.linear.x = -2.5
                pub.publish(twist)
                time.sleep(0.05)
            # 打开 debug_pitch_pid.py 和 debug_yaw_pid.py（新终端，提前source环境）
            if not opened_debug_nodes:
                term_cmds = [
                    [
                        'xfce4-terminal',
                        '--hold',
                        '--title', 'debug_pitch_pid',
                        '--working-directory', '/home/HwHiAiUser/ros',
                        '-e', "sudo bash -c \"source /home/HwHiAiUser/ros/install/setup.sh && python3 '/home/HwHiAiUser/ros/src/Nodes/camera_tracking/debug_pitch_pid.py'\""
                    ],
                    [
                        'xfce4-terminal',
                        '--hold',
                        '--title', 'debug_yaw_pid',
                        '-e', "sudo bash -c \"source /home/HwHiAiUser/ros/install/setup.sh && python3 '/home/HwHiAiUser/ros/src/Nodes/camera_tracking/debug_yaw_pid.py'\""
                    ]
                ]
                for term_cmd in term_cmds:
                    subprocess.Popen(term_cmd, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
                opened_debug_nodes = True
            # 第三阶段：斜向移动到y=0.9
            while self.odom_y < 0.9 and self.hard_test_running:
                twist = Twist()
                twist.linear.x = -0.2
                twist.linear.y = 1.5
                pub.publish(twist)
                time.sleep(0.05)
            # 停止3秒
            twist = Twist()
            pub.publish(twist)
            time.sleep(4)
            # 循环左右移动
            while self.hard_test_running:

                time.sleep(4)
        except Exception as e:
            print(f"硬编码测试异常: {e}")
        finally:
            self.hard_test_running = False
            self.update_manual_buttons()

def main():
    app = QtWidgets.QApplication(sys.argv)
    rclpy.init()  # 必须先初始化
    hard_test_node = Node("ui_hard_test_node")
    ui = UiMain(hard_test_node)
    ui.show()
    sys.exit(app.exec_())

if __name__ == '__main__':
    main()
