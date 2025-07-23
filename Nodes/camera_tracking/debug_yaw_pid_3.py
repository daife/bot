import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose, Twist
import threading
import time
import sys
from PyQt5 import QtWidgets, QtCore
import queue

pid_update_queue = queue.Queue()

class PIDController:
    def __init__(self, kp=0.012, ki=0.001, kd=0.00001):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.reset()

    def reset(self):
        self.prev_error = 0.0
        self.integral = 0.0
        self.prev_time = time.time()

    def update(self, error):
        now = time.time()
        dt = now - self.prev_time
        if dt <= 0: dt = 0.001
        self.integral += error * dt
        derivative = (error - self.prev_error) / dt
        output = (self.kp * error + self.ki * self.integral + self.kd * derivative)
        self.prev_error = error
        self.prev_time = now
        return output

class DebugYawPIDNode(Node):
    def __init__(self):
        super().__init__('debug_yaw_pid_3')
        self.pid = PIDController()
        self.subscription = self.create_subscription(
            Pose, 'paper_center_pose_smooth', self.pose_callback, 10)
        self.cmd_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.last_error = 0.0
        self.last_output = 0.0
        self.filtered_error = 0.0
        self.deadzone = 3.0
        self.running = True

    def pose_callback(self, msg):
        # 检查是否有新的PID参数更新
        try:
            while not pid_update_queue.empty():
                new_params = pid_update_queue.get_nowait()
                if new_params is not None:
                    kp, ki, kd = new_params
                    self.pid.kp = kp
                    self.pid.ki = ki
                    self.pid.kd = kd
                    self.pid.reset()
        except Exception:
            pass

        if msg.position.z == -1.0:
            self.filtered_error = 0.0
        else:
            self.filtered_error = msg.position.x

        raw_error = self.filtered_error
        error = raw_error if abs(raw_error) > self.deadzone else 0.0

        output = self.pid.update(error)
        # 限制最大角速度
        output = max(-1.0, min(1.0, output))
        self.last_output = output

        twist = Twist()
        twist.linear.x = 0.0
        twist.linear.y = 0.0
        twist.linear.z = 0.0
        twist.angular.x = 0.0
        twist.angular.y = 0.0
        twist.angular.z = -output  # 只控制自转
        self.cmd_pub.publish(twist)

    def destroy_node(self):
        self.running = False
        super().destroy_node()

class PIDGui(QtWidgets.QWidget):
    def __init__(self, node: DebugYawPIDNode):
        super().__init__()
        self.node = node
        self.setWindowTitle("PID参数调试")
        self.setGeometry(100, 100, 300, 200)

        layout = QtWidgets.QVBoxLayout()
        self.label_info = QtWidgets.QLabel()
        layout.addWidget(self.label_info)

        form_layout = QtWidgets.QFormLayout()
        self.input_kp = QtWidgets.QLineEdit()
        self.input_ki = QtWidgets.QLineEdit()
        self.input_kd = QtWidgets.QLineEdit()
        form_layout.addRow("P:", self.input_kp)
        form_layout.addRow("I:", self.input_ki)
        form_layout.addRow("D:", self.input_kd)
        layout.addLayout(form_layout)

        self.btn_update = QtWidgets.QPushButton("更新参数")
        layout.addWidget(self.btn_update)
        self.btn_update.clicked.connect(self.update_pid)

        self.setLayout(layout)

        self.timer = QtCore.QTimer()
        self.timer.timeout.connect(self.refresh_info)
        self.timer.start(100)

    def refresh_info(self):
        kp = self.node.pid.kp
        ki = self.node.pid.ki
        kd = self.node.pid.kd
        error = self.node.filtered_error
        output = self.node.last_output
        self.label_info.setText(
            f"当前参数: P={kp:.5f} I={ki:.5f} D={kd:.5f}\n"
            f"当前误差: {error:.2f}\n"
            f"当前角速度输出: {output:.3f}"
        )

    def update_pid(self):
        kp = self.node.pid.kp
        ki = self.node.pid.ki
        kd = self.node.pid.kd
        try:
            if self.input_kp.text().strip():
                kp = float(self.input_kp.text())
            if self.input_ki.text().strip():
                ki = float(self.input_ki.text())
            if self.input_kd.text().strip():
                kd = float(self.input_kd.text())
            pid_update_queue.put((kp, ki, kd))
            self.input_kp.clear()
            self.input_ki.clear()
            self.input_kd.clear()
        except Exception as e:
            QtWidgets.QMessageBox.warning(self, "参数错误", f"参数解析错误: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = DebugYawPIDNode()

    def qt_thread_func():
        app = QtWidgets.QApplication(sys.argv)
        gui = PIDGui(node)
        gui.show()
        app.exec_()
        node.destroy_node()

    qt_thread = threading.Thread(target=qt_thread_func, daemon=True)
    qt_thread.start()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("\n退出调试。")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
