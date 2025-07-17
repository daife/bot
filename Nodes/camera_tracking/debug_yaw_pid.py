import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose
import threading
import time
import sys
import wiringpi

# 新增导入
from PyQt5 import QtWidgets, QtCore
import queue

class PIDController:
    def __init__(self, kp=0.01, ki=0.001, kd=0.005):
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
        output = self.kp * error + self.ki * self.integral + self.kd * derivative
        self.prev_error = error
        self.prev_time = now
        return output

class YawServoController:
    def __init__(self, pin=7):
        self.pin = pin
        self.current_speed = 0.0
        self.running = False
        self.thread = None
        wiringpi.wiringPiSetup()
        wiringpi.pinMode(self.pin, wiringpi.GPIO.OUTPUT)
        wiringpi.digitalWrite(self.pin, wiringpi.GPIO.LOW)

    def start(self):
        if self.running: return
        self.running = True
        self.thread = threading.Thread(target=self._worker, daemon=True)
        self.thread.start()

    def stop(self):
        self.running = False
        if self.thread: self.thread.join(timeout=1.0)
        wiringpi.digitalWrite(self.pin, wiringpi.GPIO.LOW)

    def set_speed(self, speed):
        self.current_speed = max(-1.0, min(1.0, speed))

    def _worker(self):
        while self.running:
            try:
                pulse_ms = 1.5 + self.current_speed * 1.0
                pulse_s = pulse_ms / 1000.0
                period = 0.02
                wiringpi.digitalWrite(self.pin, wiringpi.GPIO.HIGH)
                time.sleep(pulse_s)
                wiringpi.digitalWrite(self.pin, wiringpi.GPIO.LOW)
                time.sleep(max(0, period - pulse_s))
            except Exception as e:
                print(f"[YawServo] PWM线程错误: {e}")
                time.sleep(0.02)

# 新增：GUI线程安全参数队列
pid_update_queue = queue.Queue()

class DebugYawPIDNode(Node):
    def __init__(self):
        super().__init__('debug_yaw_pid')
        self.pid = PIDController()
        self.servo = YawServoController(pin=7)
        self.servo.start()
        self.last_error = 0.0
        self.subscription = self.create_subscription(
            Pose, 'paper_center_pose', self.pose_callback, 10)
        self.timer = self.create_timer(0.02, self.control_callback)
        self.filtered_error = 0.0
        self.deadzone = 3.0
        self.running = True

    def pose_callback(self, msg):
        if msg.position.z == -1.0:
            self.filtered_error = 0.0
        else:
            self.filtered_error = msg.position.x

    def control_callback(self):
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

        error = self.filtered_error if abs(self.filtered_error) > self.deadzone else 0.0
        output = self.pid.update(error)
        output = max(-1.0, min(1.0, output))
        self.servo.set_speed(output)
        # 打印调试信息（可选）
        #print(f"\r误差: {error:.2f} PID输出: {output:.3f} [P:{self.pid.kp:.3f} I:{self.pid.ki:.3f} D:{self.pid.kd:.3f}]", end='')

    def destroy_node(self):
        self.running = False
        self.servo.stop()
        super().destroy_node()

# 新增：Qt5界面
class PIDGui(QtWidgets.QWidget):
    def __init__(self, node: DebugYawPIDNode):
        super().__init__()
        self.node = node
        self.setWindowTitle("PID参数调试")
        self.setGeometry(100, 100, 350, 200)

        layout = QtWidgets.QVBoxLayout()

        # 当前参数和误差显示
        self.label_info = QtWidgets.QLabel()
        layout.addWidget(self.label_info)

        # 输入框
        form_layout = QtWidgets.QFormLayout()
        self.input_kp = QtWidgets.QLineEdit()
        self.input_ki = QtWidgets.QLineEdit()
        self.input_kd = QtWidgets.QLineEdit()
        form_layout.addRow("P:", self.input_kp)
        form_layout.addRow("I:", self.input_ki)
        form_layout.addRow("D:", self.input_kd)
        layout.addLayout(form_layout)

        # 更新按钮
        self.btn_update = QtWidgets.QPushButton("更新参数")
        layout.addWidget(self.btn_update)
        self.btn_update.clicked.connect(self.update_pid)

        self.setLayout(layout)

        # 定时刷新显示
        self.timer = QtCore.QTimer()
        self.timer.timeout.connect(self.refresh_info)
        self.timer.start(100)

    def refresh_info(self):
        kp = self.node.pid.kp
        ki = self.node.pid.ki
        kd = self.node.pid.kd
        error = self.node.filtered_error
        self.label_info.setText(
            f"当前参数: P={kp:.3f} I={ki:.3f} D={kd:.3f}\n当前误差: {error:.2f}"
        )

    def update_pid(self):
        kp = self.node.pid.kp
        ki = self.node.pid.ki
        kd = self.node.pid.kd
        # 只更新填写的参数
        try:
            if self.input_kp.text().strip():
                kp = float(self.input_kp.text())
            if self.input_ki.text().strip():
                ki = float(self.input_ki.text())
            if self.input_kd.text().strip():
                kd = float(self.input_kd.text())
            pid_update_queue.put((kp, ki, kd))
            # 清空输入框
            self.input_kp.clear()
            self.input_ki.clear()
            self.input_kd.clear()
        except Exception as e:
            QtWidgets.QMessageBox.warning(self, "参数错误", f"参数解析错误: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = DebugYawPIDNode()

    # 启动Qt界面（单独线程）
    def qt_thread_func():
        app = QtWidgets.QApplication(sys.argv)
        gui = PIDGui(node)
        gui.show()
        app.exec_()
        # 关闭ROS节点
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
