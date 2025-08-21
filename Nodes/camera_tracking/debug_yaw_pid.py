#原来方案

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose
import threading
import time
import sys
import wiringpi
import os

# 新增导入
from PyQt5 import QtWidgets, QtCore
import queue

class PIDController:
    def __init__(self, kp=0.0015, ki=0.0005, kd=0.0003):
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
        # 修改输出方向
        output = (self.kp * error + self.ki * self.integral + self.kd * derivative)
        self.prev_error = error
        self.prev_time = now
        return output

class YawServoController:
    def __init__(self, pin=19):
        self.pin = pin
        wiringpi.wiringPiSetup()
        wiringpi.pinMode(self.pin, wiringpi.GPIO.PWM_OUTPUT)
        wiringpi.pwmSetRange(self.pin, 3000000)
        # 初始化为7.5%占空比（1.5ms脉宽）
        self.set_duty(7.5)

    def start(self):
        pass  # 硬件PWM无需线程

    def stop(self):
        wiringpi.pwmWrite(self.pin, 0)

    def set_speed(self, speed):
        # speed范围[-1, 1]，线性映射到5%~10%，0为7.5%
        speed = max(-1.0, min(1.0, speed))
        percent = 7.5 + speed * 2.5  # -1->5%, 0->7.5%, 1->10%
        self.set_duty(percent)

    def set_duty(self, percent):
        percent = max(5.0, min(10.0, percent))
        pulse_ms = percent * 0.2  # 5%->1.0ms, 10%->2.0ms
        duty_cycle = pulse_ms / 20.0
        pwm_value = int(duty_cycle * 3000000)
        wiringpi.pwmWrite(self.pin, pwm_value)

# 新增：GUI线程安全参数队列
pid_update_queue = queue.Queue()

class DebugYawPIDNode(Node):
    def __init__(self):
        super().__init__('debug_yaw_pid')
        self.pid = PIDController()
        self.servo = YawServoController(pin=19)  # 修改为硬件PWM引脚19
        self.servo.start()
        self.last_error = 0.0
        self.last_output = 0.0  # 新增：记录最后一次的输出值
        self.subscription = self.create_subscription(
            Pose, 'paper_center_pose_smooth', self.pose_callback, 10)
        self.timer = self.create_timer(0.02, self.control_callback)
        self.filtered_error = 0.0
        self.deadzone = 3.0
        self.running = True
        # 齿隙建模
        self.yaw_dir = 0  # -1, 0, 1
        self.reverse_time = 0
        self.gear_delay = 0.030  # 30ms
        self.suppress_gear = False

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

        raw_error = self.filtered_error
        error = raw_error if abs(raw_error) > self.deadzone else 0.0
        # 齿隙建模：检测方向反转，反转后0.030s内抑制yaw误差
        cur_dir = 0
        if error > 0:
            cur_dir = 1
        elif error < 0:
            cur_dir = -1
        now = time.time()
        if cur_dir != 0 and cur_dir != self.yaw_dir:
            # 方向反转，抑制误差
            self.reverse_time = now
            self.suppress_gear = True
            self.yaw_dir = cur_dir
        if self.suppress_gear:
            if now - self.reverse_time < self.gear_delay:
                error = 0.0  # 抑制误差输入
            else:
                self.suppress_gear = False

        output = self.pid.update(error)
        output = max(-1.0, min(1.0, output))
        self.servo.set_speed(output)
        self.last_output = output  # 新增：保存当前输出

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
        self.setGeometry(100, 100, 300, 200)

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
        output = self.node.last_output  # 新增：获取当前输出
        self.label_info.setText(
            f"当前参数: P={kp:.5f} I={ki:.5f} D={kd:.30}\n"
            f"当前误差: {error:.2f}\n"
            f"当前舵机输出值: {output:.3f}"
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
