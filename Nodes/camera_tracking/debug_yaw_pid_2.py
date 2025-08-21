#二次占空比方案

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

# 死区占空比0.31

class PIDController:
    def __init__(self, kp=0.0005, ki=0.0, kd=0.0):
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
        self.lock = threading.Lock()
        wiringpi.wiringPiSetup()
        wiringpi.pinMode(self.pin, wiringpi.GPIO.PWM_OUTPUT)
        wiringpi.pwmSetRange(self.pin, 3000000)
        self.set_duty(7.5)
        self.deadzone_min = 7.19
        self.deadzone_max = 7.81
        self.deadzone_period = 0.1  # 0.1s周期
        self.deadzone_thread = None
        self.deadzone_running = False

    def start(self):
        pass

    def stop(self):
        self.deadzone_running = False
        if self.deadzone_thread and self.deadzone_thread.is_alive():
            self.deadzone_thread.join(timeout=0.2)
        wiringpi.pwmWrite(self.pin, 0)

    def set_speed(self, speed):
        # [-1,1]映射到5%~10%
        speed = max(-1.0, min(1.0, speed))
        percent = 5.0 + (5.0 * (speed + 1.0) / 2.0)
        percent = max(5.0, min(10.0, percent))

        # 死区范围内采用二次占空比
        if self.deadzone_min <= percent <= self.deadzone_max:
            slot_count = 5
            slot_size = (self.deadzone_max - self.deadzone_min) / slot_count
            slot_idx = int((percent - self.deadzone_min) / slot_size)
            slot_idx = min(slot_count - 1, max(0, slot_idx))
            active_time = self.deadzone_period * (slot_idx + 1) / slot_count
            inactive_time = self.deadzone_period - active_time

            # 启动死区交错线程
            self._start_deadzone_pwm(active_time, inactive_time, percent)
        else:
            self._stop_deadzone_pwm()
            self.set_duty(percent)

    def set_duty(self, percent):
        percent = max(5.0, min(10.0, percent))
        pulse_ms = percent * 0.2
        duty_cycle = pulse_ms / 20.0
        pwm_value = int(duty_cycle * 3000000)
        with self.lock:
            wiringpi.pwmWrite(self.pin, pwm_value)

    def _deadzone_pwm_loop(self):
        # 交错设置占空比
        while self.deadzone_running:
            # 判断是负死区还是正死区
            if self.deadzone_active_percent < 7.5:
                self.set_duty(7.18)  # 负死区
            else:
                self.set_duty(7.82)  # 正死区
            time.sleep(self.deadzone_active_time)
            self.set_duty(7.5)
            time.sleep(self.deadzone_inactive_time)

    def _start_deadzone_pwm(self, active_time, inactive_time, percent):
        if self.deadzone_running:
            # 已在死区线程，更新参数
            self.deadzone_active_time = active_time
            self.deadzone_inactive_time = inactive_time
            self.deadzone_active_percent = percent
            return
        self.deadzone_running = True
        self.deadzone_active_time = active_time
        self.deadzone_inactive_time = inactive_time
        self.deadzone_active_percent = percent
        self.deadzone_thread = threading.Thread(
            target=self._deadzone_pwm_loop, daemon=True)
        self.deadzone_thread.start()

    def _stop_deadzone_pwm(self):
        self.deadzone_running = False
        if self.deadzone_thread and self.deadzone_thread.is_alive():
            self.deadzone_thread.join(timeout=0.2)
        self.deadzone_thread = None

# 新增：GUI线程安全参数队列
pid_update_queue = queue.Queue()

class DebugYawPIDNode(Node):
    def __init__(self):
        super().__init__('debug_yaw_pid')
        self.pid = PIDController()
        self.servo = YawServoController(pin=19)
        self.servo.start()
        self.last_error = 0.0
        self.last_output = 0.0
        self.subscription = self.create_subscription(
            Pose, 'paper_center_pose_smooth', self.pose_callback, 10)
        self.filtered_error = 0.0
        self.deadzone = 3.0
        self.running = True
        # 齿隙建模（已注释）
        # self.yaw_dir = 0
        # self.reverse_time = 0
        # self.gear_delay = 0.030
        # self.suppress_gear = False

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

        # 齿隙建模相关逻辑已注释
        # cur_dir = 0
        # if error > 0:
        #     cur_dir = 1
        # elif error < 0:
        #     cur_dir = -1
        # now = time.time()
        # if cur_dir != 0 and cur_dir != self.yaw_dir:
        #     # 方向反转，抑制误差
        #     self.reverse_time = now
        #     self.suppress_gear = True
        #     self.yaw_dir = cur_dir
        # if self.suppress_gear:
        #     if now - self.reverse_time < self.gear_delay:
        #         error = 0.0  # 抑制误差输入
        #     else:
        #         self.suppress_gear = False

        # PID只在收到话题时更新
        output = self.pid.update(error)
        output = max(-1.0, min(1.0, output))
        self.servo.set_speed(output)
        self.last_output = output

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
