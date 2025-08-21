import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose
import threading
import time
import sys
from PyQt5 import QtWidgets, QtCore
import queue

# 引入步进电机控制
import sys as _sys
_sys.path.append('/home/HwHiAiUser/Desktop/bujin')
import control

pid_update_queue = queue.Queue()

class PIDController:
    def __init__(self, kp=0.1, ki=0.0, kd=0.0):
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

class DebugYawPIDStepMotorNode(Node):
    def __init__(self):
        super().__init__('debug_yaw_pid_4')
        self.pid = PIDController()
        self.subscription = self.create_subscription(
            Pose, 'paper_center_pose_smooth', self.pose_callback, 10)
        self.last_error = 0.0
        self.filtered_error = 0.0
        self.deadzone = 3.0
        self.running = True
        self.addr = 1
        self.dir = 0  # 始终顺时针
        self.vel = 1000
        self.acc = 0
        self.raF = True
        self.snF = False
        self.pulse_min = 0
        self.pulse_max = 2800
        self.current_pulse = 900  # 初始化脉冲数
        # 初始化电机
        control.Emm_V5_En_Control(self.addr, True, False)
        time.sleep(0.1)
        control.Emm_V5_Pos_Control(
            addr=self.addr,
            dir=self.dir,
            vel=self.vel,
            acc=self.acc,
            clk=self.current_pulse,
            raF=self.raF,
            snF=self.snF
        )

    def pose_callback(self, msg):
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

        # PID输出为脉冲数增量
        pulse_delta = int(self.pid.update(error))
        # 顺时针为正方向，脉冲数增大
        new_pulse = self.current_pulse + pulse_delta
        new_pulse = max(self.pulse_min, min(self.pulse_max, new_pulse))
        if new_pulse != self.current_pulse:
            control.Emm_V5_Pos_Control(
                addr=self.addr,
                dir=self.dir,
                vel=self.vel,
                acc=self.acc,
                clk=new_pulse,
                raF=self.raF,
                snF=self.snF
            )
            self.current_pulse = new_pulse

    def destroy_node(self):
        self.running = False
        super().destroy_node()

class PIDGui(QtWidgets.QWidget):
    def __init__(self, node: DebugYawPIDStepMotorNode):
        super().__init__()
        self.node = node
        self.setWindowTitle("步进电机PID参数调试")
        self.setGeometry(100, 100, 300, 220)

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
        pulse = self.node.current_pulse
        self.label_info.setText(
            f"当前参数: P={kp:.5f} I={ki:.5f} D={kd:.5f}\n"
            f"当前误差: {error:.2f}\n"
            f"当前脉冲数: {pulse}"
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
    node = DebugYawPIDStepMotorNode()

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