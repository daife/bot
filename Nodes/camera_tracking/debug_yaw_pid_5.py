import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose
import threading
import time
import sys
from PyQt5 import QtWidgets, QtCore
import queue
import serial
import struct


pid_update_queue = queue.Queue()

# 串口初始化（请根据实际设备号修改）
uart1 = serial.Serial('/dev/ttyAMA2', baudrate=115200, bytesize=8, parity='N', stopbits=1, timeout=0.1)

def Emm_V5_En_Control(addr, state, snF):
    cmd = bytearray(16)
    cmd[0] = addr
    cmd[1] = 0xF3
    cmd[2] = 0xAB
    cmd[3] = 0x01 if state else 0x00
    cmd[4] = 0x01 if snF else 0x00
    cmd[5] = 0x6B
    uart1.write(cmd[:6])

def Emm_V5_Vel_Control(addr, dir, vel, acc, snF):
    cmd = bytearray(16)
    cmd[0] = addr
    cmd[1] = 0xF6
    cmd[2] = dir
    cmd[3] = (vel >> 8) & 0xFF
    cmd[4] = vel & 0xFF
    cmd[5] = acc
    cmd[6] = 0x01 if snF else 0x00
    cmd[7] = 0x6B
    uart1.write(cmd[:8])

def Emm_V5_Read_Sys_Params(addr, s):
    i = 0
    cmd = bytearray(16)
    cmd[i] = addr
    i += 1
    func_codes = {
        'S_CPOS': 0x36,
    }
    if s in func_codes:
        cmd[i] = func_codes[s]
        i += 1
    cmd[i] = 0x6B
    i += 1
    uart1.write(cmd[:i])

def Emm_V5_Receive_Data(uart):
    rxCmd = bytearray(128)
    i = 0
    lTime = cTime = int(time.time() * 1000)
    while True:
        if uart.in_waiting:
            if i < 128:
                rxCmd[i] = uart.read(1)[0]
                i += 1
                lTime = int(time.time() * 1000)
        else:
            cTime = int(time.time() * 1000)
            if cTime - lTime > 100:
                hex_data = ' '.join(['{:02x}'.format(b) for b in rxCmd[:i]])
                hex_data = hex_data.strip('00 ')
                if hex_data and hex_data[0] != '0':
                    hex_data = '0' + hex_data
                return hex_data, len(hex_data.replace(' ', '')) // 2

def get_motor_position(addr):
    # 读取实时位置
    Emm_V5_Read_Sys_Params(addr, 'S_CPOS')
    time.sleep(0.01)
    data, count = Emm_V5_Receive_Data(uart1)
    if not data or count < 7:
        return None
    data_hex = data.split()
    try:
        if int(data_hex[0], 16) == addr and int(data_hex[1], 16) == 0x36:
            pos = struct.unpack('>I', bytes.fromhex(''.join(data_hex[3:7])))[0]
            angle = float(pos) * 360.0 / 65536.0
            if int(data_hex[2], 16):
                angle = -angle
            return angle
    except Exception:
        pass
    return None

class PIDController:
    def __init__(self, kp=0.1, ki=0.001, kd=0.0):
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
        super().__init__('debug_yaw_pid_5')
        self.pid = PIDController()
        self.subscription = self.create_subscription(
            Pose, 'paper_center_pose_smooth', self.pose_callback, 10)
        self.filtered_error = 0.0
        self.deadzone = 3.0
        self.addr = 1
        self.dir = 0
        self.acc = 0
        self.snF = False
        self.vel_max = 2000
        self.vel_min = 0
        self.current_vel = 0
        self.current_angle = 0.0
        self._stop_flag = False
        # 初始化电机
        Emm_V5_En_Control(self.addr, True, False)
        time.sleep(0.1)
        Emm_V5_Vel_Control(
            addr=self.addr,
            dir=self.dir,
            vel=self.current_vel,
            acc=self.acc,
            snF=self.snF
        )
        # 启动实时位置线程
        self.position_thread = threading.Thread(target=self._update_position_loop, daemon=True)
        self.position_thread.start()

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

        vel = int(self.pid.update(error))
        if vel >= 0:
            self.dir = 0
        else:
            self.dir = 1
        vel = abs(vel)
        vel = max(self.vel_min, min(self.vel_max, vel))
        if vel != self.current_vel or error == 0.0:
            Emm_V5_Vel_Control(
                addr=self.addr,
                dir=self.dir,
                vel=vel,
                acc=self.acc,
                snF=self.snF
            )
            self.current_vel = vel

    def _update_position_loop(self):
        while not self._stop_flag:
            angle = get_motor_position(self.addr)
            if angle is not None:
                self.current_angle = angle
            time.sleep(0.1)

    def destroy_node(self):
        self._stop_flag = True
        super().destroy_node()

class PIDGui(QtWidgets.QWidget):
    def __init__(self, node: DebugYawPIDStepMotorNode):
        super().__init__()
        self.node = node
        self.setWindowTitle("步进电机PID速度调试")
        self.setGeometry(100, 100, 300, 250)

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
        vel = self.node.current_vel
        angle = self.node.current_angle
        self.label_info.setText(
            f"当前参数: P={kp:.5f} I={ki:.5f} D={kd:.5f}\n"
            f"当前误差: {error:.2f}\n"
            f"当前速度(RPM): {vel}\n"
            f"实时位置角度: {angle:.2f}°"
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
