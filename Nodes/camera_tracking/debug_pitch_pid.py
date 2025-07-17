import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose
import threading
import sys
import serial
from PyQt5 import QtWidgets, QtCore
import queue
import struct

# 参数更新队列
pid_update_queue = queue.Queue()

class DebugPitchPIDNode(Node):
    def __init__(self):
        super().__init__('debug_pitch_pid')
        self.last_y = 0.0
        self.last_error = 0.0
        self.pid_params = [0.0005, 0.000, 0.000]  # 默认P,I,D
        self.subscription = self.create_subscription(
            Pose, 'paper_center_pose', self.pose_callback, 10)
        self.serial_port = serial.Serial('/dev/ttyAMA0', baudrate=115200, timeout=0.1)
        self.running = True
        # 初始化时发送一次PID参数
        self.send_pid(*self.pid_params)

    def pose_callback(self, msg):
        if msg.position.z == -1.0:
            self.last_y = 0.0
        else:
            self.last_y = msg.position.y
        self.last_error = self.last_y
        self.send_y(self.last_y)

    def send_y(self, y):
        # 格式: 0xAA, 0x0A, float(y), 后面补0到14字节
        data = bytearray(14)
        data[0] = 0xAA
        struct.pack_into('<f', data, 1, float(y))
        data[13] = 0x0A
        self.serial_port.write(data)

    def send_pid(self, kp, ki, kd):
        # 格式: 0xAA, 0x0B, float(kp), float(ki), float(kd), 补齐14字节
        data = bytearray(14)
        data[0] = 0xBB
        
        struct.pack_into('<f', data, 1, float(kp))
        struct.pack_into('<f', data, 5, float(ki))
        struct.pack_into('<f', data, 9, float(kd))
        data[13] = 0x0B
        self.serial_port.write(data)

    def check_pid_update(self):
        # 检查参数更新
        try:
            while not pid_update_queue.empty():
                new_params = pid_update_queue.get_nowait()
                if new_params is not None:
                    kp, ki, kd = new_params
                    self.pid_params = [kp, ki, kd]
                    self.send_pid(kp, ki, kd)
        except Exception:
            pass

    def destroy_node(self):
        self.running = False
        try:
            self.serial_port.close()
        except Exception:
            pass
        super().destroy_node()

class PIDGui(QtWidgets.QWidget):
    def __init__(self, node: DebugPitchPIDNode):
        super().__init__()
        self.node = node
        self.setWindowTitle("Pitch PID参数调试")
        self.setGeometry(100, 100, 350, 200)

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
        kp, ki, kd = self.node.pid_params
        error = self.node.last_error
        self.label_info.setText(
            f"当前参数: P={kp:.5f} I={ki:.5f} D={kd:.5f}\n"
            f"当前偏差(y): {error:.2f}"
        )

    def update_pid(self):
        kp, ki, kd = self.node.pid_params
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
    node = DebugPitchPIDNode()

    def qt_thread_func():
        app = QtWidgets.QApplication(sys.argv)
        gui = PIDGui(node)
        gui.show()
        app.exec_()
        node.destroy_node()

    qt_thread = threading.Thread(target=qt_thread_func, daemon=True)
    qt_thread.start()

    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.1)
            node.check_pid_update()
    except KeyboardInterrupt:
        print("\n退出调试。")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
