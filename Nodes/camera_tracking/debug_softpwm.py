import threading
import time
import sys
import wiringpi
import os
from PyQt5 import QtWidgets, QtCore

class SoftPWMController:
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
        # 提升进程优先级
        try:
            os.nice(-20)  # 需要root权限，-20为最高优先级
        except Exception as e:
            print(f"提升优先级失败: {e}")
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
                print(f"[SoftPWM] PWM线程错误: {e}")
                time.sleep(0.02)

class SoftPWMGui(QtWidgets.QWidget):
    def __init__(self, pwm: SoftPWMController):
        super().__init__()
        self.pwm = pwm
        self.setWindowTitle("SoftPWM调试")
        self.setGeometry(100, 100, 300, 120)

        layout = QtWidgets.QVBoxLayout()

        self.label_info = QtWidgets.QLabel()
        layout.addWidget(self.label_info)

        form_layout = QtWidgets.QFormLayout()
        self.input_speed = QtWidgets.QLineEdit()
        form_layout.addRow("输出值(-1~1):", self.input_speed)
        layout.addLayout(form_layout)

        self.btn_update = QtWidgets.QPushButton("更新输出")
        layout.addWidget(self.btn_update)
        self.btn_update.clicked.connect(self.update_speed)

        self.setLayout(layout)

        self.timer = QtCore.QTimer()
        self.timer.timeout.connect(self.refresh_info)
        self.timer.start(100)

    def refresh_info(self):
        speed = self.pwm.current_speed
        self.label_info.setText(f"当前输出值: {speed:.3f}")

    def update_speed(self):
        try:
            val = float(self.input_speed.text())
            self.pwm.set_speed(val)
            self.input_speed.clear()
        except Exception as e:
            QtWidgets.QMessageBox.warning(self, "参数错误", f"输出值解析错误: {e}")

def main():
    pwm = SoftPWMController(pin=7)
    pwm.start()

    app = QtWidgets.QApplication(sys.argv)
    gui = SoftPWMGui(pwm)
    gui.show()
    app.exec_()
    pwm.stop()

if __name__ == '__main__':
    main()
