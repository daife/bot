import sys
import wiringpi
from PyQt5 import QtWidgets, QtCore

class PitchServo:
    def __init__(self, pin=19, min_angle=0, max_angle=180):
        self.pin = pin
        self.min_angle = min_angle
        self.max_angle = max_angle
        self.current_angle = 90
        wiringpi.wiringPiSetup()
        wiringpi.pinMode(self.pin, wiringpi.GPIO.PWM_OUTPUT)
        wiringpi.pwmSetRange(self.pin, 3000000)
        self.set_angle(self.current_angle)

    def set_angle(self, angle):
        self.current_angle = max(self.min_angle, min(self.max_angle, angle))
        # 0度->0.5ms, 180度->2.5ms, 20ms周期
        pulse_ms = 0.5 + (self.current_angle / 180.0) * 2.0
        duty_cycle = pulse_ms / 20.0
        pwm_value = int(duty_cycle * 3000000)
        wiringpi.pwmWrite(self.pin, pwm_value)

    def get_angle(self):
        return self.current_angle

    def get_duty(self):
        # 返回当前占空比（百分比）
        pulse_ms = 0.5 + (self.current_angle / 180.0) * 2.0
        return round((pulse_ms / 20.0) * 100, 2)

class ServoDebugGui(QtWidgets.QWidget):
    def __init__(self, servo: PitchServo):
        super().__init__()
        self.servo = servo
        self.setWindowTitle("硬件PWM舵机调试")
        self.setGeometry(100, 100, 300, 150)

        layout = QtWidgets.QVBoxLayout()

        self.label_info = QtWidgets.QLabel()
        layout.addWidget(self.label_info)

        form_layout = QtWidgets.QFormLayout()
        self.input_angle = QtWidgets.QLineEdit()
        form_layout.addRow("设置角度 (0-180):", self.input_angle)
        layout.addLayout(form_layout)

        self.btn_set = QtWidgets.QPushButton("设置舵机角度")
        layout.addWidget(self.btn_set)
        self.btn_set.clicked.connect(self.set_angle)

        self.setLayout(layout)

        self.timer = QtCore.QTimer()
        self.timer.timeout.connect(self.refresh_info)
        self.timer.start(200)

    def refresh_info(self):
        angle = self.servo.get_angle()
        duty = self.servo.get_duty()
        self.label_info.setText(
            f"当前舵机角度: {angle:.1f}°\n当前PWM占空比: {duty:.2f}%"
        )

    def set_angle(self):
        try:
            val = float(self.input_angle.text())
            self.servo.set_angle(val)
            self.input_angle.clear()
        except Exception as e:
            QtWidgets.QMessageBox.warning(self, "输入错误", f"角度解析错误: {e}")

def main():
    servo = PitchServo(pin=19, min_angle=0, max_angle=180)
    app = QtWidgets.QApplication(sys.argv)
    gui = ServoDebugGui(servo)
    gui.show()
    sys.exit(app.exec_())

if __name__ == '__main__':
    main()
