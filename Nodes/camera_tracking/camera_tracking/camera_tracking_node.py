import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose
import time
import serial
import struct
import wiringpi

class PIDController:
    def __init__(self, kp, ki, kd):
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
    def __init__(self, pin=19):
        self.pin = pin
        wiringpi.wiringPiSetup()
        wiringpi.pinMode(self.pin, wiringpi.GPIO.PWM_OUTPUT)
        wiringpi.pwmSetRange(self.pin, 3000000)
        # 初始化为7.5%占空比（1.5ms脉宽）
        self.set_pulse_ms(1.5)

    def set_speed(self, speed):
        speed = max(-1.0, min(1.0, speed))
        pulse_ms = 1.5 + speed
        self.set_pulse_ms(pulse_ms)

    def set_pulse_ms(self, pulse_ms):
        pulse_ms = max(1.0, min(2.0, pulse_ms))
        duty_cycle = pulse_ms / 20.0
        pwm_value = int(duty_cycle * 3000000)
        wiringpi.pwmWrite(self.pin, pwm_value)

    def stop(self):
        wiringpi.pwmWrite(self.pin, 0)

class PitchSerialController:
    def __init__(self, port='/dev/ttyAMA0', baudrate=115200):
        self.serial_port = serial.Serial(port, baudrate=baudrate, timeout=0.1)

    def send_y(self, y):
        data = bytearray(14)
        data[0] = 0xAA
        struct.pack_into('<f', data, 1, float(y))
        data[13] = 0x0A
        self.serial_port.write(data)

    def close(self):
        try:
            self.serial_port.close()
        except Exception:
            pass

class CameraTrackingNode(Node):
    def __init__(self):
        super().__init__('camera_tracking')
        # 固定PID参数
        self.yaw_pid = PIDController(kp=0.0015, ki=0.0005, kd=0.0003)
        self.pitch_pid = PIDController(kp=0.0005, ki=0.0, kd=0.0)
        self.servo = YawServoController(pin=19)
        self.pitch_serial = PitchSerialController()
        self.last_yaw_error = 0.0
        self.last_pitch_error = 0.0
        self.deadzone_yaw = 3.0
        self.subscription = self.create_subscription(
            Pose, 'paper_center_pose', self.pose_callback, 10)
        self.timer = self.create_timer(0.02, self.control_callback)

    def pose_callback(self, msg):
        if msg.position.z == -1.0:
            self.last_yaw_error = 0.0
            self.last_pitch_error = 0.0
        else:
            self.last_yaw_error = msg.position.x
            self.last_pitch_error = msg.position.y

    def control_callback(self):
        # Yaw控制
        yaw_error = self.last_yaw_error if abs(self.last_yaw_error) > self.deadzone_yaw else 0.0
        yaw_output = self.yaw_pid.update(yaw_error)
        yaw_output = max(-1.0, min(1.0, yaw_output))
        self.servo.set_speed(yaw_output)
        # Pitch控制
        pitch_output = self.pitch_pid.update(self.last_pitch_error)
        self.pitch_serial.send_y(pitch_output)

    def destroy_node(self):
        self.servo.stop()
        self.pitch_serial.close()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = CameraTrackingNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("\n退出 camera_tracking。")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
