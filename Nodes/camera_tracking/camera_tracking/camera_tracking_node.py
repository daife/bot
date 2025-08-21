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
        self.set_duty(7.5)

    def set_duty(self, percent):
        percent = max(5.0, min(10.0, percent))
        pulse_ms = percent * 0.2  # 5%->1.0ms, 10%->2.0ms
        duty_cycle = pulse_ms / 20.0
        pwm_value = int(duty_cycle * 3000000)
        wiringpi.pwmWrite(self.pin, pwm_value)

    def set_speed(self, speed):
        # speed: -1~1, 0为停止
        percent = 7.5 + speed * 2.5  # -1->5%, 0->7.5%, 1->10%
        self.set_duty(percent)

    def stop(self):
        self.set_duty(7.5)

class PitchSerialController:
    def __init__(self, port='/dev/ttyAMA0', baudrate=115200):
        # 只在初始化时打开串口
        self.serial_port = serial.Serial(port, baudrate=baudrate, timeout=0.1)

    def send_y(self, y):
        # 只发送y的偏差，不做PID调节
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
        self.yaw_pid = PIDController(kp=0.0015, ki=0.0005, kd=0.0003)
        self.servo = YawServoController(pin=19)
        self.pitch_serial = PitchSerialController()
        self.last_yaw_error = 0.0
        self.last_pitch_error = 0.0
        self.deadzone_yaw = 2.0
        self.subscription = self.create_subscription(
            Pose, 'paper_center_pose', self.pose_callback, 10)
        self.timer = self.create_timer(0.02, self.control_callback)
        # 齿隙建模
        self.yaw_dir = 0  # -1, 0, 1
        self.reverse_time = 0
        self.gear_delay = 0.035  # 35ms
        self.suppress_gear = False

    def pose_callback(self, msg):
        if msg.position.z == -1.0:
            self.last_yaw_error = 0.0
            self.last_pitch_error = 0.0
            self.servo.set_speed(0.0)
            self.pitch_serial.send_y(0.0)
        else:
            self.last_yaw_error = msg.position.x
            self.last_pitch_error = msg.position.y

    def control_callback(self):
        # 齿隙建模：检测方向反转，反转后0.035s内抑制yaw误差
        raw_error = self.last_yaw_error
        # 死区
        error = raw_error if abs(raw_error) > self.deadzone_yaw else 0.0
        # 当前方向
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
        yaw_output = self.yaw_pid.update(error)
        yaw_output = max(-1.0, min(1.0, yaw_output))
        self.servo.set_speed(yaw_output)
        # Pitch控制：直接发送y的偏差
        self.pitch_serial.send_y(self.last_pitch_error)

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
