import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose
import threading
import time
import sys
import select
import wiringpi

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
    def __init__(self, pin=6):
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

class DebugYawPIDNode(Node):
    def __init__(self):
        super().__init__('debug_yaw_pid')
        self.pid = PIDController()
        self.servo = YawServoController(pin=6)
        self.servo.start()
        self.last_error = 0.0
        self.subscription = self.create_subscription(
            Pose, 'paper_center_pose', self.pose_callback, 10)
        self.timer = self.create_timer(0.02, self.control_callback)
        self.filtered_error = 0.0
        self.deadzone = 3.0
        self.running = True
        print("输入新的PID参数（如: 0.1 0.2 0.3）并回车可动态调整，Ctrl+C退出。")
        threading.Thread(target=self.keyboard_thread, daemon=True).start()

    def pose_callback(self, msg):
        if msg.position.z == -1.0:
            self.filtered_error = 0.0
        else:
            self.filtered_error = msg.position.x

    def control_callback(self):
        error = self.filtered_error if abs(self.filtered_error) > self.deadzone else 0.0
        output = self.pid.update(error)
        # 限幅
        output = max(-1.0, min(1.0, output))
        self.servo.set_speed(output)
        # 可选：打印调试信息
        print(f"\r误差: {error:.2f} PID输出: {output:.3f} [P:{self.pid.kp:.3f} I:{self.pid.ki:.3f} D:{self.pid.kd:.3f}]", end='')

    def keyboard_thread(self):
        while self.running:
            try:
                if select.select([sys.stdin], [], [], 0.1)[0]:
                    line = sys.stdin.readline()
                    parts = line.strip().split()
                    if len(parts) == 3:
                        try:
                            kp, ki, kd = map(float, parts)
                            self.pid.kp = kp
                            self.pid.ki = ki
                            self.pid.kd = kd
                            self.pid.reset()
                            print(f"\nPID参数已更新: kp={kp}, ki={ki}, kd={kd}")
                        except Exception as e:
                            print(f"\n参数解析错误: {e}")
            except Exception:
                pass

    def destroy_node(self):
        self.running = False
        self.servo.stop()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = DebugYawPIDNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("\n退出调试。")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
