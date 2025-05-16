import wiringpi
from wiringpi import GPIO
import time
import threading
from arm_control_interfaces.action import MoveArm
import math

class Arm:
    def __init__(self, pwm_pin=19, min_angle=-30, max_angle=15, speed=90):
        self.pwm_pin = pwm_pin
        self.min_angle = min_angle
        self.max_angle = max_angle
        self.speed = speed  # degrees per second
        self._lock = threading.RLock()
        self._cancel = False

        self.current_pose = 0.0
        self.goal_pose = 0.0
        self._start_pose = 0.0
        self._start_time = time.time()
        self._cancel_time = None

        wiringpi.wiringPiSetup()
        wiringpi.pinMode(self.pwm_pin, GPIO.PWM_OUTPUT)
        wiringpi.pwmSetRange(19,3000000)
        self.set_pwm(0)


    def set_pwm(self, theta):
        pwm_val = int(((2/3 + (theta-3)/270) * 200 + 50) * 30000 / 20)#减去一个数，零偏修正
        wiringpi.pwmWrite(self.pwm_pin, pwm_val)

    def set_goal(self, pose):
        print(f"[Arm.set_goal] called with pose={pose}")
        with self._lock:
            self._cancel = False
            self._cancel_time = None
            self._start_pose = self.get_current_pose()
            print(f"[Arm.set_goal] _start_pose={self._start_pose}")
            self.current_pose = self._start_pose
            self.goal_pose = max(self.min_angle, min(self.max_angle, pose))
            self._start_time = time.time()
            self.set_pwm(self.goal_pose)
            print("[Arm.set_goal] set_pwm called, goal set")

    def get_current_pose(self):
        with self._lock:
            #print("[Arm.get_current_pose] acquired lock")
            if self._cancel and self._cancel_time is not None:
                return float(self.current_pose)
            now = time.time()
            elapsed = now - self._start_time
            delta = self.goal_pose - self._start_pose
            duration = abs(delta) / self.speed if self.speed > 0 else 0
            if duration == 0 or abs(delta) < 1e-2:
                pose = self.goal_pose
            elif elapsed >= duration:
                pose = self.goal_pose
            else:
                pose = self._start_pose + math.copysign(min(abs(delta), self.speed * elapsed), delta)
            self.current_pose = pose
            return float(self.current_pose)

    def get_status(self):
        with self._lock:
            if abs(self.goal_pose - self.get_current_pose()) < 1e-2:
                return "reached"
            elif self._cancel:
                return "cancelled"
            else:
                return "moving"

    def cancel(self):
        with self._lock:
            if not self._cancel:
                self._cancel = True
                self._cancel_time = time.time()
                self.current_pose = float(self.get_current_pose())
                self.set_pwm(self.current_pose)
