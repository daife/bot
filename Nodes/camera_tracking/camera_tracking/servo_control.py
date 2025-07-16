import wiringpi
import time
import threading
from wiringpi import GPIO

class YawServoController:
    """Yaw轴舵机控制器 - 连续旋转舵机 - 软件PWM"""
    
    def __init__(self, pin=6):
        self.pin = pin
        self.current_speed = 0.0  # -1.0 到 1.0
        self.running = False
        self.pwm_thread = None
        
        # 初始化GPIO
        wiringpi.wiringPiSetup()
        wiringpi.pinMode(self.pin, GPIO.OUTPUT)
        wiringpi.digitalWrite(self.pin, GPIO.LOW)
        
        print(f"[YawServo] 初始化完成，使用软件PWM引脚: {self.pin}")
    
    def start_pwm(self):
        """启动PWM线程"""
        if self.running:
            return
        
        self.running = True
        self.pwm_thread = threading.Thread(target=self._pwm_worker, daemon=True)
        self.pwm_thread.start()
        print("[YawServo] 软件PWM线程已启动")
    
    def stop_pwm(self):
        """停止PWM线程"""
        self.running = False
        if self.pwm_thread and self.pwm_thread.is_alive():
            self.pwm_thread.join(timeout=1.0)
        wiringpi.digitalWrite(self.pin, GPIO.LOW)
        print("[YawServo] 软件PWM线程已停止")
    
    def set_speed(self, speed):
        """
        设置舵机速度
        speed: -1.0 (最快逆时针) 到 1.0 (最快顺时针)
        """
        # 限制速度范围
        self.current_speed = max(-1.0, min(1.0, speed))
    
    def _pwm_worker(self):
        """PWM工作线程"""
        while self.running:
            try:
                # 将速度转换为脉宽
                # speed=-1.0 -> 0.5ms, speed=0.0 -> 1.5ms, speed=1.0 -> 2.5ms
                pulse_width_ms = 1.5 + self.current_speed * 1.0  # 修正为±1.0ms
                pulse_width_sec = pulse_width_ms / 1000.0

                period = 0.02  # 20ms周期

                # 生成PWM信号
                wiringpi.digitalWrite(self.pin, GPIO.HIGH)
                time.sleep(pulse_width_sec)
                wiringpi.digitalWrite(self.pin, GPIO.LOW)
                time.sleep(period - pulse_width_sec)

            except Exception as e:
                print(f"[YawServo] PWM线程错误: {e}")
                time.sleep(0.02)

class PitchServoController:
    """Pitch轴舵机控制器 - 位置舵机 - 硬件PWM"""
    
    def __init__(self, pin=19, min_angle=0, max_angle=180):
        self.pin = pin
        self.min_angle = min_angle
        self.max_angle = max_angle
        self.current_angle = 90  # 初始角度为中位
        
        # 初始化GPIO和硬件PWM (参考arm.py的设置)
        wiringpi.wiringPiSetup()
        wiringpi.pinMode(self.pin, GPIO.PWM_OUTPUT)
        
        # 设置PWM范围 (参考arm.py，但调整为适合180度舵机)
        wiringpi.pwmSetRange(self.pin, 3000000)
        
        # 设置初始角度
        self.set_angle(90)
        
        print(f"[PitchServo] 初始化完成，使用硬件PWM引脚: {self.pin}, 角度范围: {min_angle}-{max_angle}")
    
    def start_pwm(self):
        """启动PWM（硬件PWM不需要额外启动）"""
        print("[PitchServo] 硬件PWM已启用")
    
    def stop_pwm(self):
        """停止PWM"""
        wiringpi.pwmWrite(self.pin, 0)
        print("[PitchServo] 硬件PWM已停止")
    
    def set_angle(self, angle):
        """
        设置舵机角度
        angle: 0-180度
        """
        self.current_angle = max(self.min_angle, min(self.max_angle, angle))
        
        # 将角度转换为PWM值 (适用于180度舵机)
        # 0度对应0.5ms脉宽，180度对应2.5ms脉宽
        # 使用类似arm.py的PWM计算方式，但调整为180度范围
        pulse_width_ms = 0.5 + (self.current_angle / 180.0) * 2.0  # 0.5ms到2.5ms
        duty_cycle = pulse_width_ms / 20.0  # 20ms周期
        pwm_value = int(duty_cycle * 3000000)  # 使用3000000作为范围
        
        wiringpi.pwmWrite(self.pin, pwm_value)
    
    def get_angle(self):
        """获取当前角度"""
        return self.current_angle