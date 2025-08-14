import wiringpi
import time

class Claw:
    STATUS_UNGRASPED = 0      # 未抓取
    STATUS_GRASPING = 1       # 抓取中
    STATUS_GRASPED = 2        # 抓取成功
    STATUS_FAILED = 3         # 抓取失败
    STATUS_INIT = 4           # 初始化中

    CMD_GRASP = 0
    CMD_RELEASE = 1
    CMD_QUERY = 2

    def __init__(self, device=None):
        self.status = self.STATUS_INIT
        # 初始化wiringpi并设置wpi=7为输出
        wiringpi.wiringPiSetup()
        self.gpio_pin = 7
        wiringpi.pinMode(self.gpio_pin, 1)  # 1为输出模式
        wiringpi.digitalWrite(self.gpio_pin, 0)  # 默认低电平
        self.status = self.STATUS_UNGRASPED

    def grasp(self):
        if self.status != self.STATUS_UNGRASPED:
            return
        self.status = self.STATUS_GRASPING
        try:
            wiringpi.digitalWrite(self.gpio_pin, 1)  # 设置高电平，抓取
            time.sleep(2)  # 等待动作完成
            self.status = self.STATUS_GRASPED
        except Exception as e:
            print(f"[Claw] Grasp failed: {e}")
            self.status = self.STATUS_FAILED

    def release(self):
        if self.status not in [self.STATUS_GRASPED, self.STATUS_FAILED]:
            return
        try:
            wiringpi.digitalWrite(self.gpio_pin, 0)  # 设置低电平，释放/初始化
            time.sleep(2)
            self.status = self.STATUS_UNGRASPED
        except Exception as e:
            print(f"[Claw] Release failed: {e}")
            self.status = self.STATUS_FAILED

    def get_status(self):
        return self.status

    def print_angles(self):
        print("[Claw] 当前硬件版本不支持角度读取。")

    def close(self):
        wiringpi.digitalWrite(self.gpio_pin, 0)  # 关闭时确保释放