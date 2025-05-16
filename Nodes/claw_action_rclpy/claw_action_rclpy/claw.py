import wiringpi
import time
import sys
import threading
import collections

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
        self.serial = -1
        self.buffer = collections.deque(maxlen=4096)  # 循环缓冲区，存储最多4096个字符
        self.buffer_lock = threading.Lock()  # 用于线程安全访问缓冲区
        self.running = False  # 控制后台线程运行
        
        # 尝试打开可用的串口
        if device is not None:
            # 如果指定了设备，只尝试该设备
            self.device = device
            self.serial = wiringpi.serialOpen(self.device, 115200)
            if self.serial < 0:
                print(f"[Claw] 无法打开指定串口设备: {self.device}")
                sys.exit(-1)
        else:
            # 尝试ttyUSB0到ttyUSB3
            for i in range(4):
                try_device = f"/dev/ttyUSB{i}"
                print(f"[Claw] 尝试打开串口: {try_device}")
                try_serial = wiringpi.serialOpen(try_device, 115200)
                if try_serial >= 0:
                    self.device = try_device
                    self.serial = try_serial
                    print(f"[Claw] 成功打开串口设备: {self.device}")
                    break
            
            if self.serial < 0:
                print("[Claw] 错误: 无法打开任何串口设备(/dev/ttyUSB0到/dev/ttyUSB3)")
                sys.exit(-1)
        
        # 启动后台线程，持续读取串口数据
        self.running = True
        self.reader_thread = threading.Thread(target=self._serial_reader, daemon=True)
        self.reader_thread.start()
        
        try:
            self._send_current('A', -0.04)
            self._send_current('B', -0.25)
            time.sleep(2)
            self.zero_a, self.zero_b = self._read_angles()
            self._send_current('B', -0.1)
            self.status = self.STATUS_UNGRASPED
        except Exception as e:
            print(f"[Claw] 初始化失败: {e}")
            self.status = self.STATUS_FAILED

    def _serial_reader(self):
        """后台线程，持续从串口读取数据并存入缓冲区"""
        while self.running and self.serial >= 0:
            try:
                if wiringpi.serialDataAvail(self.serial) > 0:
                    ch = chr(wiringpi.serialGetchar(self.serial))
                    with self.buffer_lock:
                        self.buffer.append(ch)
                else:
                    time.sleep(0.001)  # 短暂休眠，避免CPU占用过高
            except Exception as e:
                print(f"[Claw] 串口读取错误: {e}")
                time.sleep(0.1)  # 错误后短暂休眠

    def _get_latest_line(self):
        """从缓冲区获取最新的完整行（最后两个换行符之间的内容）"""
        with self.buffer_lock:
            buffer_str = ''.join(self.buffer)
            # 找到最后两个换行符的位置
            last_newline = buffer_str.rfind('\n')
            if last_newline == -1:  # 没有换行符
                return None
                
            second_last_newline = buffer_str.rfind('\n', 0, last_newline)
            if second_last_newline == -1:  # 只有一个换行符
                line = buffer_str[:last_newline]
            else:
                line = buffer_str[second_last_newline + 1:last_newline]
                
            return line.strip() if line else None

    def _send_current(self, motor, current):
        if self.serial < 0:
            raise RuntimeError("串口未打开，无法发送电流指令")
        cmd = f"{motor}{current}\n"
        print(f"[Claw] 发送到串口: {cmd.strip()}")
        wiringpi.serialPrintf(self.serial, cmd)
        #wiringpi.serialFlush(self.serial)
        time.sleep(0.1)  # 等待指令执行完成

    def _read_angles(self):
        if self.serial < 0:
            raise RuntimeError("串口未打开，无法读取角度")
        
        # 尝试从缓冲区读取最新的一行数据
        max_retries = 10
        retry_count = 0
        
        while retry_count < max_retries:
            try:
                # 等待一小段时间，确保有新数据
                time.sleep(0.1)
                
                # 获取最新的一行数据
                last_line = self._get_latest_line()
                
                if not last_line:
                    print(f"[Claw] 尝试 {retry_count+1}/{max_retries}: 缓冲区中没有完整数据，重试...")
                    retry_count += 1
                    continue
                
                print(f"[Claw] 从缓冲区获取最新数据: {last_line}")
                
                parts = last_line.split(',')
                a = float(parts[0][1:]) if parts[0].startswith('A') else None
                b = float(parts[1][1:]) if len(parts) > 1 and parts[1].startswith('B') else None
                
                if a is None or b is None:
                    print(f"[Claw] 尝试 {retry_count+1}/{max_retries}: 无效的角度数据，重试...")
                    retry_count += 1
                    continue
                
                # 计算相对于零点的角度
                if hasattr(self, 'zero_a') and hasattr(self, 'zero_b'):
                    rel_a = (a - self.zero_a)
                    rel_b = (b - self.zero_b)
                    print(f"[Claw] 当前绝对角度: A={a:.2f}, B={b:.2f}")
                    print(f"[Claw] 相对零点角度: A={rel_a:.2f}, B={rel_b:.2f}")
                else:
                    # 初始化过程中还没有零点值
                    print(f"[Claw] 当前绝对角度: A={a:.2f}, B={b:.2f}")
                
                return a, b
                
            except Exception as e:
                print(f"[Claw] 尝试 {retry_count+1}/{max_retries}: 解析角度数据失败: {e}")
                retry_count += 1
        
        # 所有重试都失败
        raise RuntimeError(f"获取角度失败: {max_retries}次尝试后仍无法获取有效数据")

    def grasp(self):
        if self.status != self.STATUS_UNGRASPED:
            return
        self.status = self.STATUS_GRASPING
        try:
            self._send_current('A', -0.5)            
            self._send_current('B', -0.03)

            time.sleep(2)
            a_angle, _ = self._read_angles()
            rel_a = (a_angle - self.zero_a)
            if rel_a > 355:
                self.status = self.STATUS_FAILED
            else:
                self.status = self.STATUS_GRASPED
        except Exception as e:
            print(f"[Claw] Grasp failed: {e}")
            self.status = self.STATUS_FAILED

    def release(self):
        if self.status not in [self.STATUS_GRASPED, self.STATUS_FAILED]:
            return
        try:
            self._send_current('A', -0.04)
            self._send_current('B', -0.25)
            time.sleep(2)
            self._send_current('B', -0.1)
            self.status = self.STATUS_UNGRASPED
        except Exception as e:
            print(f"[Claw] Release failed: {e}")
            self.status = self.STATUS_FAILED

    def get_status(self):
        return self.status

    def print_angles(self):
        self._read_angles()

    def close(self):
        self.running = False  # 停止读取线程
        if self.reader_thread and self.reader_thread.is_alive():
            self.reader_thread.join(timeout=1.0)  # 等待线程结束，最多1秒
        if self.serial >= 0:
            wiringpi.serialClose(self.serial)
