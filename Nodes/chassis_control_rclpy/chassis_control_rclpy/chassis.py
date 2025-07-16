import serial
import time
import sys
import struct

class ChassisController:
    """通过串口与自定义驱动板通信的麦轮底盘控制器。"""
    
    def __init__(self, device="/dev/ttyAMA2"):
        """
        初始化底盘控制器。
        
        参数:
            device: 串口设备路径，默认为ttyAMA2。
        """
        self.serial_port = None
        self.device = device
        
        # 打开固定串口设备 ttyAMA2
        print(f"[底盘] 尝试打开串口: {self.device}")
        try:
            self.serial_port = serial.Serial(
                port=self.device,
                baudrate=115200,
                bytesize=serial.EIGHTBITS,
                parity=serial.PARITY_NONE,
                stopbits=serial.STOPBITS_ONE,
                timeout=1
            )
            print("[底盘] 麦轮底盘控制器初始化完成")
        except serial.SerialException as e:
            print(f"[底盘] 无法打开串口设备: {self.device}, 错误: {e}")
            sys.exit(-1)

    def _send_command(self, linear_x, linear_y, angular_z):
        """
        发送二进制命令到底盘控制器。
        
        参数:
            linear_x: X方向线速度（前进/后退）
            linear_y: Y方向线速度（左右平移）
            angular_z: Z轴角速度（旋转）
        """
        if self.serial_port is None or not self.serial_port.is_open:
            raise RuntimeError("串口未打开，无法发送命令")
        
        # 生成二进制命令: 0xAA + 三个浮点数(各4字节) + '\n'
        header = bytes([0xAA])
        data = struct.pack('<fff', linear_x, linear_y, angular_z)  # 小端序浮点数
        newline = b'\n'
        
        # 组合完整命令
        command = header + data + newline
        
        print(f"[底盘] 发送二进制命令: 0xAA + [{linear_x:.2f},{linear_y:.2f},{angular_z:.2f}] + '\\n'")
        
        # 通过串口发送二进制数据
        self.serial_port.write(command)
        self.serial_port.flush()  # 确保数据立即发送
        
        time.sleep(0.01)  # 短暂延迟确保命令被处理

    def set_velocity(self, linear_x, linear_y, angular_z):
        """
        设置麦轮底盘的速度。
        
        参数:
            linear_x: X方向线速度（前进/后退）
            linear_y: Y方向线速度（左右平移）
            angular_z: Z轴角速度（旋转）
        """
        self._send_command(linear_x, linear_y, angular_z)

    def stop(self):
        """停止底盘移动。"""
        self.set_velocity(0.0, 0.0, 0.0)

    def close(self):
        """关闭串口连接。"""
        if self.serial_port and self.serial_port.is_open:
            self.stop()  # 确保在关闭前停止
            self.serial_port.close()
            print("[底盘] 串口连接已关闭")