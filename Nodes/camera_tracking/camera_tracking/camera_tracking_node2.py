import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose, Twist
import threading
import time
import serial
import struct

K_TRANSFER = 0.1  # 转嫁到底盘的比例系数
# 串口初始化（请根据实际设备号修改）
uart1 = serial.Serial('/dev/ttyAMA2', baudrate=115200, bytesize=8, parity='N', stopbits=1, timeout=0.1)

def Emm_V5_En_Control(addr, state, snF):
    cmd = bytearray(16)
    cmd[0] = addr
    cmd[1] = 0xF3
    cmd[2] = 0xAB
    cmd[3] = 0x01 if state else 0x00
    cmd[4] = 0x01 if snF else 0x00
    cmd[5] = 0x6B
    uart1.write(cmd[:6])

def Emm_V5_Vel_Control(addr, dir, vel, acc, snF):
    cmd = bytearray(16)
    cmd[0] = addr
    cmd[1] = 0xF6
    cmd[2] = dir
    cmd[3] = (vel >> 8) & 0xFF
    cmd[4] = vel & 0xFF
    cmd[5] = acc
    cmd[6] = 0x01 if snF else 0x00
    cmd[7] = 0x6B
    uart1.write(cmd[:8])
    
def Emm_V5_Pos_Control(addr, dir, vel, acc, clk, raF, snF): # 地址电机，设置方向为CW，速度为1000RPM，加速度为50，脉冲数为2000，相对运动，无多机同步
    cmd = bytearray(16)
    cmd[0] = addr                      # 地址
    cmd[1] = 0xFD                      # 功能码
    cmd[2] = dir                       # 方向
    cmd[3] = (vel >> 8) & 0xFF         # 速度(RPM)高8位字节
    cmd[4] = vel & 0xFF                # 速度(RPM)低8位字节 
    cmd[5] = acc                       # 加速度，注意：0是直接启动
    cmd[6] = (clk >> 24) & 0xFF        # 脉冲数高8位字节(bit24 - bit31)
    cmd[7] = (clk >> 16) & 0xFF        # 脉冲数(bit16 - bit23)
    cmd[8] = (clk >> 8) & 0xFF         # 脉冲数(bit8  - bit15)
    cmd[9] = clk & 0xFF                # 脉冲数低8位字节(bit0  - bit7)
    cmd[10] = 0x01 if raF else 0x00    # 相位/绝对标志，true为0x01绝对，false为0x00相对
    cmd[11] = 0x01 if snF else 0x00    # 多机同步运动标志，true为0x01，false为0x00
    cmd[12] = 0x6B                     # 校验字节
    uart1.write(cmd[:13])

def Emm_V5_Read_Sys_Params(addr, s):
    i = 0
    cmd = bytearray(16)
    cmd[i] = addr
    i += 1
    func_codes = {
        'S_CPOS': 0x36,
    }
    if s in func_codes:
        cmd[i] = func_codes[s]
        i += 1
    cmd[i] = 0x6B
    i += 1
    uart1.write(cmd[:i])

def Emm_V5_Receive_Data(uart, max_wait_ms=30):
    rxCmd = bytearray(128)
    i = 0
    lTime = cTime = int(time.time() * 1000)
    start_time = lTime
    while True:
        if uart.in_waiting:
            if i < 128:
                rxCmd[i] = uart.read(1)[0]
                i += 1
                lTime = int(time.time() * 1000)
        else:
            cTime = int(time.time() * 1000)
            # 增加最大等待时间限制
            if cTime - lTime > 100 or cTime - start_time > max_wait_ms:
                hex_data = ' '.join(['{:02x}'.format(b) for b in rxCmd[:i]])
                hex_data = hex_data.strip('00 ')
                if hex_data and hex_data[0] != '0':
                    hex_data = '0' + hex_data
                return hex_data, len(hex_data.replace(' ', '')) // 2

def get_motor_position(addr):
    Emm_V5_Read_Sys_Params(addr, 'S_CPOS')
    time.sleep(0.01)
    # 设置最大等待时间为30ms
    data, count = Emm_V5_Receive_Data(uart1, max_wait_ms=30)
    if not data or count < 7:
        return None
    data_hex = data.split()
    try:
        if int(data_hex[0], 16) == addr and int(data_hex[1], 16) == 0x36:
            pos = struct.unpack('>I', bytes.fromhex(''.join(data_hex[3:7])))[0]
            angle = float(pos) * 360.0 / 65536.0
            if int(data_hex[2], 16):
                angle = -angle
            return angle
    except Exception:
        pass
    return None

class PIDController:
    def __init__(self, kp=0.1, ki=0.001, kd=0.0):
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
        output = (self.kp * error + self.ki * self.integral + self.kd * derivative)
        self.prev_error = error
        self.prev_time = now
        return output

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



class CameraTrackingNode2(Node):
    def __init__(self):
        super().__init__('camera_tracking_node2')
        self.pid = PIDController()
        self.pitch_serial = PitchSerialController()
        self.subscription = self.create_subscription(
            Pose, 'paper_center_pose_smooth', self.pose_callback, 10)
        self.twist_pub = self.create_publisher(Twist, '/camera_tracking_twist', 10)
        self.filtered_error = 0.0
        self.last_pitch_error = 0.0
        self.deadzone = 3.0
        self.addr = 1
        self.dir = 0
        self.acc = 0
        self.snF = False
        self.vel_max = 2000
        self.vel_min = 0
        self.current_vel = 0
        self.current_angle = -60.0  # 软件累加角度，初始为0度
        self._stop_flag = False
        self.last_update_time = time.time()
        self.motor_vel_last = 0  # 上一次实际发送给电机的速度
        # 启动硬件角度读取线程
        self.position_thread = threading.Thread(target=self._update_position_loop, daemon=True)
        self.position_thread.start()

        Emm_V5_En_Control(self.addr, True, False)
        time.sleep(0.1)
        Emm_V5_Pos_Control(
            addr=1,         # 地址1
            dir=1,           # 方向：0（CW）或1（CCW），根据实际需求调整
            vel=1000,        # 速度：1000 RPM
            acc=50,          # 加速度档位（50，根据实际调整）
            clk=533,      # 脉冲数
            raF=True,       # 相对位置模式（False表示相对当前位置移动）
            snF=False        # 不使用多机同步 
        )
        time.sleep(2)
        Emm_V5_Vel_Control(
            addr=self.addr,
            dir=self.dir,
            vel=self.current_vel,
            acc=self.acc,
            snF=self.snF
        )

    def pose_callback(self, msg):
        if msg.position.z == -1.0:
            self.filtered_error = 0.0
            self.last_pitch_error = 0.0
            self.current_vel = 0
        else:
            self.filtered_error = msg.position.x
            self.last_pitch_error = msg.position.y

        raw_error = self.filtered_error
        error = raw_error if abs(raw_error) > self.deadzone else 0.0

        vel = int(self.pid.update(error))
        if vel >= 0:
            self.dir = 0  # 顺时针
        else:
            self.dir = 1  # 逆时针
        abs_vel = abs(vel)
        abs_vel = max(self.vel_min, min(self.vel_max, abs_vel))

        now = time.time()
        dt = now - self.last_update_time
        self.last_update_time = now

        # 软件累加角度
        delta_angle = self.motor_vel_last * 360.0 / 60.0 * dt
        if self.dir == 0:
            self.current_angle += delta_angle
        else:
            self.current_angle -= delta_angle

        twist_msg = Twist()
        transfer_ratio = 0.0

        # 默认 motor_vel = abs_vel
        motor_vel = abs_vel

        # 负方向完全转嫁
        if self.current_angle < -157.5 and self.dir == 1 and abs_vel > 0:
            transfer_ratio = 1.0
            motor_vel = 0
            chassis_vel = abs_vel * K_TRANSFER
            Emm_V5_Vel_Control(
                addr=self.addr,
                dir=self.dir,
                vel=0,
                acc=self.acc,
                snF=self.snF
            )
            self.current_vel = 0
            twist_msg.angular.z = chassis_vel
        # -127.5 ~ -157.5 逆时针速度转嫁
        elif -157.5 < self.current_angle <= -127.5 and self.dir == 1 and abs_vel > 0:
            transfer_ratio = (self.current_angle + 157.5) / 30.0
            motor_vel = int(abs_vel * (1.0 - transfer_ratio))
            chassis_vel = abs_vel * transfer_ratio * K_TRANSFER
            Emm_V5_Vel_Control(
                addr=self.addr,
                dir=self.dir,
                vel=motor_vel,
                acc=self.acc,
                snF=self.snF
            )
            self.current_vel = motor_vel
            twist_msg.angular.z = chassis_vel
        # 正方向完全转嫁
        elif self.current_angle > 157.5 and self.dir == 0 and abs_vel > 0:
            transfer_ratio = 1.0
            motor_vel = 0
            chassis_vel = abs_vel * K_TRANSFER
            Emm_V5_Vel_Control(
                addr=self.addr,
                dir=self.dir,
                vel=0,
                acc=self.acc,
                snF=self.snF
            )
            self.current_vel = 0
            twist_msg.angular.z = -chassis_vel
        # 127.5 ~ 157.5 顺时针速度转嫁
        elif 127.5 <= self.current_angle <= 157.5 and self.dir == 0 and abs_vel > 0:
            transfer_ratio = (self.current_angle - 127.5) / 30.0
            motor_vel = int(abs_vel * (1.0 - transfer_ratio))
            chassis_vel = abs_vel * transfer_ratio * K_TRANSFER
            Emm_V5_Vel_Control(
                addr=self.addr,
                dir=self.dir,
                vel=motor_vel,
                acc=self.acc,
                snF=self.snF
            )
            self.current_vel = motor_vel
            twist_msg.angular.z = -chassis_vel
        else:
            motor_vel = abs_vel
            Emm_V5_Vel_Control(
                addr=self.addr,
                dir=self.dir,
                vel=abs_vel,
                acc=self.acc,
                snF=self.snF
            )
            self.current_vel = abs_vel
            twist_msg.angular.z = 0.0

        # 更新 motor_vel_last 为本次实际发送给电机的速度
        self.motor_vel_last = motor_vel

        # print(f"[Angle] 当前软件累加角度: {self.current_angle:.2f}°")

        self.twist_pub.publish(twist_msg)
        self.pitch_serial.send_y(self.last_pitch_error)

    def _update_position_loop(self):
        while not self._stop_flag:
            hw_angle = get_motor_position(self.addr)
            if hw_angle is not None:
                self.current_angle = hw_angle
                # print(f"[Angle] 硬件矫正角度: {self.current_angle:.2f}°")
            time.sleep(0.05)  # 20Hz硬件矫正

    def destroy_node(self):
        self._stop_flag = True
        # 停止电机
        Emm_V5_Vel_Control(
            addr=self.addr,
            dir=self.dir,
            vel=0,
            acc=self.acc,
            snF=self.snF
        )
        self.pitch_serial.close()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = CameraTrackingNode2()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("\n退出 camera_tracking_node2。")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()