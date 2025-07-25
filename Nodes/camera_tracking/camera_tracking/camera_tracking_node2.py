import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose, Twist
import time
import serial
import struct
import threading
import math

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
    print(f"[DEBUG] Enter Emm_V5_Read_Sys_Params: addr={addr}, s={s}")
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

def Emm_V5_Receive_Data(uart):
    print("[DEBUG] Enter Emm_V5_Receive_Data")
    rxCmd = bytearray(128)
    i = 0
    lTime = cTime = int(time.time() * 1000)
    while True:
        if uart.in_waiting:
            if i < 128:
                rxCmd[i] = uart.read(1)[0]
                i += 1
                lTime = int(time.time() * 1000)
        else:
            cTime = int(time.time() * 1000)
            if cTime - lTime > 100:
                hex_data = ' '.join(['{:02x}'.format(b) for b in rxCmd[:i]])
                hex_data = hex_data.strip('00 ')
                if hex_data and hex_data[0] != '0':
                    hex_data = '0' + hex_data
                return hex_data, len(hex_data.replace(' ', '')) // 2

def get_motor_position(addr):
    print(f"[DEBUG] Enter get_motor_position: addr={addr}")
    Emm_V5_Read_Sys_Params(addr, 'S_CPOS')
    time.sleep(0.01)
    data, count = Emm_V5_Receive_Data(uart1)
    if not data or count < 7:
        return None
    data_hex = data.split()
    try:
        if int(data_hex[0], 16) == addr and int(data_hex[1], 16) == 0x36:
            pos = struct.unpack('>I', bytes.fromhex(''.join(data_hex[3:7])))[0]
            angle = float(pos) * 360.0 / 65536.0
            if int(data_hex[2], 16):
                angle = -angle
            # # 角度归一化到0~360
            # angle = angle % 360.0
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

class CameraTrackingNode2(Node):
    def __init__(self):
        super().__init__('camera_tracking_node2')
        self.yaw_pid = PIDController()
        self.last_yaw_error = 0.0
        self.deadzone_yaw = 2.0
        self.addr = 1
        self.dir = 0
        self.acc = 0
        self.snF = False
        self.vel_max = 2000
        self.vel_min = 0
        self.current_vel = 0
        self.current_angle = 0.0
        self._stop_flag = False
        self.twist_pub = self.create_publisher(Twist, '/camera_tracking_twist', 10)
        Emm_V5_Vel_Control(
            addr=self.addr,
            dir=self.dir,
            vel=0,
            acc=self.acc,
            snF=self.snF
        )
        # 启动实时位置线程
        self.position_thread = threading.Thread(target=self._update_position_loop, daemon=True)
        self.position_thread.start()
        self.subscription = self.create_subscription(
            Pose, 'paper_center_pose_smooth', self.pose_callback, 10)
        self.timer = self.create_timer(0.1, self.control_callback)

    def pose_callback(self, msg):
        if msg.position.z == -1.0:
            self.last_yaw_error = 0.0
        else:
            self.last_yaw_error = msg.position.x

    def control_callback(self):
        # Yaw控制
        raw_error = self.last_yaw_error
        error = raw_error if abs(raw_error) > self.deadzone_yaw else 0.0
        vel = int(self.yaw_pid.update(error))
        # Debug: print PID output and error
        print(f"[PID] error={error:.2f}, PID output vel={vel}")

        # 方向（确保顺/逆时针都能设置）
        if vel >= 0:
            self.dir = 0  # 顺时针
        else:
            self.dir = 1  # 逆时针
        abs_vel = abs(vel)
        abs_vel = max(self.vel_min, min(self.vel_max, abs_vel))
        # Debug: print direction and velocity before sending command
        print(f"[Motor] dir={self.dir} (0:CW, 1:CCW), abs_vel={abs_vel}, angle={self.current_angle:.2f}")

        # 角度限制与转嫁
        angle = self.current_angle
        twist_msg = Twist()
        transfer_ratio = 0.0
        # 0~30度，逆时针（dir==1）速度转嫁
        if 0.0 <= angle <= 30.0 and self.dir == 1 and abs_vel > 0:
            transfer_ratio = (30.0 - angle) / 30.0
            motor_vel = int(abs_vel * (1.0 - transfer_ratio))
            chassis_vel = abs_vel * transfer_ratio * K_TRANSFER
            print(f"[Transfer] CCW region: transfer_ratio={transfer_ratio:.2f}, motor_vel={motor_vel}, chassis_vel={chassis_vel:.2f}")
            Emm_V5_Vel_Control(
                addr=self.addr,
                dir=self.dir,
                vel=motor_vel,
                acc=self.acc,
                snF=self.snF
            )
            self.current_vel = motor_vel
            twist_msg.angular.z = chassis_vel
        # 285~315度，顺时针（dir==0）速度转嫁
        elif 285.0 <= angle <= 315.0 and self.dir == 0 and abs_vel > 0:
            transfer_ratio = (angle - 285.0) / 30.0
            motor_vel = int(abs_vel * (1.0 - transfer_ratio))
            chassis_vel = abs_vel * transfer_ratio * K_TRANSFER
            print(f"[Transfer] CW region: transfer_ratio={transfer_ratio:.2f}, motor_vel={motor_vel}, chassis_vel={chassis_vel:.2f}")
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
            # 其他区域，直接控制电机，底盘速度为0
            print(f"[Direct] Normal region: dir={self.dir}, vel={abs_vel}")
            Emm_V5_Vel_Control(
                addr=self.addr,
                dir=self.dir,
                vel=abs_vel,
                acc=self.acc,
                snF=self.snF
            )
            self.current_vel = abs_vel
            twist_msg.angular.z = 0.0
        self.twist_pub.publish(twist_msg)

    def _update_position_loop(self):
        while not self._stop_flag:
            angle = get_motor_position(self.addr)
            if angle is not None:
                self.current_angle = angle
            time.sleep(0.1)

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