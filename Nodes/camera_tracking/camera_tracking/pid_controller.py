import time

class PIDController:
    """PID控制器"""
    
    def __init__(self, kp=1.0, ki=0.0, kd=0.0, output_limits=None, windup_limit=None):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        
        self.output_limits = output_limits
        self.windup_limit = windup_limit
        
        self.reset()
    
    def reset(self):
        """重置PID控制器"""
        self.prev_error = 0.0
        self.integral = 0.0
        self.prev_time = time.time()
    
    def update(self, error, dt=None):
        """更新PID控制器"""
        current_time = time.time()
        
        if dt is None:
            dt = current_time - self.prev_time
        
        if dt <= 0:
            dt = 0.001  # 避免除零
        
        # 比例项
        proportional = self.kp * error
        
        # 积分项
        self.integral += error * dt
        
        # 积分限幅
        if self.windup_limit is not None:
            self.integral = max(min(self.integral, self.windup_limit), -self.windup_limit)
        
        integral_term = self.ki * self.integral
        
        # 微分项
        derivative = (error - self.prev_error) / dt
        derivative_term = self.kd * derivative
        
        # 总输出
        output = proportional + integral_term + derivative_term
        
        # 输出限幅
        if self.output_limits is not None:
            output = max(min(output, self.output_limits[1]), self.output_limits[0])
        
        # 更新历史值
        self.prev_error = error
        self.prev_time = current_time
        
        return output

class CascadePIDController:
    """串级PID控制器 (用于Yaw轴)"""
    
    def __init__(self, position_pid_params, velocity_pid_params):
        # 外环位置PID
        self.position_pid = PIDController(**position_pid_params)
        # 内环速度PID
        self.velocity_pid = PIDController(**velocity_pid_params)
        
        self.target_velocity = 0.0
    
    def update(self, position_error, current_velocity=0.0):
        """
        更新串级PID
        position_error: 位置误差 (像素)
        current_velocity: 当前速度 (估计值)
        """
        # 外环：位置PID输出目标速度
        self.target_velocity = self.position_pid.update(position_error)
        
        # 内环：速度PID输出控制量
        velocity_error = self.target_velocity - current_velocity
        output = self.velocity_pid.update(velocity_error)
        
        return output
    
    def reset(self):
        """重置串级PID"""
        self.position_pid.reset()
        self.velocity_pid.reset()
        self.target_velocity = 0.0
