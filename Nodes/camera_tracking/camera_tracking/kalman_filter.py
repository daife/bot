import numpy as np

class KalmanFilter2D:
    """
    二维卡尔曼滤波器，用于平滑像素坐标偏差
    状态向量: [x, y, vx, vy] (位置和速度)
    """
    
    def __init__(self, process_variance=1.0, measurement_variance=10.0):
        # 状态向量 [x, y, vx, vy]
        self.state = np.zeros(4)
        
        # 状态协方差矩阵
        self.P = np.eye(4) * 100.0
        
        # 状态转移矩阵 (dt = 0.033s, 约30Hz)
        dt = 0.033
        self.F = np.array([
            [1, 0, dt, 0],
            [0, 1, 0, dt],
            [0, 0, 1, 0],
            [0, 0, 0, 1]
        ])
        
        # 过程噪声协方差矩阵
        q = process_variance
        self.Q = np.array([
            [dt**4/4*q, 0, dt**3/2*q, 0],
            [0, dt**4/4*q, 0, dt**3/2*q],
            [dt**3/2*q, 0, dt**2*q, 0],
            [0, dt**3/2*q, 0, dt**2*q]
        ])
        
        # 观测矩阵 (只观测位置)
        self.H = np.array([
            [1, 0, 0, 0],
            [0, 1, 0, 0]
        ])
        
        # 观测噪声协方差矩阵
        r = measurement_variance
        self.R = np.array([
            [r, 0],
            [0, r]
        ])
        
        self.initialized = False
    
    def predict(self):
        """预测步骤"""
        # 状态预测
        self.state = np.dot(self.F, self.state)
        
        # 协方差预测
        self.P = np.dot(np.dot(self.F, self.P), self.F.T) + self.Q
    
    def update(self, measurement):
        """更新步骤"""
        if not self.initialized:
            # 初始化状态
            self.state[0] = measurement[0]  # x
            self.state[1] = measurement[1]  # y
            self.state[2] = 0.0             # vx
            self.state[3] = 0.0             # vy
            self.initialized = True
            return
        
        # 计算新息
        z = np.array(measurement)
        y = z - np.dot(self.H, self.state)
        
        # 新息协方差
        S = np.dot(np.dot(self.H, self.P), self.H.T) + self.R
        
        # 卡尔曼增益
        K = np.dot(np.dot(self.P, self.H.T), np.linalg.inv(S))
        
        # 状态更新
        self.state = self.state + np.dot(K, y)
        
        # 协方差更新
        I = np.eye(4)
        self.P = np.dot((I - np.dot(K, self.H)), self.P)
    
    def get_position(self):
        """获取滤波后的位置"""
        return self.state[0], self.state[1]
    
    def get_velocity(self):
        """获取估计的速度"""
        return self.state[2], self.state[3]
    
    def reset(self):
        """重置滤波器"""
        self.state = np.zeros(4)
        self.P = np.eye(4) * 100.0
        self.initialized = False
