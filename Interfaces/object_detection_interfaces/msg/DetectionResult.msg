# 目标检测结果信息

# 标准ROS头部，包含时间戳和坐标系信息
std_msgs/Header header

# 检测到的对象类别ID
int32 class_id

# 目标检测的置信度 (0-1)
float32 confidence

# 目标对象到base_link的距离 (米)
float32 distance

# 目标对象的3D位置
geometry_msgs/Point position

# 检测成功标志 (当距离小于阈值时为true)
bool success
