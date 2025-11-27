# 时间与位置信息
string utc_time           # UTC时间（协调世界时）
string latitude           # 纬度（例如："31.23N"）
string longitude          # 经度（例如："121.47E"）
string num_satellites     # 卫星数量
string altitude           # 海拔高度（例如："500.0m"）

# 航向与速度
string heading_true       # 相对于真北的航向角（例如："120.5°"）
string heading_magnetic   # 相对于磁北的航向角（例如："118.0°"）
string speed_knots        # 地面速度（单位：节，例："12.3Kn"）
string speed_kmph         # 地面速度（单位：千米/小时，例："22.8Km/h"）

# IMU 数据（惯性测量单元）
float32 accel_x           # X轴加速度（单位：m/s²）
float32 accel_y           # Y轴加速度（单位：m/s²）
float32 accel_z           # Z轴加速度（单位：m/s²）

float32 gyro_x            # X轴角速度（单位：rad/s）
float32 gyro_y            # Y轴角速度（单位：rad/s）
float32 gyro_z            # Z轴角速度（单位：rad/s）

float32 roll              # 横滚角（单位：度）
float32 pitch             # 俯仰角（单位：度）
float32 yaw               # 偏航角（单位：度）
