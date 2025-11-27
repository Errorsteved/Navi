int32 id             # 目标跟踪ID
string class_name    # 目标类别名称，比如 'car', 'person'
int32 bbox_x         # 检测框左上角x坐标
int32 bbox_y         # 检测框左上角y坐标
int32 bbox_width     # 检测框宽度
int32 bbox_height    # 检测框高度
float32 distance_m   # 估算的目标距离 (单位: 米)
float32 speed_kmh    # 估算的目标速度 (单位: km/h)