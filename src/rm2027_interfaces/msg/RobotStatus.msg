# 0x0201 机器人状态数据（服务器 → 对应机器人，10 Hz，内容数据段 17 字节）
# 协议表 1-10
std_msgs/Header header

uint8 robot_id                    # 本机器人 ID，见附录二（7 = 红方哨兵，107 = 蓝方哨兵）
uint8 robot_level                 # 机器人等级
uint16 current_hp                 # 当前血量
uint16 maximum_hp                 # 血量上限
uint16 shooter_barrel_cooling_value   # 射击热量冷却速率，单位 /s
uint16 shooter_barrel_heat_limit      # 射击热量上限
uint16 chassis_power_limit            # 底盘功率上限，单位 W
float32 shooter_initial_speed         # 偏移 12 机器人射击初速度上限，单位 m/s（2027 新增，float）
bool power_gimbal_output              # 云台输出：0 不输出，1 输出
bool power_chassis_output             # 底盘输出：0 不输出，1 输出
bool power_launcher_output            # 发射机构输出：0 不输出，1 输出
