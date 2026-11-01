# 0x0003 机器人血量数据（服务器 → 全体机器人，3 Hz，内容数据段 24 字节）
# 协议表 1-8。
# ★ 2027 版是"己方/对方"视角，不是红蓝视角：字段名里的 ally 就是本机器人所属方，
#   所以不需要先知道自己是红还是蓝就能直接用。2 号位在 2027 是保留位。
# ★ 双方血量都是真值，不需要雷达；damage_difference 与累计控制度也在这里。
std_msgs/Header header

uint16 ally_1_robot_hp      # 偏移 0  己方 1 号重装机器人血量（未上场/被罚下为 0）
uint16 reserved             # 偏移 2  保留位
uint16 ally_3_robot_hp      # 偏移 4  己方 3 号步兵机器人血量
uint16 ally_4_robot_hp      # 偏移 6  己方 4 号步兵机器人血量
int16 damage_difference     # 偏移 8  己方全队总伤害与对方全队总伤害之差
uint16 ally_7_robot_hp      # 偏移 10 己方 7 号哨兵机器人血量（就是自己）
uint16 outpost_r_hp         # 偏移 12 R 前哨血量
uint16 ally_base_hp         # 偏移 14 己方基地血量
uint16 outpost_b_hp         # 偏移 16 B 前哨血量
uint16 enemy_base_hp        # 偏移 18 对方基地血量
uint16 ally_control_degree  # 偏移 20 己方累计控制度
uint16 enemy_control_degree # 偏移 22 对方累计控制度
