# 0x0204 机器人增益数据（服务器 → 对应机器人，3 Hz，内容数据段 7 字节）
# 协议表 1-15
std_msgs/Header header

uint8 recovery_buff       # 回血增益，百分比（10 表示每秒恢复血量上限的 10%）
uint16 cooling_buff       # 射击热量冷却增益具体值（直接值，x 表示冷却增加 x/s）
uint8 defence_buff        # 防御增益，百分比
uint8 vulnerability_buff  # 负防御增益，百分比（30 表示 -30%）
uint16 attack_buff        # 攻击增益，百分比（50 表示 50%）
