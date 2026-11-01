# 0x0208 弹丸允许发弹量（服务器 → 己方重装/步兵/哨兵，10 Hz，内容数据段 8 字节）
# 协议表 1-18
std_msgs/Header header

uint16 projectile_allowance_17mm      # 机器人自身剩余的 17mm 允许发弹量
uint16 projectile_allowance_42mm      # 42mm 允许发弹量
uint16 remaining_gold_coin            # 剩余金币数量
uint16 projectile_allowance_fortress  # 堡垒储备允许发弹量
