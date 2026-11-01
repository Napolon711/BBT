# 0x0104 裁判警告数据（服务器 → 对应机器人，1/2/3 Hz，内容数据段 3 字节）
# 协议表 1-13
std_msgs/Header header

uint8 level                 # 警告等级：1 黄牌，2 红牌，3 判负
uint8 offending_robot_id    # 违规机器人 ID，见附录二
uint8 count                 # 触犯次数（用于区分同一机器人同类警告的第几次）
