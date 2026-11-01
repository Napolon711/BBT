# 0x020B 地面机器人位置数据（服务器 → 对应机器人，1 Hz，内容数据段 40 字节）
# 协议表 1-21。★ 只包含"己方"3 台地面机器人，单位 m。
# 2027 版把 2026 的 1/2 号步兵改成了"重装 + 3 号 + 4 号"，并保留 4 个 4 字节保留位。
# 保留位在下位机转发时被丢弃，不进入本消息。
std_msgs/Header header

geometry_msgs/Point heavy_engineer   # 己方重装机器人位置（x, y），单位 m
geometry_msgs/Point standard_3       # 己方 3 号步兵机器人位置，单位 m
geometry_msgs/Point standard_4       # 己方 4 号步兵机器人位置，单位 m
bool valid                           # 三台位置是否全部有效（任一保留位异常时由下位机置 false）
