# 0x0202 实时功率热量数据（服务器 → 对应机器人，10 Hz，内容数据段 14 字节）
# 协议表 1-13。
# ★ 2027 版前 8 字节是保留位（2026 版这里直接是缓冲能量），真正有用的字段从偏移 8 开始。
#   下位机若沿用 2026 的 6 字节结构体会整帧拒绝。
std_msgs/Header header

uint16 buffer_energy      # 偏移 8  缓冲能量，单位 J
uint16 shooter_17mm_heat  # 偏移 10 17mm 发射机构的射击热量
uint16 shooter_42mm_heat  # 偏移 12 42mm 发射机构的射击热量
