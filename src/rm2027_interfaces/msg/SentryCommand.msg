# 子内容 0x0120 哨兵自主决策指令（机器人 → 裁判系统服务器）
#
# 它不是一个独立的 cmd_id，而是 0x0301"机器人交互数据"里 data_cmd_id = 0x0120 的子内容，
# 内容数据段 4 字节。桥会自动补上 0x0301 的 6 字节子头
# （data_cmd_id + sender_id + receiver_id=0x8080），再交给下位机发帧。
#
# ★ 所有"次数"类和"发弹量"类字段都要求单调递增，桥会自己维护单调计数器，
#   本消息里的字段是**绝对值**（连续发同样的值 = 不重复生效），不是增量。
std_msgs/Header header

# bit0 是否确认复活：0 确认不复活（即使读条完成），1 确认复活
bool confirm_respawn
# bit1 是否确认兑换立即复活
bool confirm_buy_respawn
# bit2-12 将要兑换的允许发弹量累计值（开局 0，只能单调递增）
uint16 exchange_allowance_total
# bit13-16 远程兑换发弹量的请求次数（开局 0，每次只能 +1）
uint8 remote_allowance_times
# bit17-20 远程兑换血量的请求次数（开局 0，每次只能 +1）
uint8 remote_hp_times
# bit21-23 姿态：1 进攻，2 防御，3 移动，4 强化进攻，5 强化防御，6 强化移动
uint8 posture
# bit24 是否确认让能量机关进入正在激活状态
bool activate_energy_mechanism
