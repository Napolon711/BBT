# 0x0206 机器人受伤害数据（服务器 → 对应机器人，即时发送，内容数据段 7 字节）
# 协议表 1-16。本机裁判系统本地判定，最终是否生效以服务器判定为准。
std_msgs/Header header

uint8 armor_id             # 装甲模块（或测速模块）ID 编号，其他原因扣血时为 0
uint8 hp_deduction_reason  # 0 弹丸攻击，1 装甲模块离线，2 超级电容管理模块离线，5 受撞击
uint16 hit_sequence        # 命中事件序号，每发命中递增 1
int16 hit_position_x       # 命中位置 x，单位 mm（装甲检测面左下角为原点）
int16 hit_position_y       # 命中位置 y，单位 mm
