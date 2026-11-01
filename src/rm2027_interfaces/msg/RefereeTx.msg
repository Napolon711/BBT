# 上位机要求下位机向裁判系统发送一帧（上行原始帧）
#
# 桥负责组帧：SOF + data_length + seq + CRC8 + cmd_id + data + CRC16。
# 下位机只负责把 data 原样放进内容数据段，按 bus 选择串口发出。
# 这样 0x0301（机器人交互数据）下面的子内容 0x0120 / 0x0121 等都由上位机组包，
# 协议改动不需要重新烧录下位机。
std_msgs/Header header

uint8 BUS_MAIN = 0
uint8 BUS_VIDEO = 1

uint16 cmd_id          # 命令码，例如 0x0301
uint8 bus              # 从哪条链路发出，默认 BUS_MAIN
uint8[] data           # 内容数据段（不含帧头/帧尾/CRC）
