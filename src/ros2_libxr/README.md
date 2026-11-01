# ros2_libxr

基于 `27BT-UC` 当前串口桥，使用 `ref_frame/ref_tx/chassis_state/fold_cmd` 契约。裁判协议在 NUC 解析，消息定义来自同工作区 `rm2027_interfaces`。

BBT 增补了 `0x0303` 操作手点击解析，输出 `/referee/operator_cmd` Float64 `[type,param,seq,x,y]`，重复内容去重。其余线结构、14 元素摘要、速度轴、ROTOR 合成及 200 ms 看门狗与参考方案一致。

```bash
source install/setup.bash
ros2 launch ros2_libxr bridge.launch.py port:=/dev/ttyACM0
```

参数见 `config/bridge.yaml`；完整接口见仓库 `docs/电控接口文档.md`。`/sentry/buy_*` 为意图观测，实际购买只消费 `/referee/sentry_command`。`libxr/` 为保留的历史依赖，当前桥不参与编译该目录。
