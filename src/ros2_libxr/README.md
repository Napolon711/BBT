# ros2_libxr

基于 `27BT-UC` 当前串口桥，使用 `ref_frame/ref_tx/chassis_state/fold_cmd` 契约。裁判协议在 NUC 解析，消息定义来自同工作区 `rm2027_interfaces`。

BBT 增补了 `0x0303` 操作手点击解析，输出 `/referee/operator_cmd` Float64 `[type,param,seq,x,y]`，重复内容去重。其余线结构、14 元素摘要、速度轴、ROTOR 合成及 200 ms 看门狗与参考方案一致。

```bash
source install/setup.bash
ros2 launch ros2_libxr bridge.launch.py own_team:=red port:=/dev/ttyACM0
```

每次开局必须人工指定 `own_team:=red` 或 `own_team:=blue`，且与 BBT 决策节点一致。桥将发送方固定为红方 `7` / 蓝方 `107`；裁判上报的 `robot_id` 只作为原始遥测发布，不切换己方、不否决购买。`own_team` 在节点运行期间只读，换边时重启节点。游戏状态、弹量、血量和购买计数仍按各自的新鲜度校验。

伪下位机可用 `python3 src/ros2_libxr/scripts/fake_lower_machine.py --own-team red --spawn-bridge` 验证；蓝方将 `red` 换成 `blue`。

参数见 `config/bridge.yaml`；完整接口见仓库 `docs/电控接口文档.md`。`/sentry/buy_*` 为意图观测，实际购买只消费 `/referee/sentry_command`。`libxr/` 为保留的历史依赖，当前桥不参与编译该目录。
