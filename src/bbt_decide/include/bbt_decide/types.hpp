// BBT 哨兵保底决策树：数据模型与配置。
// 本文件不依赖 ROS，纯逻辑库与单元测试直接包含。
//
// 与主工作区（27BT-UC/sentry_decide）的关系：
//   本包是从主包"保底"分支裁剪出来的独立版 —— 只保留哨兵导航、电控（底盘/折叠）、
//   裁判系统三类接口；RL / 雷达 / 自瞄不要（自瞄完全交给电控）。
//   购买：**决策层自己自动买**（血/弹低了、金币够就下单，见 Config 的 auto_buy 段）；
//   操作手的买指令不是决策层的事（裁判客户端/电控直接办），收到也忽略——买没买到
//   从裁判数据里看（血量/弹量涨了就是到账了）。决策层唯一接的操作手输入是"指个
//   临时点"。
//
// 设计硬规则（对应使用者的要求）：
//   1) 所有点（家 / 巡逻点 / 过洞对正点）和所有区域全部由人手填在 decision.yaml 里，
//      代码不含任何硬编码坐标、不含硬编码区域 —— 换点、挪洞、换地图都不用改代码；
//   2) "要不要过洞"按 **region** 判（与主工作区同款语义）：区域是用户自定义的名字表
//      （走廊可以就两个区域 left/right，正式场地可以五个 own_low…enemy_high），
//      洞的两个对正点各自声明它服务的区域；车当前区域与目标点区域不同 → 必须过洞，
//      入口 = 绑定"车当前区域"的对正点，出口 = 绑定"目标区域"的那个。多洞 BFS 串联；
//   3) 数据一律从真实 ROS 话题进出（见 RosPorts），仿真数据由 Arena 桥按同一套
//      话题发进来，没有任何进程内注水通道；
//   4) 地图对齐：点表按场地图纸系填，field_to_map（旋转 + 平移）在加载时把全部
//      点搬到导航建图系（tools/align_map.py 解算，与主工作区同款）。
#pragma once

#include <cmath>
#include <cstdint>
#include <string>
#include <vector>

namespace bbt_decide
{

// 二维点 / 二维向量。单位 m。不带坐标系信息：点表按场地系（图纸/STL）填，
// config 加载时经 field_to_map 统一搬到导航地图系，之后全场只有地图系。
struct Vec2
{
  double x = 0.0;  // X 坐标，单位 m
  double y = 0.0;  // Y 坐标，单位 m
};

// 场地系（图纸/STL）→ 导航建图系的 2D 刚体变换（旋转 + 平移，无缩放）。
// 由 tools/align_map.py 用"洞中心：场地系坐标 ↔ 建图系实测坐标"解算并写进 yaml；
// enabled = false 时两个系视为重合（仿真里通常不需要，Arena 桥自己做了镜像）。
struct FrameTransform
{
  bool enabled = false;   // 是否启用变换；false = 恒等
  double dx = 0.0;        // 平移量 x，单位 m
  double dy = 0.0;        // 平移量 y，单位 m
  double dyaw_deg = 0.0;  // 旋转量，单位度，逆时针为正
};

// 把一个点从场地系变换到地图系：先绕原点旋转 dyaw_deg，再平移 (dx, dy)。
// 参数：p — 场地系点；t — 变换参数。返回：地图系点。纯函数。
inline Vec2 apply_frame_transform(const Vec2& p, const FrameTransform& t)
{
  if (!t.enabled) return p;
  const double a = t.dyaw_deg * 3.14159265358979323846 / 180.0;
  const double c = std::cos(a);
  const double s = std::sin(a);
  return Vec2{p.x * c - p.y * s + t.dx, p.x * s + p.y * c + t.dy};
}

// 点的用途分类。home/patrol 是基础两类；outpost/retake 只在前哨争夺开着时参与选点。
enum Zone : int
{
  kZoneHome = 0,     // 家 = 补给区：血量低时回这里
  kZonePatrol = 1,   // 巡逻点：两个前哨阵地都被我方完全控制后按环形顺序轮换着走
  kZoneOutpost = 2,  // 对面前哨攻击位：站在上面激活对面前哨阵地的增益点 + 打前哨站
                     // （"打前哨" = 到这个点 + 告诉电控优先打对面前哨，见 attack_cmd）
  kZoneRetake = 3,   // 我方前哨夺回位：站在上面激活我方前哨阵地的增益点把控制权抢回来
  kZoneCount = 4     // 分类总数（校验用，不是一个分类）
};

// 前哨阵地控制状态（裁判 0x0101 的 2-bit 编码，己方视角；-1 = 没收到过数据）。
// 语义来自 2027 规则 5.6.2：完全控制 = 己方占领；临时控制 = 正在转化中；
// 未控制 = 在对方手里（或中立）。己方"完全控制"是决策里"占领"的唯一判据。
constexpr int kCtrlUnknown = -1;  // 没收到过 0x0101/0x0003（未知就当没事，不乱跑）
constexpr int kCtrlFull = 0;      // 己方完全控制
constexpr int kCtrlTemp = 1;      // 己方临时控制
constexpr int kCtrlNone = 2;      // 己方未控制（对方控制 / 争夺中）

// 区域（region）：用户自定义的"场地分片"，只用于**过洞路由**，不是几何多边形。
// yaml 里的 regions: 是一张名字表（名字随意，数量随意），代码里用下标当区域号。
// 走廊可以就 [left, right]；正式场地写 [own_low, own_high, mid, enemy_low, enemy_high]。
//
// 车当前区域 = **纯记账**，不做任何几何判定（不比最近点、不订阅区域话题）：
//   开赛沿写 start_region（开赛时车摆在哪个区，start_region 就配哪个）；
//   每次过洞完成写出口对正点绑定的 region——这是唯一更新点。
// 为什么记账就够（使用者口径）：车只会按决策发的导航目标移动，跨区域的移动必然
// 走"过洞流程"，过洞完成必写出口区域 → 记账自洽，不存在车绕开决策换区的路径。
// （代价：比赛中被外力推着穿过洞/被搬走，记账不会自己纠正——靠洞口"未折叠
// 拒绝裸穿"保护和下一次开赛沿重置兜底；真需要外部修正时再加接口不迟。）

// 点表里的一条记录：一个可以导航过去的点。
// 坐标按场地系手填（加载时统一搬到地图系）；id 全场唯一；region 必填。
struct PointDef
{
  int id = -1;              // 点唯一 id（-1 = 未填）；决策的 target_point 填这个
  Vec2 pos;                 // 点的位置（加载后为地图系），单位 m
  int zone = kZonePatrol;   // 用途分类，见 Zone
  int region = -1;          // 所属区域（cfg.regions 的下标）；-1 = 未填 → 该点不可选
};

// 一段洞（需要折叠云台才能穿的通道）的定义。
// align[k] 是第 k 个对正点，align_region[k] 声明它服务哪个区域 —— 车在哪个区域，
// 就用绑定该区域的对正点当入口，另一个当出口。方向就这么定，不做几何猜测。
// center（可选）= 洞中心，给 tools/align_map.py 当对齐基准，不参与逻辑。
struct TunnelDef
{
  std::string name;                  // 洞名，日志用，不参与逻辑
  Vec2 align[2];                     // 两个对正点，各在洞一侧，单位 m（加载后地图系）
  int align_region[2] = {-1, -1};    // 对正点各自服务的区域下标；未填(-1)的洞整条跳过
  Vec2 center;                       // 洞中心（可选，地图对齐基准）
};

// 裁判系统快照：摘要与 typed 裁判数据合成。
// 门控只看 game_progress（开始/结束）；血量/弹量用于回家锁存；前哨字段驱动前哨争夺。
// 不搞"有效性+新鲜度"判定——能收到比赛状态和血量就行（使用者口径）。没收到过数据时
// game_progress 停在 0（未开始），决策自然不跑。**一切"未知"都用 -1，绝不用 0 顶替**
// （前哨血量 0 = 被打掉，是有效信息，不能和"没收到"混淆——这是 -1 约定的核心原因）。
struct RefereeSnapshot
{
  int game_progress = 0;   // 0 未开始 / 4 进行中 / 5 结束（RoboMaster 裁判系统编码）
  int hp = 0;              // 当前血量（点）
  int hp_max = 1;          // 血量上限（点）；默认 1 只是防除零
  int shots = -1;          // 剩余可发弹量；-1 = 未知（未知时不参与弹量锁存，金币仅透传）
  int coins = -1;          // 金币；-1 = 未知
  // ---- 前哨（2027 规则 5.6.2 前哨阵地；R/B 已由桥按我方颜色映射成我方/对方）----
  int our_outpost_hp = -1;     // 我方前哨站血量；-1 未知；0 = 被打掉（阵地易主时会以 500 血重建）
  int our_outpost_ctrl = kCtrlUnknown;   // 我方前哨阵地控制状态（kCtrl*；未知 = 不参与争夺判定）
  int enemy_outpost_hp = -1;   // 对面前哨站血量；-1 未知；0 = 被打掉
  int enemy_outpost_ctrl = kCtrlUnknown; // 对面前哨阵地控制状态
  // ---- 场地 / 经济（观测用；不直接驱动选点）----
  int our_base_hp = -1;        // 己方基地血量
  int enemy_base_hp = -1;      // 对方基地血量
  int ally_ctrl_deg = -1;     // 己方累计控制度
  int enemy_ctrl_deg = -1;    // 对方累计控制度
  int robot_id = 0;            // 本机 ID（红 <100 / 蓝 ≥101；0 = 未知）
  // ---- 兑换 / 复活（0x020D；自动买的门控用）----
  int revive_available = -1;   // 0 不可复活 / 1 可免费复活 / 2 可金币兑换立即复活；-1 未知
  int revive_cost = -1;        // 兑换立即复活所需金币；-1 未知
  int remote_ammo_pool = -1;   // 队伍 17mm 允许发弹量剩余可兑换数；-1 未知；0 = 别再下单了
  int out_of_combat = -1;      // 是否脱战：1 脱战 / 0 交战中；-1 未知。远程兑换血量要求脱战
  double stage_remain_s = -1.0; // 当前阶段剩余时间，单位 s；-1 = 未填
};

// 操作手指令（经裁判系统通道进来）：本决策层只消费**指点**（type=4，让车去某个
// 点表点）。买弹/买血/买复活（type 1–3）不是决策层的事——裁判客户端/电控直接办，
// 收到也只记下序号忽略；效果从裁判数据里看（血量/弹量涨了就是买到了）。
// seq 是指令序号（发送侧每条 +1，0 = 无指令）：决策层按它去重，同一条只处理一次。
// 实车链路（裁判 0x0303 小地图点击）给的是**坐标**而不是点表 id：type=4 时
// param = -1、x/y = 点击坐标，决策层把它吸附到最近的点表点（半径 op_snap_m 内才接）。
struct OpCmd
{
  uint32_t seq = 0;   // 指令序号；0 = 没有指令
  int type = 0;       // 0 无 / 1 买弹 / 2 买血 / 3 买复活（这三类忽略）/ 4 指点 /
                      // 5 点了某个对方机器人头像（param = 机器人 ID；自瞄的事，忽略）
  int param = 0;      // 4:点表 id（sim 链路）或 -1（实车坐标指点，看 x/y）；其他类型决策层不看
  double x = 0.0;     // 4 且 param=-1 时的点击坐标（场地系，单位 m）
  double y = 0.0;
};

// 话题接线表：抽象 Port → 具体话题名。
// 仿真和实车只是话题名可能不同（当前两套默认相同），数据都从真实 ROS 话题进出。
struct TopicMap
{
  // 哨兵导航
  std::string nav_goal;    // 导航目标点（发布）：PoseStamped，map 系
  std::string nav_status;  // 导航状态（订阅）：Int32 0 idle/1 running/2 success/3 failure
  std::string nav_cancel;  // 取消导航（发布）：Empty
  std::string odom;        // 位姿（订阅）：Odometry（到达兜底 + 洞口保护用）
  // 电控
  std::string cmd_vel;       // 底盘速度（发布）：Twist（本包只发 0 停车，不发巡航速度）
  std::string chassis_mode;  // 底盘模式（发布）：Int32 0 RELAX/1 INDEPENDENT/2 ROTOR/3 FOLLOW
  std::string spin;          // 小陀螺（发布）：Float32，rad/s；过洞期间恒发 0
  std::string fold_cmd;      // 折叠指令（发布）：UInt8 1 折叠 / 2 展开
  std::string fold_state;    // 折叠状态（订阅）：Int32 0–4，编码同 FoldState
  // 裁判系统
  std::string referee;       // 裁判摘要（订阅）：Float64MultiArray，实车 14 元素；
                             // 同时兼容旧 BBT 完整 19 元素及 Arena 9/12 元素
  std::string op_cmd;        // 操作手指令（订阅）：Int32MultiArray [type, param, seq]。
                             // 决策层只消费 type=4 指点（param=点表 id）；买类(1–3)不是
                             // 决策层的事，收到即忽略（操作手经裁判客户端/电控直接办）。
                             // 可留空 = 用 op_cmd_xy（两者至少接一个）
  std::string op_cmd_xy;     // 操作手指令·坐标版（订阅）：Float64MultiArray
                             // [type, param, seq, x, y]。实车链路用（裁判 0x0303 给的是
                             // 点击坐标）；可留空 = 用 op_cmd
  std::string buy_ammo;      // 自动买弹请求（发布）：Int32 发数（裁判客户端/电控执行）
  std::string buy_hp;        // 自动买血请求（发布）：Int32 量
  std::string buy_revive;    // 自动买复活请求（发布）：Int32 1（hp==0 即阵亡，不需要
                             // 单独的阵亡判定）
  std::string attack_cmd;    // 自瞄意图提示（发布）：Int32MultiArray(4)
                             // [target_type, target_id, fire, free_fire]（打前哨时
                             // type=2/id=1=对面前哨；电控自瞄可参考可忽略）
  // 数据源声明（订阅）：String "sim"/"real"。实车模式下听到 "sim" 立即判裁判无效并停车
  std::string data_source;
  // 接口数据流统计（发布）：String(JSON) 1 Hz，用来确认"数据确实从真实接口进来了"
  std::string iface_status;

  // 做什么：必填话题非空检查。op_cmd / op_cmd_xy 二选一（至少一个非空）。
  // 参数：err — 输出，第一个空字段名；成功时不改。返回：true = 全部非空。
  bool all_named(std::string& err) const
  {
    struct Item { const char* n; const std::string* v; };
    const Item items[] = {
        {"nav_goal", &nav_goal}, {"nav_status", &nav_status}, {"nav_cancel", &nav_cancel},
        {"odom", &odom}, {"cmd_vel", &cmd_vel},
        {"chassis_mode", &chassis_mode}, {"spin", &spin}, {"fold_cmd", &fold_cmd},
        {"fold_state", &fold_state}, {"referee", &referee},
        {"buy_ammo", &buy_ammo}, {"buy_hp", &buy_hp}, {"buy_revive", &buy_revive},
        {"attack_cmd", &attack_cmd},
        {"data_source", &data_source}, {"iface_status", &iface_status}};
    for (const auto& it : items)
    {
      if (it.v->empty())
      {
        err = it.n;
        return false;
      }
    }
    if (op_cmd.empty() && op_cmd_xy.empty())
    {
      err = "op_cmd/op_cmd_xy";
      return false;
    }
    return true;
  }
};

// 话题表出厂默认值：与 decision.yaml 及 ROS 适配层的现行契约一致。
// 操作手坐标指点使用 /referee/operator_cmd；仿真 ID 指点使用 /sentry/operator_point。
inline TopicMap default_topic_map()
{
  TopicMap t;
  t.nav_goal = "/goal_pose";
  t.nav_status = "/nav_status";
  t.nav_cancel = "/nav_cancel";
  t.odom = "/odom";
  t.cmd_vel = "/cmd_vel";
  t.chassis_mode = "/move_mode";
  t.spin = "/cmd_spin";
  t.fold_cmd = "/gimbal_fold_cmd";
  t.fold_state = "/gimbal_fold_state";
  t.referee = "/referee/summary";
  t.op_cmd = "/sentry/operator_point";      // Int32 ID 指点；留空 = 不接
  t.op_cmd_xy = "/referee/operator_cmd";    // Float64 坐标指点；留空 = 不接
  t.buy_ammo = "/sentry/buy_ammo";
  t.buy_hp = "/sentry/buy_hp";
  t.buy_revive = "/sentry/buy_revive";
  t.attack_cmd = "/sentry/attack_cmd";
  t.data_source = "/sentry/data_source";
  t.iface_status = "/sentry/iface_status";
  return t;
}

// Config — 全部可调参数 + 手填配置表（一份 yaml 对应这个结构）。加载后当常量用。
// 注意：field_to_map 在加载末尾生效 —— points / tunnels 的坐标全部被搬到地图系。
struct Config
{
  // ---- 过洞总开关（显式）----
  // false = 这套车不过洞：region 表、点的 region 字段、洞表**全部不需要**（region
  // 与过洞功能耦合在一起——region 只为过洞路由而存在）。所有点视为互相直连，
  // 任何目标直接发导航。true = 按 region 路由过洞（默认）。
  bool enable_tunnel = true;

  // ---- 前哨争夺总开关（显式）----
  // true（默认）= 参与前哨争夺：我方前哨被打掉/阵地失守 → 去 retake 点；
  // 对面前哨阵地不被我方完全控制 → 去 outpost 点；两边阵地都被我方完全控制 → 巡逻。
  // 判据只有 0x0003 的前哨血量 + 0x0101 的前哨阵地控制状态，不需要雷达。
  // 数据未知（-1）的一侧不参与判定（不知道就当没事，别乱跑）。
  // false = 整套前哨逻辑不存在（走廊等没有前哨的地图），outpost/retake 点也不需要。
  bool enable_outpost = true;
  double op_goal_timeout_s = 120.0; // 指点过期后恢复规则决策；跨洞恢复仍须先完成
  double op_snap_m = 1.5;  // 操作手坐标指点吸附半径：点击处 m 内最近的点表点才算数，
                           // 半径内没有点 = 忽略这次点击（防误点墙上/场外）

  // ---- 手填配置表（决策只引用 id / 区域下标，坐标全在这里改）----
  std::vector<std::string> regions;  // 区域名表（下标 = 区域号）；换地图改这张表。
                                     // 仅 enable_tunnel = true 时必填
  int start_region = 0;              // 开局所在区域（下标）：开赛沿把记账重置成它，
                                     // 之后唯一的更新点是每次过洞完成写出口区域
  std::vector<PointDef> points;      // 点表：家 + 巡逻点（enable_tunnel=true 时各绑区域）
  std::vector<TunnelDef> tunnels;    // 洞表：每条两个对正点 + 各自服务的区域
  FrameTransform field_to_map;       // 场地系 → 地图系（align_map.py 解算；加载时生效）
  bool require_map_alignment = false; // 正式地图实车必须显式标定，恒等也要 enabled=true

  // ---- 阈值（单位见各字段注释）----
  double hp_home_ratio = 0.30;    // 血量比例低于此值 → 回家（低阈值触发）
  double hp_resume_ratio = 0.60;  // 回家后血量回到此值才解除锁存（高阈值恢复，防抖）
  int ammo_home_min = 50;         // 弹量低于此值 → 同样锁存回家补给；-1 = 不按弹量回家
  int ammo_resume_min = 150;      // 回家后弹量回到此值才算恢复（血量、弹量**同时**
                                  // 恢复才解除锁存，缺一个都不出门）；弹量未知(-1)不拦
  double spin_w = 1.5;            // 受击旋转角速度 rad/s：所有非过洞导航段恒转
                                  // （含巡逻段），过洞段恒 0（姿态必须稳定）

  // ---- 自动购买（决策层自己下单；操作手的买指令不经这里）----
  // 判定只用裁判数据（血量/弹量/金币）+ 金币门槛 + 冷却。远程兑换有延迟：
  // 下单后冷却 buy_cooldown_s 内不重复发；买没买到看裁判数据——血量/弹量涨了
  // 触发条件自然消失，没涨（钱不够/兑换失败）冷却过了会再试。
  bool enable_auto_buy = true;    // 总开关（false = 三种都不买，输出口也不发）
  int ammo_buy_min_shots = 30;    // 弹量低于此值 → 下单买弹；-1 = 不自动买弹
  int ammo_buy_rounds = 100;      // 每次买弹发数
  int ammo_buy_min_coins = 200;   // 金币低于此值不买弹（给自己留复活钱自己权衡着改）
  double hp_buy_ratio = 0.40;     // 血量比例低于此值（且活着）→ 下单买血；<=0 = 不买血。
                                  // 故意比 hp_home_ratio(0.30) 高：先试远程买血回一口，
                                  // 买不到再走回家那条路
  int hp_buy_amount = 1;          // 一次远程回血请求；恢复量由裁判规则决定
  int hp_buy_min_coins = 300;     // 金币低于此值不买血
  int revive_buy_min_coins = 200; // 金币 ≥ 此值且 hp==0（=阵亡，不需要单独的阵亡判定）
                                  // → 下单买复活；-1 = 不自动买复活
  double buy_cooldown_s = 10.0;   // 同一类购买的下单间隔（等远程兑换到账）。
                                  // 比赛外不下单（比赛外一条消息不发；备赛采购是
                                  // 操作手手动的事）
  double odom_stale_ms = 1000.0;  // 位姿超过这么久没更新 → 当作无效（区域保持记账、
                                  // 到达只认导航 success、洞口保护不判——全部朝安全方向退化）
  double patrol_dwell_s = 15.0;   // 巡逻：到点或待够这么久才换下一个巡逻点
  double switch_dwell_s = 2.0;    // 换点最小驻留（防抖，低于此时间的换点请求被压住）
  int switch_confirm_frames = 3;  // 换点需连续一致帧数（防抖）
  double gate_clear_m = 0.5;      // 点离闸线（两对正点连线）的最小距离，太近不算可用点
  double nav_timeout_s = 25.0;    // 单段导航超时：到对正点/到巡逻点超过这么久算失败
  double nav_retry_s = 2.0;       // 导航失败后的重试间隔（重试节奏，防打爆导航）
  double fold_timeout_s = 3.0;    // 折叠/展开确认超时
  double fold_stale_ms = 1000.0;  // 折叠反馈断流后按未知处理
  double tunnel_retry_cooldown_s = 5.0;  // 过洞失败后的重试冷却
  double in_tunnel_margin_m = 0.3; // 车离闸线中段小于此距离视为"已在洞口/洞内"
  double arrive_radius_m = 0.5;   // 距离小于此值算"已站在点上"（nav 没报 success 时的兜底）

  // ---- 运行模式 ----
  int mode_independent = 1;  // 底盘模式枚举里的 INDEPENDENT（开局切一次）

  // 接线表：data_source=sim 用 topics_sim，=real 用 topics_real
  TopicMap topics_sim = default_topic_map();
  TopicMap topics_real = default_topic_map();
};

// 决策树一帧的输出。执行层（Runner）拿它去发导航目标/折叠指令。
struct Decision
{
  int target_point = -1;   // 点表 id；-1 = 原地保持（停车）
  int tunnel_id = -1;      // 洞表下标；-1 = 本帧不需要过洞
  uint64_t stamp_ms = 0;   // 决策生成时刻，单调毫秒
};

// 加载 yaml 配置（与 decision.yaml 一一对应）。
// 参数：path — yaml 文件路径；cfg — 输出，加载成功时整体替换（坐标已搬到地图系）。
// 参数：error — 输出，失败原因；warnings — 输出，不致命的告警列表。
// 返回：true = 成功。失败时 cfg 内容未定义。
bool load_config(const std::string& path, Config& cfg, std::string& error,
                 std::vector<std::string>& warnings);

}  // namespace bbt_decide
