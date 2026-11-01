// BBT 纯逻辑测试：不依赖 ROS，直接驱动 DebugPorts 跑"感知→决策树→执行器"。
// 覆盖使用者的核心要求（region 方案）：
//   ① 随意换地图：走廊里自定义 left/right 两个区域 + 一个洞两个对正点，
//      巡逻、折叠过洞全流程能跑；
//   ② 区域名/分布完全不同的另一张图（三区域双洞串联）照跑，BFS 路由正确；
//   ③ 过洞判定与入口选择完全按 align_region，完成过洞自动记账区域；
//   ④ 孤岛区域的点选不到；回家锁存、比赛门控、洞口未折叠拒绝裸穿、地图对齐。
#include <cmath>
#include <cstdio>
#include <fstream>
#include <string>

#include "bbt_decide/planner.hpp"
#include "bbt_decide/types.hpp"

using namespace bbt_decide;

static int g_failures = 0;
// 断言失败就打文件行号，不中断（一次跑完能看到所有问题）。
#define CHECK(cond)                                                    \
  do                                                                   \
  {                                                                    \
    if (!(cond))                                                       \
    {                                                                  \
      ++g_failures;                                                    \
      std::printf("FAIL %s:%d: %s\n", __FILE__, __LINE__, #cond);      \
    }                                                                  \
  } while (0)

// ---------------- 测试脚手架 ----------------

// 填一帧"比赛进行中、血量 hp/400"的裁判快照（stamp 与模拟时钟同步，保证新鲜）。
static void set_match(DebugPorts& p, int progress, int hp)
{
  p.ref.game_progress = progress;
  p.ref.hp = hp;
  p.ref.hp_max = 400;
}

// 让"导航到达"成立：把车挪到目标处并报 success。
static void arrive(DebugPorts& p, double x, double y)
{
  p.pose.x = x;
  p.pose.y = y;
  p.pose.valid = true;
  p.goal = kGoalSuccess;
}

// 推进一步（时间也推进一小段）。每 tick 从 Ports 同步区域（region=-1 时保持记账）。
// pose.stamp_ms 同步刷成当前模拟时钟：模拟 odom 持续在发（断流用例绕过本 helper）。
static int tick(DebugPorts& p, const Config& c, PlanState& plan, ExecState& ex, double dt = 0.05)
{
  p.time_s += dt;
  p.pose.stamp_ms = static_cast<uint64_t>(p.time_s * 1000.0);
  return step(p, c, plan, ex, p.time_s);
}

// 走廊场景配置（region 版）：自定义 left/right 两个区域，中间一个洞。
static Config corridor_cfg()
{
  Config c;
  c.regions = {"left", "right"};
  c.start_region = 0;  // left
  c.switch_confirm_frames = 1;  // 测试里关掉防抖节奏，逐 tick 生效
  c.switch_dwell_s = 0.0;
  c.patrol_dwell_s = 1.0;
  c.points = {{1, {1.0, 5.0}, kZoneHome, 0},
              {10, {5.0, 5.0}, kZonePatrol, 0},
              {11, {15.0, 5.0}, kZonePatrol, 1},
              {12, {19.0, 5.0}, kZonePatrol, 1}};
  TunnelDef t;
  t.name = "corridor_hole";
  t.align[0] = {9.0, 5.0};
  t.align_region[0] = 0;   // left 侧对正点
  t.align[1] = {11.0, 5.0};
  t.align_region[1] = 1;   // right 侧对正点
  t.center = {10.0, 5.0};
  c.tunnels = {t};
  return c;
}

// ---------------- ① 走廊自定义区域：巡逻 + 折叠过洞全流程 ----------------
static void test_corridor_regions()
{
  DebugPorts p;
  p.pose = {2.0, 5.0, 0.0, 0, true};
  p.fold = kFoldUnfolded;
  Config c = corridor_cfg();
  PlanState plan;
  plan.current_region = c.start_region;  // 节点里也这么初始化
  ExecState ex;

  set_match(p, 4, 400);
  tick(p, c, plan, ex);
  CHECK(p.goal_count == 1);          // 第一个目标
  CHECK(p.goal_x == 5.0 && p.goal_y == 5.0);  // 最近巡逻点 id10（left 侧）

  // 到达 id10 → 换下一个巡逻点 id11（right 侧）→ 必须过洞：入口 = left 侧对正点
  arrive(p, 5.0, 5.0);
  tick(p, c, plan, ex);
  CHECK(ex.tunnel.active());
  CHECK(p.goal_count == 2);
  CHECK(p.goal_x == 9.0 && p.goal_y == 5.0);  // 绝不直发跨区域的 (15,5)
  CHECK(p.fold_cmd_count == 0);               // 还没折叠
  CHECK(ex.tunnel.exit_region == 1);

  // 到入口 → 折叠
  arrive(p, 9.0, 5.0);
  tick(p, c, plan, ex);  // kWaitEntry 看到 success → kFold
  tick(p, c, plan, ex);  // kFold → 下发折叠（DebugPorts 立即变 Folded）
  CHECK(p.fold_cmd_count == 1);
  CHECK(p.fold == kFoldFolded);

  // 折叠完成 → 穿洞（导到 right 侧对正点 (11,5)）
  tick(p, c, plan, ex);  // kWaitFold → kNavExit
  tick(p, c, plan, ex);  // kNavExit → 发出口目标
  CHECK(p.goal_count == 3);
  CHECK(p.goal_x == 11.0 && p.goal_y == 5.0);
  CHECK(p.fold_cmd_count == 1);  // 出口目标发出时云台必须保持折叠

  // 到出口 → 停车 → 展开 → 完成（区域记账变成 right）
  arrive(p, 11.0, 5.0);
  tick(p, c, plan, ex);  // kWaitExit success → 停车 + kUnfold
  tick(p, c, plan, ex);  // kUnfold → 下发展开
  CHECK(p.unfold_cmd_count == 1);
  tick(p, c, plan, ex);  // kWaitUnfold → 完成，交回常规导航
  CHECK(!ex.tunnel.active());
  CHECK(p.fold == kFoldUnfolded);
  CHECK(plan.current_region == 1);  // 过洞完成 → 记账区域 = 出口对正点绑定的区域

  // 交回常规导航：继续去 id11 (15,5)（现在同区域，不再过洞）
  tick(p, c, plan, ex);
  CHECK(p.goal_count == 4);
  CHECK(p.goal_x == 15.0 && p.goal_y == 5.0);
  CHECK(!ex.tunnel.active());

  // 再换 id12 (19,5)：同在 right，直达
  arrive(p, 15.0, 5.0);
  tick(p, c, plan, ex);
  tick(p, c, plan, ex);
  CHECK(p.goal_x == 19.0 && p.goal_y == 5.0);
  CHECK(!ex.tunnel.active());
}

// ---------------- ② 换一张完全不同的地图：三区域双洞串联 ----------------
static void test_renamed_map_chained()
{
  Config c;
  c.regions = {"north", "middle", "south"};  // 区域名随便起
  c.start_region = 0;
  c.switch_confirm_frames = 1;
  c.switch_dwell_s = 0.0;
  c.patrol_dwell_s = 1.0;
  c.points = {{1, {1.0, 5.0}, kZoneHome, 0},
              {30, {8.0, 5.0}, kZonePatrol, 1},
              {31, {13.0, 5.0}, kZonePatrol, 2}};
  TunnelDef t1, t2;
  t1.name = "gate1";
  t1.align[0] = {5.0, 5.0};
  t1.align_region[0] = 0;
  t1.align[1] = {7.0, 5.0};
  t1.align_region[1] = 1;
  t2.name = "gate2";
  t2.align[0] = {9.0, 5.0};
  t2.align_region[0] = 1;
  t2.align[1] = {11.0, 5.0};
  t2.align_region[1] = 2;
  c.tunnels = {t1, t2};

  // 路由：north → south 必须依次过两条洞
  const auto route = route_tunnels(c, 0, 2);
  CHECK(route.size() == 2);
  CHECK(route[0] == 0 && route[1] == 1);
  CHECK(find_tunnel(c, 0, 1) == 0);
  CHECK(find_tunnel(c, 1, 2) == 1);
  CHECK(find_tunnel(c, 0, 2) == -1);  // 没有直连洞，必须中转

  DebugPorts p;
  p.pose = {1.0, 5.0, 0.0, 0, true};
  p.fold = kFoldUnfolded;
  PlanState plan;
  plan.current_region = 0;
  ExecState ex;

  set_match(p, 4, 400);
  // 最近巡逻点 id30（middle 侧）→ 车在 north，跨区域 → 立刻启动过第一洞
  tick(p, c, plan, ex);
  CHECK(ex.tunnel.active());
  CHECK(ex.tunnel.tunnel == 0);
  CHECK(p.goal_x == 5.0);  // 入口 = north 侧对正点

  arrive(p, 5.0, 5.0);
  tick(p, c, plan, ex);
  tick(p, c, plan, ex);  // 折叠
  CHECK(p.fold_cmd_count == 1);
  tick(p, c, plan, ex);
  tick(p, c, plan, ex);  // 出口 (7,5)
  CHECK(p.goal_x == 7.0);
  arrive(p, 7.0, 5.0);
  tick(p, c, plan, ex);
  tick(p, c, plan, ex);  // 展开
  tick(p, c, plan, ex);  // 完成 → 记账区域 = middle
  CHECK(!ex.tunnel.active());
  CHECK(plan.current_region == 1);

  // 同区域：常规导航直达 id30
  tick(p, c, plan, ex);
  CHECK(!ex.tunnel.active());
  CHECK(p.goal_x == 8.0);

  // 到 id30 后换 id31（south）→ 自动过第二洞
  arrive(p, 8.0, 5.0);
  tick(p, c, plan, ex);  // 决策换 id31 + 启动过洞
  CHECK(ex.tunnel.active());
  CHECK(ex.tunnel.tunnel == 1);
  CHECK(p.goal_x == 9.0);  // 入口 = middle 侧对正点
  arrive(p, 9.0, 5.0);
  tick(p, c, plan, ex);
  tick(p, c, plan, ex);  // 折叠
  CHECK(p.fold_cmd_count == 2);
  tick(p, c, plan, ex);
  tick(p, c, plan, ex);  // 出口 (11,5)
  CHECK(p.goal_x == 11.0);
  arrive(p, 11.0, 5.0);
  tick(p, c, plan, ex);
  tick(p, c, plan, ex);  // 展开
  tick(p, c, plan, ex);  // 完成 → 记账区域 = south
  CHECK(!ex.tunnel.active());
  CHECK(plan.current_region == 2);

  tick(p, c, plan, ex);  // 同区域直达 id31
  CHECK(!ex.tunnel.active());
  CHECK(p.goal_x == 13.0);
}

// ---------------- ③ 区域纯记账：开赛沿重置 + 过洞完成写出口，零区域输入 ----------------
static void test_region_bookkeeping()
{
  Config c = corridor_cfg();
  DebugPorts p;
  p.fold = kFoldUnfolded;
  PlanState plan;
  plan.current_region = 1;   // 上一场的残留记账（比如 right）
  ExecState ex;

  // 开赛沿：记账重置回 start_region（left）。没有重置的话 id10 会变跨区域目标、
  // 直接启动过洞——这一拍就是"重置生效"的证据。位姿只影响选哪个巡逻点。
  set_match(p, 4, 400);
  p.pose = {2.0, 5.0, 0.0, 0, true};
  tick(p, c, plan, ex);
  CHECK(plan.current_region == 0);
  CHECK(p.goal_x == 5.0);    // id10 同区直达
  CHECK(!ex.tunnel.active());

  // 过洞完成：记账 = 出口对正点的区域（唯一更新点），继续去 id11 不再折返
  arrive(p, 5.0, 5.0);
  tick(p, c, plan, ex);      // 换 id11 → 过洞入口 (9,5)
  arrive(p, 9.0, 5.0);
  tick(p, c, plan, ex);      // 到入口 → 折叠
  tick(p, c, plan, ex);      // 折叠
  tick(p, c, plan, ex);      // 折叠到位 → 出口目标
  tick(p, c, plan, ex);      // 发出口 (11,5)
  arrive(p, 11.0, 5.0);
  tick(p, c, plan, ex);      // 到出口 → 展开
  tick(p, c, plan, ex);      // 展开
  tick(p, c, plan, ex);      // 完成：记账 = right
  CHECK(!ex.tunnel.active());
  CHECK(plan.current_region == 1);
  tick(p, c, plan, ex);      // 完成的下一拍：交回常规导航
  CHECK(p.goal_x == 15.0);   // id11 同区直达

  // 比赛结束 → 再开赛：记账再次重置回 start_region
  set_match(p, 5, 400);
  tick(p, c, plan, ex);
  set_match(p, 4, 400);
  p.pose = {2.0, 5.0, 0.0, 0, true};
  tick(p, c, plan, ex);
  CHECK(plan.current_region == 0);
}

// ---------------- ③c odom 断流：位姿过期只影响"到达兜底"，与区域无关 ----------------
static void test_odom_stale()
{
  Config c = corridor_cfg();
  c.patrol_dwell_s = 10.0;  // 本用例只看"到达"，不让驻留超时抢先换点
  DebugPorts p;
  p.fold = kFoldUnfolded;
  PlanState plan;
  plan.current_region = 0;
  ExecState ex;

  set_match(p, 4, 400);
  p.pose = {5.6, 5.0, 0.0, 0, true};  // 站在 id10 半径外一点
  tick(p, c, plan, ex);
  CHECK(p.goal_x == 5.0);             // 目标 id10

  // odom 断流 1.5 s：位姿过期 → 半径到达不生效（导航没报 success 就不算到），不跳点
  p.time_s += 1.5;
  p.pose = {5.2, 5.0, 0.0, 0, true};  // 其实已经站进 0.5 m 半径
  p.pose.stamp_ms = static_cast<uint64_t>((p.time_s - 1.5) * 1000.0);  // 过期时间戳
  step(p, c, plan, ex, p.time_s);
  CHECK(plan.cur_point == 10);

  // odom 恢复（时间戳新鲜）→ 半径到达生效 → 轮换下一个
  p.pose.stamp_ms = static_cast<uint64_t>(p.time_s * 1000.0);
  step(p, c, plan, ex, p.time_s);
  CHECK(plan.cur_point == 11);
}

// ---------------- ⑤ 孤岛区域：点选不到 ----------------
static void test_isolated_region()
{
  Config c;
  c.regions = {"a", "b", "island"};
  c.start_region = 0;
  c.switch_confirm_frames = 1;
  c.switch_dwell_s = 0.0;
  c.points = {{1, {1.0, 5.0}, kZoneHome, 0},
              {10, {5.0, 5.0}, kZonePatrol, 0},
              {11, {9.0, 5.0}, kZonePatrol, 2}};  // island：没有任何洞连着
  TunnelDef t;
  t.name = "ab";
  t.align[0] = {6.0, 5.0};
  t.align_region[0] = 0;
  t.align[1] = {8.0, 5.0};
  t.align_region[1] = 1;
  c.tunnels = {t};

  CHECK(!regions_reachable(c, 0, 2));   // island 不可达
  CHECK(regions_reachable(c, 0, 1));    // b 经洞可达
  CHECK(!point_usable(c, 0, c.points[2]));  // island 的点不可选
  CHECK(point_usable(c, 0, c.points[1]));

  // 路由检查：a → b 走洞
  const auto route = route_tunnels(c, 0, 1);
  CHECK(route.size() == 1 && route[0] == 0);
}

// ---------------- ⑤ 回家锁存（家在洞另一侧也要回去） ----------------
static void test_home_latch_across_tunnel()
{
  Config c = corridor_cfg();
  DebugPorts p;
  p.pose = {2.0, 5.0, 0.0, 0, true};
  p.fold = kFoldUnfolded;
  PlanState plan;
  plan.current_region = 0;
  ExecState ex;

  // 把家换到 right 侧，血量 25% < 30% → 锁存回家（必须过洞回去）
  c.points[0].region = 1;
  set_match(p, 4, 100);
  tick(p, c, plan, ex);
  CHECK(plan.cur_point == 1);
  tick(p, c, plan, ex);  // 启动过洞回 right 侧的家
  CHECK(ex.tunnel.active());
  CHECK(p.goal_x == 9.0);  // 入口对正点
}

// ---------------- ⑥ 比赛门控：沿触发收尾 + 比赛外完全静默 ----------------
static void test_gating()
{
  Config c = corridor_cfg();
  DebugPorts p;
  p.pose = {2.0, 5.0, 0.0, 0, true};
  p.fold = kFoldUnfolded;
  PlanState plan;
  plan.current_region = 0;
  ExecState ex;

  // 比赛未开始（progress=0，含"从没收到过裁判数据"的默认值）→ 完全静默：
  // 一条消息都不发（没有可终止的东西，也就没有收尾）
  set_match(p, 0, 400);
  tick(p, c, plan, ex);
  tick(p, c, plan, ex);
  CHECK(p.goal_count == 0);
  CHECK(p.cancel_count == 0);
  CHECK(p.stop_count == 0);
  CHECK(p.spin_count == 0);
  CHECK(p.mode_count == 0);

  // 比赛开始（上升沿）：切一次底盘模式，正常选点巡逻，非过洞段受击旋转恒发
  set_match(p, 4, 400);
  tick(p, c, plan, ex);
  CHECK(p.mode_count == 1);
  CHECK(p.goal_count == 1);
  CHECK(p.goal_x == 5.0);
  CHECK(p.spin_last_w == c.spin_w);

  // 裁判断流（数据不再更新）：不做新鲜度判定，按最后缓存的比赛状态继续跑
  tick(p, c, plan, ex);
  tick(p, c, plan, ex);
  CHECK(p.goal_count == 1);  // 同一个目标，继续导航，没有落待机
  CHECK(p.mode_count == 1);  // 模式只切一次，不重复发

  // 比赛结束（下降沿）：一次性收尾——取消导航 + 停自旋（不给电控发停车，
  // 取消导航后底盘自己停），之后完全静默
  set_match(p, 5, 400);
  tick(p, c, plan, ex);
  CHECK(p.cancel_count == 1);
  CHECK(p.spin_last_w == 0.0);
  CHECK(p.stop_count == 0);
  const int goal_after_end = p.goal_count;
  const int spin_after_end = p.spin_count;
  tick(p, c, plan, ex);
  tick(p, c, plan, ex);
  CHECK(p.goal_count == goal_after_end);  // 静默：不再发任何消息
  CHECK(p.spin_count == spin_after_end);
  CHECK(p.cancel_count == 1);
  CHECK(p.mode_count == 1);

  // 再开赛：模式再切一次（每次开赛沿各一次），导航恢复
  set_match(p, 4, 400);
  tick(p, c, plan, ex);
  CHECK(p.mode_count == 2);
  CHECK(p.goal_count == goal_after_end + 1);
}

// ---------------- ⑥c 弹量锁存：低弹回家，血量+弹量都恢复才出门 ----------------
static void test_ammo_latch()
{
  Config c = corridor_cfg();  // 默认 ammo_home_min=50 / ammo_resume_min=150
  DebugPorts p;
  p.pose = {2.0, 5.0, 0.0, 0, true};
  p.fold = kFoldUnfolded;
  PlanState plan;
  plan.current_region = 0;
  ExecState ex;

  // 弹量未知(-1)：不参与判定，血量满 → 巡逻
  set_match(p, 4, 400);
  tick(p, c, plan, ex);
  CHECK(!plan.home_latched);
  CHECK(plan.cur_point == 10);

  // 弹量 30 < 50：锁存回家（弹量掉了也该回家补给）
  p.ref.shots = 30;
  tick(p, c, plan, ex);
  CHECK(plan.home_latched);
  CHECK(plan.cur_point == 1);  // 家

  // 操作手买了弹（弹量 200）且血量本来就满 → 解除，恢复巡逻
  p.ref.shots = 200;
  tick(p, c, plan, ex);
  CHECK(!plan.home_latched);
  CHECK(plan.cur_point == 10);

  // 低血 + 低弹一起触发；血量恢复但弹量没到 150 → 仍然不出门（双条件解除）
  set_match(p, 4, 100);  // 25% < 30%
  p.ref.shots = 40;
  tick(p, c, plan, ex);
  CHECK(plan.home_latched);
  set_match(p, 4, 280);  // 70% ≥ 60%，但 40 < 150
  tick(p, c, plan, ex);
  CHECK(plan.home_latched);
  CHECK(plan.cur_point == 1);
  p.ref.shots = 200;     // 弹也补上了 → 解除
  tick(p, c, plan, ex);
  CHECK(!plan.home_latched);

  // ammo_home_min = -1：弹量锁存整个关掉，只剩血量
  Config c2 = corridor_cfg();
  c2.ammo_home_min = -1;
  PlanState plan2;
  plan2.current_region = 0;
  ExecState ex2;
  DebugPorts p2;
  p2.pose = {2.0, 5.0, 0.0, 0, true};
  p2.fold = kFoldUnfolded;
  set_match(p2, 4, 400);
  p2.ref.shots = 0;  // 弹量 0 也不触发
  tick(p2, c2, plan2, ex2);
  CHECK(!plan2.home_latched);
  CHECK(plan2.cur_point == 10);
}

// ---------------- ⑥d 操作手指令：只消费"指点"，买类忽略 ----------------
static void test_operator_cmds()
{
  Config c = corridor_cfg();
  DebugPorts p;
  p.pose = {2.0, 5.0, 0.0, 0, true};
  p.fold = kFoldUnfolded;
  PlanState plan;
  plan.current_region = 0;
  ExecState ex;

  // --- 买类指令（type 1-3）：不是决策层的事，收到即忽略（也不自动买：
  //     金币未知 -1，宁可不花不错花），一条消息都不产生 ---
  set_match(p, 0, 400);
  p.op = {1, 1, 100};  // 操作手买弹 100 发
  tick(p, c, plan, ex);
  p.op = {2, 2, 60};   // 买血 60
  tick(p, c, plan, ex);
  p.op = {3, 3, 0};    // 买复活
  tick(p, c, plan, ex);
  CHECK(p.buy_ammo_calls == 0);
  CHECK(p.buy_hp_calls == 0);
  CHECK(p.buy_revive_calls == 0);
  CHECK(p.goal_count == 0);  // 比赛外不产生任何运动消息
  CHECK(plan.op_seq_seen == 3);  // 序号已记账（不会反复尝试解析同一条）

  // --- 指点：比赛开始后执行，最高优先级 ---
  set_match(p, 4, 400);
  tick(p, c, plan, ex);        // 巡逻：目标 id10 (5,5)
  CHECK(p.goal_count == 1);
  CHECK(p.spin_last_w == c.spin_w);

  p.op = {4, 4, 10};           // 指点 id10——和当前目标同一点
  tick(p, c, plan, ex);
  CHECK(plan.op_goal == 10);
  CHECK(plan.op_fresh == false);  // 已消费：目标更新 = 取消旧 Goal 重发一次
  CHECK(p.goal_count == 2);
  CHECK(p.cancel_count == 1);
  CHECK(p.goal_x == 5.0);

  // 重指后旧目标的 success 回执不作数（op_fresh 期间不信 success），到点才清除
  arrive(p, 5.0, 5.0);
  tick(p, c, plan, ex);
  CHECK(plan.op_goal == -1);   // 到达：指点消费掉
  tick(p, c, plan, ex);        // 恢复巡逻：下一个 id11（right 侧）→ 过洞入口 (9,5)
  CHECK(ex.tunnel.active());
  CHECK(p.goal_x == 9.0);

  // --- 指点的点不存在：静默忽略 ---
  p.op = {9, 4, 99};
  tick(p, c, plan, ex);
  CHECK(plan.op_goal == -1);
}

// ---------------- ⑥g 自动购买：弹/血低了、金币够就自己下单；到账看裁判数据 ----------------
static void test_auto_buy()
{
  Config c = corridor_cfg();  // 默认：弹 30/100发/200金币，血 0.40/200/300，复活 200，冷却 10 s
  DebugPorts p;
  p.pose = {2.0, 5.0, 0.0, 0, true};
  p.fold = kFoldUnfolded;
  PlanState plan;
  plan.current_region = 0;
  ExecState ex;

  // 比赛外（含备赛阶段）：再缺也不下单（比赛外一条消息不发）
  set_match(p, 0, 400);
  p.ref.shots = 10;
  p.ref.coins = 1000;
  tick(p, c, plan, ex);
  CHECK(p.buy_ammo_calls == 0);

  // 比赛进行 + 弹量 10 < 30 + 金币 1000 → 自动买弹 100 发（就下一次，冷却挡住重复下单）
  set_match(p, 4, 400);
  tick(p, c, plan, ex);
  CHECK(p.buy_ammo_calls == 1);
  CHECK(p.buy_ammo_param == 100);
  tick(p, c, plan, ex);
  tick(p, c, plan, ex);
  CHECK(p.buy_ammo_calls == 1);  // 冷却 10 s 内不重复

  // 远程兑换有延迟：冷却过了弹量还没涨（还是 10）→ 再下一单
  p.time_s += 10.5;
  tick(p, c, plan, ex);
  CHECK(p.buy_ammo_calls == 2);

  // 到账了：弹量涨到 300 → 即使冷却过了也不再下单
  p.ref.shots = 300;
  p.time_s += 10.5;
  tick(p, c, plan, ex);
  CHECK(p.buy_ammo_calls == 2);

  // 金币不足：条件在也不下单
  p.ref.shots = 10;
  p.ref.coins = 100;  // < ammo_buy_min_coins(200)
  p.time_s += 10.5;
  tick(p, c, plan, ex);
  CHECK(p.buy_ammo_calls == 2);

  // 弹量未知（-1）：不判弹量、不买
  p.ref.shots = -1;
  p.ref.coins = 1000;
  p.time_s += 10.5;
  tick(p, c, plan, ex);
  CHECK(p.buy_ammo_calls == 2);

  // --- 自动买血：活着、血量比例 < 0.40、金币够 ---
  p.ref.shots = 300;
  p.ref.coins = 500;
  set_match(p, 4, 150);       // 37.5% < 40%
  tick(p, c, plan, ex);
  CHECK(p.buy_hp_calls == 1);
  CHECK(p.buy_hp_param == 1);
  p.time_s += 10.5;           // 冷却过了还没回血 → 再试
  tick(p, c, plan, ex);
  CHECK(p.buy_hp_calls == 2);
  set_match(p, 4, 300);       // 75% ≥ 40%：到账（或本来就够），不再买
  p.time_s += 10.5;
  tick(p, c, plan, ex);
  CHECK(p.buy_hp_calls == 2);

  // --- 自动买复活：hp == 0 即阵亡（不需要单独的阵亡判定），金币够就下单；
  //     死着不买血（活着才买血）---
  p.ref.coins = 500;
  set_match(p, 4, 0);
  tick(p, c, plan, ex);
  CHECK(p.buy_revive_calls == 1);
  CHECK(p.buy_hp_calls == 2);     // hp==0 不触发买血
  p.time_s += 10.5;
  tick(p, c, plan, ex);
  CHECK(p.buy_revive_calls == 2); // 复活兑换也有延迟，冷却过了再试
  set_match(p, 4, 120);           // 复活了（残血 30%）：停止买复活；
  p.time_s += 10.5;               // 血量 30% < 40% → 转为自动买血（先远程补一口）
  tick(p, c, plan, ex);
  CHECK(p.buy_revive_calls == 2);
  CHECK(p.buy_hp_calls == 3);
  CHECK(plan.home_latched);       // 同时血量锁存保持：从原地接着回家

  // --- 总开关关掉：三类全不买 ---
  Config c2 = corridor_cfg();
  c2.enable_auto_buy = false;
  DebugPorts p2;
  p2.pose = {2.0, 5.0, 0.0, 0, true};
  p2.fold = kFoldUnfolded;
  PlanState plan2;
  ExecState ex2;
  set_match(p2, 4, 0);
  p2.ref.shots = 0;
  p2.ref.coins = 1000;
  tick(p2, c2, plan2, ex2);
  CHECK(p2.buy_ammo_calls == 0);
  CHECK(p2.buy_hp_calls == 0);
  CHECK(p2.buy_revive_calls == 0);
}

// ---------------- ⑥e 受击旋转：非过洞段恒 spin_w，过洞段恒 0 ----------------
static void test_spin_states()
{
  Config c = corridor_cfg();
  c.spin_w = 2.2;
  DebugPorts p;
  p.pose = {2.0, 5.0, 0.0, 0, true};
  p.fold = kFoldUnfolded;
  PlanState plan;
  plan.current_region = 0;
  ExecState ex;

  set_match(p, 4, 400);
  tick(p, c, plan, ex);   // 巡逻导航段（非过洞）：恒 spin_w
  tick(p, c, plan, ex);
  CHECK(p.spin_last_w == 2.2);
  CHECK(p.spin_zero_count == 0);

  arrive(p, 5.0, 5.0);    // 换 id11 → 过洞：全程 spin 0（姿态必须稳定）
  tick(p, c, plan, ex);
  CHECK(ex.tunnel.active());
  CHECK(p.spin_last_w == 0.0);
  arrive(p, 9.0, 5.0);    // 入口 → 折叠 → 出口 → 展开
  tick(p, c, plan, ex);
  tick(p, c, plan, ex);
  CHECK(p.fold == kFoldFolded);
  tick(p, c, plan, ex);
  tick(p, c, plan, ex);   // 出口目标
  CHECK(p.spin_last_w == 0.0);
  arrive(p, 11.0, 5.0);
  tick(p, c, plan, ex);
  tick(p, c, plan, ex);   // 展开
  tick(p, c, plan, ex);   // 展开完成
  CHECK(!ex.tunnel.active());
  tick(p, c, plan, ex);   // 交回常规导航：spin_w 恢复
  CHECK(p.spin_last_w == 2.2);
}

// ---------------- ⑥f 过洞总开关：关掉 = region 整套不需要，点直接去 ----------------
static void test_tunnel_disabled()
{
  Config c;  // 不填 regions / tunnels，点也不标 region
  c.enable_tunnel = false;
  c.switch_confirm_frames = 1;
  c.switch_dwell_s = 0.0;
  c.patrol_dwell_s = 1.0;
  c.points = {{1, {1.0, 5.0}, kZoneHome, -1},
              {10, {5.0, 5.0}, kZonePatrol, -1},
              {11, {15.0, 5.0}, kZonePatrol, -1}};
  DebugPorts p;
  p.pose = {2.0, 5.0, 0.0, 0, true};
  p.fold = kFoldUnfolded;
  PlanState plan;
  ExecState ex;

  set_match(p, 4, 400);
  tick(p, c, plan, ex);
  CHECK(p.goal_count == 1);
  CHECK(p.goal_x == 5.0);   // id10 直达

  arrive(p, 5.0, 5.0);
  tick(p, c, plan, ex);     // id11 (15,5)——没有任何区域概念，不启动过洞，直接发
  CHECK(!ex.tunnel.active());
  CHECK(p.goal_count == 2);
  CHECK(p.goal_x == 15.0);
  CHECK(p.fold_cmd_count == 0);

  // 血量锁存回家照常工作（家也在"没有区域"的点表里）
  set_match(p, 4, 80);      // 20% < 30%
  tick(p, c, plan, ex);
  CHECK(plan.home_latched);
  CHECK(plan.cur_point == 1);
}

// ---------------- ⑦ 过洞中比赛结束：中止且保持折叠 ----------------
static void test_abort_during_tunnel()
{
  Config c = corridor_cfg();
  DebugPorts p;
  p.pose = {5.0, 5.0, 0.0, 0, true};
  p.fold = kFoldUnfolded;
  PlanState plan;
  plan.current_region = 0;
  ExecState ex;

  set_match(p, 4, 400);
  tick(p, c, plan, ex);  // 目标 id10(5,5)，已站在点上
  arrive(p, 5.0, 5.0);
  tick(p, c, plan, ex);  // 换 id11 → 启动过洞 → 入口目标
  CHECK(ex.tunnel.active());
  arrive(p, 9.0, 5.0);
  tick(p, c, plan, ex);
  tick(p, c, plan, ex);  // 折叠
  CHECK(p.fold == kFoldFolded);

  // 比赛结束（下降沿）→ 一次性收尾：取消导航 + 停自旋 + 终止过洞（折叠保持，
  // 还在洞口不能乱展开；不给电控发停车），之后静默；不写区域记账（中止 ≠ 已到出口）
  const int region_before = plan.current_region;
  const int cancels_before_end = p.cancel_count;
  set_match(p, 5, 400);
  tick(p, c, plan, ex);
  CHECK(!ex.tunnel.active());
  CHECK(p.fold == kFoldFolded);
  CHECK(p.cancel_count == cancels_before_end + 1); // 结束沿只新增一次取消
  CHECK(p.stop_count == 0);         // 比赛外停车不发消息（取消导航后底盘自己停）
  CHECK(p.spin_last_w == 0.0);      // 我们开的自旋我们关掉
  CHECK(plan.current_region == region_before);
}

// ---------------- ⑧ 未折叠已在洞口：拒绝裸穿（region 版） ----------------
static void test_refuse_unfolded_in_mouth()
{
  Config c = corridor_cfg();
  DebugPorts p;
  p.pose = {10.4, 5.0, 0.0, 0, true};  // 站在洞轴中段（洞口/洞内）
  p.fold = kFoldUnfolded;
  PlanState plan;
  plan.current_region = 0;  // 记账 left；离车最近的巡逻点是 right 侧的 id11
  ExecState ex;

  set_match(p, 4, 400);
  tick(p, c, plan, ex);  // 目标 id11(right) 跨区域，但车在洞口且未折叠
  CHECK(!ex.tunnel.active());
  CHECK(p.goal_count == 0);  // 一个导航目标都不发：沿洞轴任何方向都会裸穿撞云台
  CHECK(p.stop_count >= 1);
  tick(p, c, plan, ex);  // 冷却期内依旧拒绝
  CHECK(p.goal_count == 0);

  // 已折叠才允许"前进到出口"（车在 left → 出口 = right 侧对正点）
  p.fold = kFoldFolded;
  p.time_s += c.tunnel_retry_cooldown_s + 0.1;  // 越过冷却
  set_match(p, 4, 400);
  tick(p, c, plan, ex);
  CHECK(ex.tunnel.active());
  CHECK(p.goal_x == 11.0);  // 直奔出口对正点
}

// ---------------- ⑨ 车区域与洞两端都对不上：拒绝，绝不猜入口 ----------------
static void test_region_mismatch_refused()
{
  Config c = corridor_cfg();
  c.regions.push_back("balcony");  // 加一个区域，没有任何洞连它
  DebugPorts p;
  p.pose = {3.0, 5.0, 0.0, 0, true};
  p.fold = kFoldUnfolded;
  set_match(p, 4, 400);
  p.time_s = 1.0;

  // 直接调 start：from_region = balcony，洞两端是 left/right → 拒绝
  TunnelRunner tr;
  const Vec2 pos{3.0, 5.0};
  CHECK(!tr.start(p, c, 0, 2, &pos, c.points[1], p.time_s));
  CHECK(!tr.active());
  CHECK(p.goal_count == 0);  // 拒绝路径不发目标，只停车
  CHECK(p.stop_count >= 1);

  // 集成层：记账在 balcony（孤岛，比如配置错了 start_region）→ 没有任何可达点
  // → 待机停车（纯记账模式下没有任何输入会纠正它，停下来等人查配置）
  PlanState plan;
  plan.current_region = 2;
  ExecState ex;
  ex.match_on = true;  // 模拟比赛中段（跳过开赛沿的区域重置）
  tick(p, c, plan, ex);
  CHECK(plan.current_region == 2);   // 记账保持（无外部纠正）
  CHECK(p.goal_count == 0);          // 孤岛区域选不出点，待机
  CHECK(p.stop_count >= 1);
}

// ---------------- ⑩ 地图对齐 ----------------
static void test_map_alignment()
{
  // 数值：绕原点转 90° 再平移 (1,2)：(x,y) → (-y+1, x+2)
  const FrameTransform t{true, 1.0, 2.0, 90.0};
  const Vec2 v = apply_frame_transform({1.0, 5.0}, t);
  CHECK(std::fabs(v.x - (-4.0)) < 1e-9);
  CHECK(std::fabs(v.y - 3.0) < 1e-9);
  // 关闭时恒等
  const FrameTransform off{false, 9.0, 9.0, 33.0};
  const Vec2 w = apply_frame_transform({1.0, 5.0}, off);
  CHECK(w.x == 1.0 && w.y == 5.0);

  // 加载时生效：点和对正点都被搬到建图系
  const std::string path = "/tmp/bbt_test_align.yaml";
  {
    std::ofstream f(path);
    f << "regions: [left, right]\nstart_region: left\n"
      << "field_to_map:\n  enabled: true\n  dx: 1.0\n  dy: 2.0\n  dyaw_deg: 90.0\n"
      << "points:\n  - {id: 1, x: 1.0, y: 5.0, zone: home, region: left}\n"
      << "tunnels:\n  - name: h\n    center: [0.0, 0.0]\n"
      << "    align:\n    - {p: [1.0, 5.0], region: left}\n    - {p: [2.0, 5.0], region: right}\n";
  }
  Config c;
  std::string err;
  std::vector<std::string> warns;
  CHECK(load_config(path, c, err, warns));
  CHECK(err.empty());
  CHECK(std::fabs(c.points[0].pos.x - (-4.0)) < 1e-9);
  CHECK(std::fabs(c.points[0].pos.y - 3.0) < 1e-9);
  CHECK(std::fabs(c.tunnels[0].align[1].x - (-4.0)) < 1e-9);  // (2,5) → (-4,4)
  CHECK(std::fabs(c.tunnels[0].align[1].y - 4.0) < 1e-9);
}

// ---------------- ⑪ 配置加载与校验 ----------------
static void test_config_load()
{
  const std::string path = "/tmp/bbt_test_cfg.yaml";
  {
    std::ofstream f(path);
    f << "regions: [left, right]\nstart_region: left\n"
      << "safety:\n  patrol_dwell_s: 12\n  odom_stale_ms: 800\n"
      << "points:\n"
      << "  - {id: 1, x: 1.0, y: 5.0, zone: home, region: left}\n"
      << "  - {id: 10, x: 5.0, y: 5.0, zone: patrol, region: right}\n"
      << "tunnels:\n"
      << "  - name: hole\n    center: [10.0, 5.0]\n"
      << "    align:\n    - {p: [9.0, 5.0], region: left}\n    - {p: [11.0, 5.0], region: right}\n";
  }
  Config c;
  std::string err;
  std::vector<std::string> warns;
  CHECK(load_config(path, c, err, warns));
  CHECK(err.empty());
  CHECK(c.regions.size() == 2);
  CHECK(c.start_region == 0);
  CHECK(std::fabs(c.odom_stale_ms - 800.0) < 1e-9);
  CHECK(c.points.size() == 2);
  CHECK(c.tunnels.size() == 1);
  CHECK(std::fabs(c.patrol_dwell_s - 12.0) < 1e-9);
  CHECK(c.points[0].zone == kZoneHome);
  CHECK(c.points[0].region == 0);
  CHECK(c.points[1].region == 1);
  CHECK(c.tunnels[0].align_region[0] == 0 && c.tunnels[0].align_region[1] == 1);
  CHECK(std::fabs(c.tunnels[0].align[1].x - 11.0) < 1e-9);

  // 点的 region 不在表里 → 报错
  {
    std::ofstream f(path);
    f << "regions: [left, right]\npoints:\n  - {id: 1, x: 0, y: 0, zone: home, region: fish}\n";
  }
  CHECK(!load_config(path, c, err, warns));
  CHECK(!err.empty());

  // regions 为空 → 报错
  {
    std::ofstream f(path);
    f << "points:\n  - {id: 1, x: 0, y: 0, zone: home, region: a}\n";
  }
  CHECK(!load_config(path, c, err, warns));

  // 洞对正点缺 region → 报错
  {
    std::ofstream f(path);
    f << "regions: [a, b]\npoints:\n  - {id: 1, x: 0, y: 0, zone: home, region: a}\n"
      << "tunnels:\n  - name: h\n    align:\n    - {p: [0, 0], region: a}\n    - {p: [1, 1], region: zz}\n";
  }
  CHECK(!load_config(path, c, err, warns));

  // 孤岛区域 → 告警（不报错）
  {
    std::ofstream f(path);
    f << "regions: [a, b, lonely]\nstart_region: a\n"
      << "points:\n  - {id: 1, x: 0, y: 0, zone: home, region: a}\n"
      << "tunnels:\n  - name: h\n    align:\n    - {p: [0, 0], region: a}\n    - {p: [1, 1], region: b}\n";
  }
  warns.clear();
  CHECK(load_config(path, c, err, warns));
  bool has_island_warn = false;
  for (const auto& w : warns)
  {
    if (w.find("lonely") != std::string::npos) has_island_warn = true;
  }
  CHECK(has_island_warn);

  // 过洞开关关闭：regions 可省、点可不标 region、洞表忽略（告警）——全都能加载
  {
    std::ofstream f(path);
    f << "enable_tunnel: false\n"
      << "safety:\n  spin_w: 2.0\n  ammo_home_min: 40\n  ammo_resume_min: 120\n"
      << "points:\n  - {id: 1, x: 0, y: 0, zone: home}\n"
      << "  - {id: 10, x: 3, y: 0, zone: patrol}\n"
      << "tunnels:\n  - name: h\n    align:\n    - {p: [0, 0], region: a}\n    - {p: [1, 1], region: b}\n";
  }
  warns.clear();
  CHECK(load_config(path, c, err, warns));
  CHECK(err.empty());
  CHECK(!c.enable_tunnel);
  CHECK(c.regions.empty());          // region 整套没配也不报错
  CHECK(c.points.size() == 2);
  CHECK(c.points[0].region == -1);   // 点不标 region 合法
  CHECK(c.tunnels.empty());          // 洞表被忽略
  CHECK(std::fabs(c.spin_w - 2.0) < 1e-9);
  CHECK(c.ammo_home_min == 40 && c.ammo_resume_min == 120);
  bool has_ignore_warn = false;
  for (const auto& w : warns)
  {
    if (w.find("洞表已忽略") != std::string::npos) has_ignore_warn = true;
  }
  CHECK(has_ignore_warn);

  // 开关关着但硬要写 region：不认识也不报错（region 整套不存在）
  {
    std::ofstream f(path);
    f << "enable_tunnel: false\n"
      << "points:\n  - {id: 1, x: 0, y: 0, zone: home, region: whatever}\n";
  }
  CHECK(load_config(path, c, err, warns));
  CHECK(c.points[0].region == -1);

  // 自动购买参数解析（缺省值见 types.hpp）
  {
    std::ofstream f(path);
    f << "enable_tunnel: false\n"
      << "auto_buy:\n"
      << "  enable: false\n"
      << "  ammo_min_shots: 25\n"
      << "  ammo_rounds: 150\n"
      << "  hp_ratio: 0.35\n"
      << "  revive_min_coins: 250\n"
      << "  cooldown_s: 15\n"
      << "points:\n  - {id: 1, x: 0, y: 0, zone: home}\n";
  }
  CHECK(load_config(path, c, err, warns));
  CHECK(!c.enable_auto_buy);
  CHECK(c.ammo_buy_min_shots == 25);
  CHECK(c.ammo_buy_rounds == 150);
  CHECK(std::fabs(c.hp_buy_ratio - 0.35) < 1e-9);
  CHECK(c.revive_buy_min_coins == 250);
  CHECK(std::fabs(c.buy_cooldown_s - 15.0) < 1e-9);
  CHECK(c.ammo_buy_min_coins == 200);   // 没写的用默认
  CHECK(c.hp_buy_amount == 1);
}

// ---------------- ⑫ 几何基础 ----------------
static void test_geometry()
{
  const Vec2 a{0.0, 0.0}, b{4.0, 0.0};
  CHECK(std::fabs(project_param({2.0, 0.0}, a, b) - 0.5) < 1e-9);
  CHECK(project_param({-1.0, 0.0}, a, b) < 0.0);  // A 外侧
  CHECK(project_param({5.0, 0.0}, a, b) > 1.0);   // B 外侧
  CHECK(std::fabs(dist_to_segment({2.0, 3.0}, a, b) - 3.0) < 1e-9);
}

// ---------------- 前哨争夺：开局打前哨 / 我方失守抢回 / 双占领巡逻 ----------------
// 场景：单区域（不开过洞），点表配 home + patrol + outpost(21) + retake(20)。
// 用例覆盖使用者口径：
//   a. 开局（对面前哨阵地未控制、我方安好）→ 去 outpost 攻击位，attack_cmd 点名对面前哨；
//   b. 我方前哨被打掉（hp=0，控制状态随便）→ 优先去 retake（先保家再打人）；
//   c. 我方完全控制（hp>0 且 ctrl=0）+ 对面完全控制（ctrl=0）→ 巡逻；
//   d. 对面前哨阵地被对面抢回（ctrl=2）→ 再去 outpost（来回抢）；
//   e. 数据未知（-1）→ 不参与判定，巡逻；
//   f. enable_outpost=false → 整套不存在；
//   g. retake 点attack_cmd 自由开火（free_fire=1）。
static void test_outpost_contest()
{
  Config c;
  c.regions = {"mid"};
  c.start_region = 0;
  c.enable_tunnel = false;  // 单区域：纯测前哨段，不掺过洞
  c.switch_confirm_frames = 1;
  c.switch_dwell_s = 0.0;
  c.patrol_dwell_s = 1.0;
  c.points = {{1, {1.0, 5.0}, kZoneHome, 0},
              {20, {8.0, 5.0}, kZoneRetake, 0},
              {21, {12.0, 5.0}, kZoneOutpost, 0},
              {10, {4.0, 5.0}, kZonePatrol, 0}};
  DebugPorts p;
  p.pose = {2.0, 5.0, 0.0, 0, true};
  p.fold = kFoldUnfolded;
  PlanState plan;
  plan.current_region = 0;
  ExecState ex;

  // a. 开局：我方前哨安好（500 血/完全控制），对面前哨阵地在对面手里 → 打前哨
  set_match(p, 4, 400);
  p.ref.our_outpost_hp = 500;
  p.ref.our_outpost_ctrl = kCtrlFull;
  p.ref.enemy_outpost_hp = 1000;
  p.ref.enemy_outpost_ctrl = kCtrlNone;
  tick(p, c, plan, ex);
  tick(p, c, plan, ex);  // 第一 tick 建目标，第二 tick 生效（switch_confirm=1）
  CHECK(plan.cur_point == 21);
  CHECK(p.attack_calls > 0 && p.attack_last_type == 2 && p.attack_last_id == 1);

  // c. 对面阵地被我方完全控制（打掉+队友占完）→ 巡逻
  p.ref.enemy_outpost_hp = 0;
  p.ref.enemy_outpost_ctrl = kCtrlFull;
  tick(p, c, plan, ex);
  tick(p, c, plan, ex);
  CHECK(plan.cur_point == 10);
  CHECK(p.attack_last_type == 0 && p.attack_last_fire == 1);  // 巡逻段自由开火

  // d. 对面把阵地抢回（ctrl 回到未控制）→ 再去 outpost（来回抢）
  p.ref.enemy_outpost_ctrl = kCtrlNone;
  tick(p, c, plan, ex);
  tick(p, c, plan, ex);
  CHECK(plan.cur_point == 21);

  // b. 我方前哨被打掉（不管控制状态）→ 优先 retake
  p.ref.our_outpost_hp = 0;
  p.ref.our_outpost_ctrl = kCtrlFull;  // 就算阵地还标着完全控制，站没了也要去重建
  tick(p, c, plan, ex);
  tick(p, c, plan, ex);
  CHECK(plan.cur_point == 20);
  CHECK(p.attack_last_type == 0 && p.attack_last_fire == 1);  // retake 自由开火

  // 我方阵地只是临时控制（站还在）→ 同样要去 retake 站满转完全
  p.ref.our_outpost_hp = 500;
  p.ref.our_outpost_ctrl = kCtrlTemp;
  tick(p, c, plan, ex);
  tick(p, c, plan, ex);
  CHECK(plan.cur_point == 20);

  // 我方完全恢复 + 对面保持未控制 → 回到打对面（先保家再打人）
  p.ref.our_outpost_ctrl = kCtrlFull;
  tick(p, c, plan, ex);
  tick(p, c, plan, ex);
  CHECK(plan.cur_point == 21);

  // e. 前哨数据未知（-1，桥断流/没收到）→ 全部落到巡逻，绝不乱跑
  p.ref.our_outpost_hp = -1;
  p.ref.our_outpost_ctrl = kCtrlUnknown;
  p.ref.enemy_outpost_hp = -1;
  p.ref.enemy_outpost_ctrl = kCtrlUnknown;
  tick(p, c, plan, ex);
  tick(p, c, plan, ex);
  CHECK(plan.cur_point == 10);

  // f. enable_outpost=false：对面阵地明明失守也不去打（整套不存在）
  c.enable_outpost = false;
  p.ref.our_outpost_hp = 500;
  p.ref.our_outpost_ctrl = kCtrlFull;
  p.ref.enemy_outpost_hp = 1000;
  p.ref.enemy_outpost_ctrl = kCtrlNone;
  tick(p, c, plan, ex);
  tick(p, c, plan, ex);
  CHECK(plan.cur_point == 10);
}

// ---------------- 前哨争夺优先级：回家 > 前哨 > 巡逻 ----------------
static void test_outpost_priority()
{
  Config c;
  c.regions = {"mid"};
  c.start_region = 0;
  c.enable_tunnel = false;
  c.switch_confirm_frames = 1;
  c.switch_dwell_s = 0.0;
  c.points = {{1, {1.0, 5.0}, kZoneHome, 0},
              {20, {8.0, 5.0}, kZoneRetake, 0},
              {21, {12.0, 5.0}, kZoneOutpost, 0}};
  DebugPorts p;
  p.pose = {2.0, 5.0, 0.0, 0, true};
  PlanState plan;
  ExecState ex;

  // 双方前哨都要处理 + 血量崩了 → 回家优先（残血打不了争夺）
  set_match(p, 4, 80);  // 80/400 = 20% < 30%
  p.ref.our_outpost_hp = 0;
  p.ref.our_outpost_ctrl = kCtrlNone;
  p.ref.enemy_outpost_ctrl = kCtrlNone;
  tick(p, c, plan, ex);
  tick(p, c, plan, ex);
  CHECK(plan.cur_point == 1);
}

int main()
{
  test_geometry();
  test_config_load();
  test_map_alignment();
  test_corridor_regions();
  test_renamed_map_chained();
  test_region_bookkeeping();
  test_odom_stale();
  test_isolated_region();
  test_home_latch_across_tunnel();
  test_gating();
  test_ammo_latch();
  test_operator_cmds();
  test_auto_buy();
  test_spin_states();
  test_tunnel_disabled();
  test_abort_during_tunnel();
  test_refuse_unfolded_in_mouth();
  test_region_mismatch_refused();
  test_outpost_contest();
  test_outpost_priority();
  if (g_failures == 0)
  {
    std::printf("planner_test: 全部通过\n");
    return 0;
  }
  std::printf("planner_test: %d 处失败\n", g_failures);
  return 1;
}
