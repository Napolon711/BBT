// BBT 行为树测试：用 trees/main.xml 驱动同一套走廊流程。
// 目的：证明"跑树"和"直接调 step()/纯函数"（planner_test）行为一致——
// BT 节点只是编排，逻辑全在 planner.cpp，两边不会分叉。
#include <cstdio>
#include <memory>

#include "bbt_decide/bt.hpp"

using namespace bbt_decide;

static int g_failures = 0;
#define CHECK(cond)                                                    \
  do                                                                   \
  {                                                                    \
    if (!(cond))                                                       \
    {                                                                  \
      ++g_failures;                                                    \
      std::printf("FAIL %s:%d: %s\n", __FILE__, __LINE__, #cond);      \
    }                                                                  \
  } while (0)

// 测试脚手架：DebugPorts + 一棵从 trees/main.xml 载入的树。
struct TreeBench
{
  std::shared_ptr<DebugPorts> ports;
  std::shared_ptr<Context> ctx;
  BT::Tree tree;

  explicit TreeBench(const Config& cfg)
  {
    ports = std::make_shared<DebugPorts>();
    ctx = std::make_shared<Context>();
    ctx->cfg = cfg;
    ctx->ports = ports;
    ctx->plan.current_region = cfg.start_region;
    BT::BehaviorTreeFactory factory;
    register_nodes(factory, ctx);
    factory.registerBehaviorTreeFromFile(SRC_DIR "/trees/main.xml");
    tree = factory.createTree("Main");
  }

  void set_match(int progress, int hp)
  {
    ports->ref.game_progress = progress;
    ports->ref.hp = hp;
    ports->ref.hp_max = 400;
  }
  void arrive(double x, double y)
  {
    ports->pose.x = x;
    ports->pose.y = y;
    ports->pose.valid = true;
    ports->goal = kGoalSuccess;
  }
  // 推一拍树（时间 + 位姿时间戳同步推进，模拟 odom 持续在发）。
  void tick(double dt = 0.05)
  {
    ports->time_s += dt;
    ports->pose.stamp_ms = static_cast<uint64_t>(ports->time_s * 1000.0);
    tree.tickOnce();
  }
  int acting() const
  {
    return ctx->ex.tunnel.active() ? ctx->ex.tunnel.after_point : ctx->decision.target_point;
  }
};

// 走廊配置（与 planner_test 的 corridor_cfg 同一份，区域 left/right + 中间一个洞）。
static Config corridor_cfg()
{
  Config c;
  c.regions = {"left", "right"};
  c.start_region = 0;
  c.switch_confirm_frames = 1;
  c.switch_dwell_s = 0.0;
  c.patrol_dwell_s = 1.0;
  c.points = {{1, {1.0, 5.0}, kZoneHome, 0},
              {10, {5.0, 5.0}, kZonePatrol, 0},
              {11, {15.0, 5.0}, kZonePatrol, 1},
              {12, {19.0, 5.0}, kZonePatrol, 1}};
  TunnelDef t;
  t.name = "corridor_hole";
  t.align[0] = {9.0, 5.0};
  t.align_region[0] = 0;
  t.align[1] = {11.0, 5.0};
  t.align_region[1] = 1;
  c.tunnels = {t};
  return c;
}

// ① 比赛未开始 → Standby 接管：完全静默，一条消息都不发（没有可终止的东西）。
static void test_standby_gate()
{
  TreeBench b(corridor_cfg());
  b.ports->pose = {2.0, 5.0, 0.0, 0, true};
  b.ports->fold = kFoldUnfolded;
  b.set_match(0, 400);  // 未开始
  b.tick();
  b.tick();
  CHECK(b.ports->goal_count == 0);
  CHECK(b.ports->stop_count == 0);   // 不给电控发停车（取消导航后底盘自己停）
  CHECK(b.ports->cancel_count == 0);
  CHECK(b.ports->spin_count == 0);
  CHECK(b.ports->mode_count == 0);
  CHECK(!b.ctx->ex.tunnel.active());
}

// ② 走廊全流程：巡逻 → 跨区域 → 折叠过洞 → 记账区域翻转 → 继续巡逻。
static void test_corridor_flow()
{
  TreeBench b(corridor_cfg());
  b.ports->pose = {2.0, 5.0, 0.0, 0, true};
  b.ports->fold = kFoldUnfolded;
  b.set_match(4, 400);

  b.tick();  // 最近巡逻点 id10（left 侧），同区直达
  CHECK(b.acting() == 10);
  CHECK(b.ports->goal_count == 1);
  CHECK(b.ports->goal_x == 5.0 && b.ports->goal_y == 5.0);

  b.arrive(5.0, 5.0);
  b.tick();  // 换 id11（right 侧）→ 跨区域 → 过洞：入口 = left 侧对正点 (9,5)
  CHECK(b.ctx->ex.tunnel.active());
  CHECK(b.ports->goal_count == 2);
  CHECK(b.ports->goal_x == 9.0 && b.ports->goal_y == 5.0);
  CHECK(b.ports->fold_cmd_count == 0);
  CHECK(b.acting() == 11);  // 过洞中：目标 = 过完要去的点

  b.arrive(9.0, 5.0);
  b.tick();  // 到入口 → 折叠
  b.tick();
  CHECK(b.ports->fold_cmd_count == 1);
  CHECK(b.ports->fold == kFoldFolded);

  b.tick();  // 折叠到位 → 发出口目标 (11,5)
  b.tick();
  CHECK(b.ports->goal_count == 3);
  CHECK(b.ports->goal_x == 11.0 && b.ports->goal_y == 5.0);

  b.arrive(11.0, 5.0);
  b.tick();  // 到出口 → 停车 + 展开
  b.tick();
  CHECK(b.ports->unfold_cmd_count == 1);
  b.tick();  // 展开完成 → 记账区域 = right，交回常规导航
  CHECK(!b.ctx->ex.tunnel.active());
  CHECK(b.ctx->plan.current_region == 1);

  b.tick();  // 继续去 id11 (15,5)（现在同区）
  CHECK(b.ports->goal_count == 4);
  CHECK(b.ports->goal_x == 15.0 && b.ports->goal_y == 5.0);
  CHECK(!b.ctx->ex.tunnel.active());
}

// ③ 比赛进行中过洞到一半比赛结束 → 下降沿一次性收尾：取消导航 + 停自旋 +
// 终止过洞流程（halt 只清状态），折叠保持；之后静默，不给电控发停车。
static void test_match_end_mid_tunnel()
{
  TreeBench b(corridor_cfg());
  b.ports->pose = {5.0, 5.0, 0.0, 0, true};
  b.ports->fold = kFoldUnfolded;
  b.set_match(4, 400);

  b.tick();  // 目标 id10（已站在点上）
  b.arrive(5.0, 5.0);
  b.tick();  // 换 id11 → 过洞启动
  CHECK(b.ctx->ex.tunnel.active());
  b.arrive(9.0, 5.0);
  b.tick();
  b.tick();  // 折叠
  CHECK(b.ports->fold == kFoldFolded);

  const int cancels_before_end = b.ports->cancel_count;
  b.set_match(5, 400);  // 比赛结束 → IsMatchRunning 失败 → Standby，TunnelCross 被 halt
  b.tick();
  CHECK(!b.ctx->ex.tunnel.active());
  CHECK(b.ports->fold == kFoldFolded);  // 收尾不动折叠（还在洞口）
  CHECK(b.ports->cancel_count == cancels_before_end + 1);    // 恰好一次取消
  CHECK(b.ports->stop_count == 0);      // 比赛外不发停车
  CHECK(b.ports->spin_last_w == 0.0);   // 我们开的自旋我们关掉
  const int goal_at_end = b.ports->goal_count;
  const int spin_at_end = b.ports->spin_count;
  b.tick();  // 之后完全静默
  b.tick();
  CHECK(b.ports->goal_count == goal_at_end);
  CHECK(b.ports->spin_count == spin_at_end);
  CHECK(b.ports->cancel_count == cancels_before_end + 1);
}

// ④ 操作手指令：买类忽略（那是操作手和裁判客户端的事）；指点立即接管
//    （同点重指 = 取消重发）。自动购买由树自己下单（门控节点里跑）。
static void test_operator_via_tree()
{
  TreeBench b(corridor_cfg());
  b.ports->pose = {2.0, 5.0, 0.0, 0, true};
  b.ports->fold = kFoldUnfolded;

  b.set_match(0, 400);        // 比赛外：操作手买弹也不转发（金币未知也不会自动买）
  b.ports->op = {1, 1, 100};  // 操作手买弹 100 发
  b.tick();
  CHECK(b.ports->buy_ammo_calls == 0);
  CHECK(b.ports->goal_count == 0);  // 运动消息一条没有

  b.set_match(4, 400);        // 比赛开始：模式切一次，巡逻 id10
  b.tick();
  CHECK(b.ports->mode_count == 1);
  CHECK(b.acting() == 10);

  b.ports->op = {2, 4, 10};   // 指点 id10（与当前目标同一点）→ 目标更新：取消重发
  b.tick();
  CHECK(b.ctx->plan.op_goal == 10);
  CHECK(b.ports->goal_count == 2);
  CHECK(b.ports->cancel_count == 1);
  CHECK(b.ports->spin_last_w == b.ctx->cfg.spin_w);  // 非过洞段受击旋转恒发

  b.arrive(5.0, 5.0);         // 到点 → 指点消费掉，恢复巡逻
  b.tick();
  CHECK(b.ctx->plan.op_goal == -1);

  // 自动买弹：弹量 10 < 30、金币 1000 ≥ 200 → 树自己下单一次（冷却挡重复）
  b.ports->ref.shots = 10;
  b.ports->ref.coins = 1000;
  b.tick();
  CHECK(b.ports->buy_ammo_calls == 1);
  CHECK(b.ports->buy_ammo_param == 100);
  b.tick();
  CHECK(b.ports->buy_ammo_calls == 1);  // 冷却内不重复
}

// Production tree: same-point rescue, unusable click and idle stop must behave correctly.
static void test_operator_at_goal_and_idle()
{
  auto cfg = corridor_cfg(); cfg.enable_tunnel=false;
  TreeBench b(cfg);b.set_match(4,400);b.ports->pose={5,5,0,0,true};
  b.ports->op={1,4,10};b.tick();
  CHECK(b.ports->goal_count==1 && b.ctx->plan.op_goal==10);
  b.ports->op={2,4,10};b.tick();
  CHECK(b.ports->goal_count==2 && b.ctx->plan.op_goal==10);
  b.tick();CHECK(b.ctx->plan.op_goal==-1);
  b.ctx->cfg.points.clear();b.ctx->plan=PlanState{};
  const int cancels=b.ports->cancel_count;
  b.tick();const int stopped=b.ports->stop_count;
  for(int i=0;i<50;++i)b.tick();
  CHECK(b.ports->cancel_count==cancels+1 && b.ports->stop_count==stopped);
}

int main()
{
  test_operator_at_goal_and_idle();
  test_standby_gate();
  test_corridor_flow();
  test_match_end_mid_tunnel();
  test_operator_via_tree();
  if (g_failures == 0)
  {
    std::printf("bt_test: 全部通过\n");
    return 0;
  }
  std::printf("bt_test: %d 处失败\n", g_failures);
  return 1;
}
