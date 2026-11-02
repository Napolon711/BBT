// Native blackboard contract tests exercise XML remapping at public leaf boundaries.
#include <cstdio>
#include <functional>
#include <memory>
#include <string>

#include "bbt_decide/bt.hpp"

using namespace bbt_decide;

namespace
{
int failures = 0;

#define CHECK(condition)                                                        \
  do                                                                           \
  {                                                                            \
    if (!(condition))                                                          \
    {                                                                          \
      ++failures;                                                              \
      std::printf("FAIL %s:%d: %s\n", __FILE__, __LINE__, #condition);            \
    }                                                                          \
  } while (0)

void test(const char* name, const std::function<void()>& body)
{
  const int previous = failures;
  try { body(); }
  catch (const std::exception& error)
  {
    ++failures;
    std::printf("FAIL %s: unexpected exception: %s\n", name, error.what());
  }
  if (previous == failures) std::printf("PASS %s\n", name);
}

template <typename Action>
void expect_error(Action action)
{
  bool threw = false;
  try { action(); }
  catch (const std::exception&) { threw = true; }
  CHECK(threw);
}

std::shared_ptr<const Config> config()
{
  auto cfg = std::make_shared<Config>();
  cfg->enable_tunnel = false;
  return cfg;
}

std::shared_ptr<DebugPorts> hardware(int hp, int match_progress = 4)
{
  auto io = std::make_shared<DebugPorts>();
  io->time_s = 12.0;
  io->pose = {1.0, 2.0, 0.0, 12000, true};
  io->ref.game_progress = match_progress;
  io->ref.hp = hp;
  io->ref.hp_max = 400;
  io->ref.shots = 200;
  return io;
}

BT::Tree leaf(BT::BehaviorTreeFactory& factory, const BT::Blackboard::Ptr& board,
              const std::string& node)
{
  return factory.createTreeFromText(
      "<root BTCPP_format=\"4\" main_tree_to_execute=\"Contract\">"
      "<BehaviorTree ID=\"Contract\">" + node + "</BehaviorTree></root>", board);
}

void initialized_board_contract()
{
  const auto cfg = config();
  const std::shared_ptr<Ports> io = hardware(350);
  PlanState plan;
  plan.op_generation = 71;
  ExecState execution;
  execution.retry_until_s = 99.0;
  const auto board = make_blackboard(cfg, io, plan, execution);
  CHECK(board->get<std::shared_ptr<const Config>>("config") == cfg);
  CHECK(board->get<std::shared_ptr<Ports>>("io") == io);
  CHECK(board->get<PlanState>("plan").op_generation == 71);
  CHECK(board->get<ExecState>("execution").retry_until_s == 99.0);
  CHECK(!board->get<Sense>("sense").match_running);
  CHECK(!board->get<Decision>("target").target_valid);
}

void condition_reads_remapped_sense()
{
  BT::BehaviorTreeFactory factory;
  register_nodes(factory);
  const auto board = make_blackboard(config(), hardware(350));
  Sense running;
  running.match_running = true;
  board->set("selected_sense", running);
  auto tree = leaf(factory, board, "<MatchRunning name=\"CheckSelectedMatch\" sense=\"{selected_sense}\"/>");
  CHECK(tree.tickOnce() == BT::NodeStatus::SUCCESS);
  CHECK(!board->get<Sense>("sense").match_running);

  running.match_running = false;
  board->set("selected_sense", running);
  CHECK(tree.tickOnce() == BT::NodeStatus::FAILURE);
}

void read_inputs_remaps_input_output_and_memory()
{
  BT::BehaviorTreeFactory factory;
  register_nodes(factory);
  const auto default_io = hardware(101, 0);
  const auto selected_io = hardware(321);
  selected_io->op = {9, 4, -1, 6.25, 4.75};
  const auto board = make_blackboard(config(), default_io);
  board->set("selected_config", config());
  board->set<std::shared_ptr<Ports>>("selected_io", selected_io);
  board->set("selected_plan", PlanState{});
  auto tree = leaf(factory, board,
      "<ReadInputs config=\"{selected_config}\" io=\"{selected_io}\" "
      "sense=\"{selected_sense}\" plan=\"{selected_plan}\"/>");
  CHECK(tree.tickOnce() == BT::NodeStatus::SUCCESS);

  const auto sensed = board->get<Sense>("selected_sense");
  const auto remembered = board->get<PlanState>("selected_plan");
  CHECK(sensed.match_running);
  CHECK(sensed.hp == 321);
  CHECK(remembered.op_active);
  CHECK(remembered.op_seq_seen == 9);
  CHECK(remembered.op_target.pos.x == 6.25);
  CHECK(remembered.op_target.pos.y == 4.75);
  CHECK(!board->get<Sense>("sense").match_running);
  CHECK(!board->get<PlanState>("plan").op_active);
}

void selector_writes_remapped_target()
{
  BT::BehaviorTreeFactory factory;
  register_nodes(factory);
  const auto board = make_blackboard(config(), hardware(350));
  PlanState selected;
  selected.op_active = true;
  selected.op_generation = 14;
  selected.op_target.id = -3;
  selected.op_target.pos = {7.125, 8.375};
  board->set("selected_plan", selected);
  board->set<std::shared_ptr<Ports>>("selected_io", hardware(350));
  auto tree = leaf(factory, board,
      "<SelectOperator io=\"{selected_io}\" plan=\"{selected_plan}\" target=\"{selected_target}\"/>");
  CHECK(tree.tickOnce() == BT::NodeStatus::SUCCESS);
  const auto target = board->get<Decision>("selected_target");
  CHECK(target.target_valid);
  CHECK(target.operator_target);
  CHECK(target.generation == 14);
  CHECK(target.target.pos.x == 7.125);
  CHECK(target.target.pos.y == 8.375);
  CHECK(target.stamp_ms == 12000);
  CHECK(!board->get<Decision>("target").target_valid);
}

void commit_target_writes_remapped_memory_and_target()
{
  BT::BehaviorTreeFactory factory;
  register_nodes(factory);
  const auto board = make_blackboard(config(), hardware(350));
  PlanState selected;
  selected.cur_target_valid = true;
  selected.cur_target.id = 28;
  selected.cur_target.pos = {14.73, 3.61};
  selected.cur_point = 28;
  selected.cur_since_s = 11.5;
  board->set("selected_plan", selected);
  Decision candidate;
  candidate.target_point = 40;
  candidate.target.id = 40;
  candidate.target.pos = {15.4, 4.45};
  candidate.target_valid = true;
  board->set("selected_target", candidate);
  auto tree = leaf(factory, board,
      "<CommitTarget config=\"{config}\" io=\"{io}\" "
      "plan=\"{selected_plan}\" target=\"{selected_target}\"/>");
  CHECK(tree.tickOnce() == BT::NodeStatus::SUCCESS);
  // A new autonomous candidate is debounced: memory records the candidate,
  // while the output remains the already committed navigation target.
  CHECK(board->get<PlanState>("selected_plan").pending_point == 40);
  CHECK(board->get<Decision>("selected_target").target_point == 28);
  CHECK(board->get<Decision>("selected_target").target.pos.x == 14.73);
  CHECK(!board->get<PlanState>("plan").cur_target_valid);
  CHECK(!board->get<Decision>("target").target_valid);
}

void registration_is_reusable_and_tree_boards_are_isolated()
{
  BT::BehaviorTreeFactory factory;
  register_nodes(factory);
  const auto running_io = hardware(350);
  const auto stopped_io = hardware(100, 0);
  const auto running_board = make_blackboard(config(), running_io);
  const auto stopped_board = make_blackboard(config(), stopped_io);
  const std::string xml =
      "<Sequence><ReadInputs config=\"{config}\" io=\"{io}\" "
      "sense=\"{sense}\" plan=\"{plan}\"/>"
      "<MatchRunning sense=\"{sense}\"/></Sequence>";
  auto running_tree = leaf(factory, running_board, xml);
  auto stopped_tree = leaf(factory, stopped_board, xml);
  CHECK(running_tree.tickOnce() == BT::NodeStatus::SUCCESS);
  CHECK(stopped_tree.tickOnce() == BT::NodeStatus::FAILURE);
  CHECK(running_board->get<Sense>("sense").hp == 350);
  CHECK(stopped_board->get<Sense>("sense").hp == 100);

  running_io->ref.game_progress = 0;
  stopped_io->ref.game_progress = 4;
  CHECK(stopped_tree.tickOnce() == BT::NodeStatus::SUCCESS);
  CHECK(running_tree.tickOnce() == BT::NodeStatus::FAILURE);
  CHECK(!running_board->get<Sense>("sense").match_running);
  CHECK(stopped_board->get<Sense>("sense").match_running);
}

void missing_and_wrong_type_inputs_report_errors()
{
  BT::BehaviorTreeFactory factory;
  register_nodes(factory);
  const auto missing = BT::Blackboard::create();
  expect_error([&] {
    auto tree = leaf(factory, missing, "<MatchRunning sense=\"{missing_sense}\"/>");
    tree.tickOnce();
  });
  const auto wrong_type = BT::Blackboard::create();
  wrong_type->set("selected_sense", 1);
  expect_error([&] {
    auto tree = leaf(factory, wrong_type, "<MatchRunning sense=\"{selected_sense}\"/>");
    tree.tickOnce();
  });
}

void subtree_remapping_preserves_running_memory()
{
  BT::BehaviorTreeFactory factory;
  register_nodes(factory);
  const auto default_io = hardware(350);
  default_io->goal = kGoalRunning;
  const auto selected_io = hardware(350);
  selected_io->goal = kGoalRunning;
  ExecState default_execution;
  default_execution.applied_goal = 101;
  const auto board = make_blackboard(config(), default_io, {}, default_execution);
  ExecState selected_execution;
  selected_execution.applied_goal = 40;
  board->set("selected_execution", selected_execution);
  board->set<std::shared_ptr<Ports>>("selected_io", selected_io);
  auto tree = factory.createTreeFromText(R"(
    <root BTCPP_format="4" main_tree_to_execute="Caller">
      <BehaviorTree ID="Caller">
        <SubTree ID="StopChild" io="{selected_io}" execution="{selected_execution}"/>
      </BehaviorTree>
      <BehaviorTree ID="StopChild">
        <WaitStopped io="{io}" execution="{execution}"/>
      </BehaviorTree>
    </root>)", board);
  CHECK(tree.tickOnce() == BT::NodeStatus::RUNNING);
  CHECK(board->get<ExecState>("selected_execution").stop_sent);
  CHECK(board->get<ExecState>("selected_execution").applied_goal == -2);
  CHECK(selected_io->goal == kGoalIdle);
  CHECK(board->get<ExecState>("execution").applied_goal == 101);
  CHECK(default_io->goal == kGoalRunning);

  tree.haltTree();
  CHECK(board->get<ExecState>("selected_execution").stop_sent);
  CHECK(tree.tickOnce() == BT::NodeStatus::RUNNING);
  CHECK(board->get<ExecState>("selected_execution").stop_sent);
  CHECK(board->get<ExecState>("execution").applied_goal == 101);
}
}  // namespace

int main()
{
  test("initialized typed blackboard", initialized_board_contract);
  test("condition input remapping", condition_reads_remapped_sense);
  test("sensor input/output and operator memory remapping", read_inputs_remaps_input_output_and_memory);
  test("selector output remapping", selector_writes_remapped_target);
  test("commit inout remapping", commit_target_writes_remapped_memory_and_target);
  test("reusable registration and isolated trees", registration_is_reusable_and_tree_boards_are_isolated);
  test("missing and wrong type inputs fail", missing_and_wrong_type_inputs_report_errors);
  test("subtree RUNNING memory remapping", subtree_remapping_preserves_running_memory);
  return failures == 0 ? 0 : 1;
}
