// Production behavior tree: typed XML ports expose each leaf's data dependencies.
#pragma once

#include <behaviortree_cpp/bt_factory.h>
#include <memory>

#include "bbt_decide/planner.hpp"
#include "bbt_decide/ports.hpp"

namespace bbt_decide
{
// Create one independent native blackboard per tree. Canonical root keys:
// config: shared_ptr<const Config>, io: shared_ptr<Ports> (injected dependencies);
// sense: Sense, plan: PlanState, execution: ExecState, target: Decision (values).
// Nodes access only declared ports, so XML/SubTree mappings may rename these keys.
// plan/execution persist through reactive halts; no separate Context mirror exists.
BT::Blackboard::Ptr make_blackboard(std::shared_ptr<const Config> config,
                                    std::shared_ptr<Ports> io,
                                    PlanState plan = {}, ExecState execution = {});

// Registration contains no tree-specific state. One factory can build independent trees.
void register_nodes(BT::BehaviorTreeFactory& factory);

} // namespace bbt_decide
