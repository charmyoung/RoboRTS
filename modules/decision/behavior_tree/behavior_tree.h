#ifndef MODULE_DECISION_BEHAVIOR_TREE_BEHAVIOR_TREE_H
#define MODULE_DECISION_BEHAVIOR_TREE_BEHAVIOR_TREE_H

#include <chrono>

#include "modules/decision/behavior_tree/behavior_node.h"

namespace rrts{
namespace decision {
class BehaviorTree {
 public:
  BehaviorTree(BehaviorNode::Ptr root_node, int cycle_duration) :
      root_node_(root_node),
      cycle_duration_(cycle_duration),
      running_(false) {}

  void Configuration() {

  }
  void Execute() {
    running_ = true;
    unsigned int frame = 0;
    while (running_) {
      std::chrono::steady_clock::time_point start_time = std::chrono::steady_clock::now();

      std::cout << "Frame " << frame << ":" << std::endl;
      root_node_->Run();
      // for debug
      if (root_node_->GetBehaviorState() == BehaviorNode::BehaviorState::SUCCESS) { break; }

      std::chrono::steady_clock::time_point end_time = std::chrono::steady_clock::now();
      std::chrono::milliseconds execution_duration =
          std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time);
      std::chrono::milliseconds sleep_time = cycle_duration_ - execution_duration;
      if (sleep_time > std::chrono::milliseconds(0)) {
        std::this_thread::sleep_for(sleep_time);
        std::cout << "sleep: " << sleep_time.count() << "ms" << std::endl;
      } else {
        std::cout << "The time planning once is " << execution_duration.count() << " beyond the expected time "
                  << cycle_duration_.count() << std::endl;
      }
      std::cout << "----------------------------------" << std::endl;
      frame++;
    }

  }
 private:
  BehaviorNode::Ptr root_node_;
  std::chrono::milliseconds cycle_duration_;
  bool running_;
};

}//namespace decision
}//namespace rrts

#endif //MODULE_DECISION_BEHAVIOR_TREE_BEHAVIOR_TREE_H
