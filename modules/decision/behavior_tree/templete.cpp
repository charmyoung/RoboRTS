#include "modules/decision/behavior_tree/behavior_tree.h"

namespace rrts{
namespace decision{

class ActionTest: public ActionNode{
 public:
  ActionTest(std::string name, const Blackboard::Ptr &blackboard_ptr, unsigned int cycle_num):
      ActionNode::ActionNode(name,blackboard_ptr),
      cycle_num_(cycle_num),cycle_index_(0){}
  virtual ~ActionTest()= default;
 private:
  virtual void OnInitialize() {
    cycle_index_ = 0;
    std::cout<<name_<<" "<<__FUNCTION__<<std::endl;
  };
  virtual BehaviorState Update() {
    std::cout<<name_<<" "<<__FUNCTION__<<std::endl;
    cycle_index_++;
    if (cycle_index_ == cycle_num_) {
      if((std::rand()%2 ==0)){
        return SUCCESS;
      }else{
        return FAILURE;
      }
    } else{
      return RUNNING;
    }
  }
  virtual void OnTerminate(BehaviorState state) {
    switch (state){
      case BehaviorState::IDLE:
        std::cout<<name_<<" "<<__FUNCTION__<<" IDLE!"<<std::endl;
        break;
      case BehaviorState::SUCCESS:
        std::cout<<name_<<" "<<__FUNCTION__<<" SUCCESS!"<<std::endl;
        break;
      case BehaviorState::FAILURE:
        std::cout<<name_<<" "<<__FUNCTION__<<" FAILURE!"<<std::endl;
        break;
      default:
        std::cout<<name_<<" "<<__FUNCTION__<<" ERROR!"<<std::endl;
        return;
    }
  };

  unsigned int cycle_num_;
  unsigned int cycle_index_;
};
class PreconditionTest: public PreconditionNode {
 public:
  PreconditionTest(std::string name, const Blackboard::Ptr &blackboard_ptr,
                   const BehaviorNode::Ptr &child_node_ptr = nullptr) :
      PreconditionNode(name, blackboard_ptr, child_node_ptr) {}
  virtual ~PreconditionTest() = default;
 private:
  virtual bool Precondition() {
    bool condition = static_cast<bool>(std::rand() % 2 == 0);
    std::cout << __FUNCTION__ << ": " << condition << std::endl;
    return condition;
  }

};

}//namespace decision
} //namespace rrts

int main(){
  auto blackboard_ptr = std::make_shared<rrts::decision::Blackboard>();
  /*
   *                             root
   *                              |
   *                           sequence
   *                          /   |    \
   *                      pre0  par0    seq0
   *                        |    /  \     |  \
   *                      sel0 act2 act3 act4 act5
   *                      /   \
   *                  act0   act1
   *
   */


  auto action0_ptr = std::make_shared<rrts::decision::ActionTest>("root - precondition0 - selector0 - action0",blackboard_ptr,5);
  auto action1_ptr = std::make_shared<rrts::decision::ActionTest>("root - precondition0 - selector0 - action1",blackboard_ptr,6);

  auto action2_ptr = std::make_shared<rrts::decision::ActionTest>("root - sequence0 - action2",blackboard_ptr,3);
  auto action3_ptr = std::make_shared<rrts::decision::ActionTest>("root - sequence0 - action3",blackboard_ptr,4);

  auto action4_ptr = std::make_shared<rrts::decision::ActionTest>("root - parallel0 - action4",blackboard_ptr,2);
  auto action5_ptr = std::make_shared<rrts::decision::ActionTest>("root - parallel0 - action5",blackboard_ptr,6);

  auto selector0_ptr = std::make_shared<rrts::decision::SelectorNode>("root - precondition0 - selector0",blackboard_ptr);
  selector0_ptr->AddChildren(action0_ptr);
  selector0_ptr->AddChildren(action1_ptr);
  auto precond0_ptr = std::make_shared<rrts::decision::PreconditionTest>("root - precondition0",blackboard_ptr, selector0_ptr);

  auto sequence0_ptr = std::make_shared<rrts::decision::SequenceNode>("root - sequence0",blackboard_ptr);
  sequence0_ptr->AddChildren(action2_ptr);
  sequence0_ptr->AddChildren(action3_ptr);

  auto parallel0_ptr = std::make_shared<rrts::decision::ParallelNode>("root - parallel0",blackboard_ptr,2);
  parallel0_ptr->AddChildren(action4_ptr);
  parallel0_ptr->AddChildren(action5_ptr);

  auto root_ptr = std::make_shared<rrts::decision::SequenceNode>("root",blackboard_ptr);

  root_ptr->AddChildren(precond0_ptr);
  root_ptr->AddChildren(sequence0_ptr);
  root_ptr->AddChildren(parallel0_ptr);
  rrts::decision::BehaviorTree bt(root_ptr,100);
  bt.Execute();
};