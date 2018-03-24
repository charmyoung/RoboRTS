#ifndef MODULE_DECISION_BEHAVIOR_TREE_GLOBAL_PLAN_BEHAVIOR_H
#define MODULE_DECISION_BEHAVIOR_TREE_GLOBAL_PLAN_BEHAVIOR_H

#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include "messages/GlobalPlannerAction.h"

#include "common/error_code.h"

#include "modules/decision/behavior_tree/behavior_tree.h"
#include "modules/decision/behavior_tree/blackboard.h"

namespace rrts{
namespace decision{

class GlobalPlannerBehavior: public ActionNode{
 public:
  GlobalPlannerBehavior(const Blackboard::Ptr &blackboard_ptr):
      ActionNode::ActionNode("global_planner_behavior", blackboard_ptr),
      actionlib_client_("global_planner_node_action", true){
    blackboard_ptr.Setbool("global_planner_behavior",false);
    actionlib_client_.waitForServer();
    blackboard_ptr.Setbool("global_planner_behavior",true);
  }
  virtual ~GlobalPlannerBehavior()= default;

 private:
  virtual void OnInitialize() {
    messages::GlobalPlannerGoal global_planner_goal;
    actionlib_client_.sendGoal(global_planner_goal,
                                                  boost::bind(&DecisionNode::GlobalPlannerDoneCallback, this, _1, _2),
                                                  boost::bind(&DecisionNode::GlobalPlannerActiveCallback, this),
                                                  boost::bind(&DecisionNode::GlobalPlannerFeedbackCallback, this, _1));
    std::cout<<name_<<" "<<__FUNCTION__<<std::endl;
  };
  virtual BehaviorState Update() {
    while (actionlib_client_.getState() == ) {

    }
    return action_state_;
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
  }

  void GlobalPlannerActiveCallback(){
    action_state_ = BehaviorState::RUNNING;
  }

  void GlobalPlannerFeedbackCallback(const messages::GlobalPlannerFeedbackConstPtr& feedback){
    if (feedback->error_code != ErrorCode::OK) {
    // std::cout<<"Global planner: "<<feedback->error_msg<<std::endl;
    }
    if (!feedback->path.poses.empty()) {
      local_planner_goal_.route = feedback->path;
    }
  }

  void GlobalPlannerDoneCallback(const actionlib::SimpleClientGoalState& state,
                                 const messages::GlobalPlannerResultConstPtr& result){

  }

  BehaviorState action_state_;
  actionlib::SimpleActionClient<messages::GlobalPlannerAction> actionlib_client_;

};

} //namespace decision
} //namespace rrts
#endif //MODULE_DECISION_BEHAVIOR_TREE_GLOBAL_PLAN_BEHAVIOR_H
