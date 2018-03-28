#ifndef MODULE_DECISION_BEHAVIOR_TREE_BLACKBOARD_H
#define MODULE_DECISION_BEHAVIOR_TREE_BLACKBOARD_H

#include <iostream>

namespace rrts{
namespace decision {

class Blackboard {
 public:
  typedef std::shared_ptr<Blackboard> Ptr;
  Blackboard() {};
  ~Blackboard() {};
  //! Goal
  void SetGoal(geometry_msgs::PoseStamped goal) {
    goal_ = goal;
  }
  geometry_msgs::PoseStamped GetGoal() {
    return goal_;
  }
  //! Path
  void SetPath(nav_msgs::Path path) {
    pathl_ = path;
  }
  nav_msgs::Path GetPath() {
    return path_;
  }
  //! EnemyFound
  void SetEnemyFound() {
    enemy_found_ = enemy_found;
  }
  bool GetEnemyFound() {
    return enemy_found_;
  }

 private:
  geometry_msgs::PoseStamped goal_;
  nav_msgs::Path path_;
  bool enemy_found_;
};
} //namespace decision
} //namespace rrts
#endif //MODULE_DECISION_BEHAVIOR_TREE_BLACKBOARD_H
