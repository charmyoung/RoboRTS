/****************************************************************************
 *  Copyright (C) 2018 RoboMaster.
 *
 *  This program is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation, either version 3 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of 
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program. If not, see <http://www.gnu.org/licenses/>.
 ***************************************************************************/

#include <ros/ros.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <thread>
#include "messages/GlobalPlannerAction.h"
#include "messages/LocalPlannerAction.h"
#include "messages/ArmorDetectionAction.h"
#include "messages/LocalizationAction.h"

#include "common/error_code.h"
#include "common/io.h"
#include "common/log.h"
#include "common/node_state.h"

#include "modules/decision/proto/decision.pb.h"
#include "modules/perception/map/costmap/costmap_interface.h"

namespace rrts{
namespace decision{

using rrts::common::ErrorCode;
using rrts::common::ErrorInfo;

class DecisionNode {
 public:
  DecisionNode() : global_planner_actionlib_client_("global_planner_node_action", true),
                   local_planner_actionlib_client_("local_planner_node_action", true),
                   localization_actionlib_client_("localization_node_action", true),
                   armor_detection_actionlib_client_("armor_detection_node_action", true),
                   action_state_(global_planner_actionlib_client_.getState()),
                   rviz_goal_(false), new_path_(false),random_count_(0),
                   decision_state_(rrts::common::IDLE) {
    //point mode
    ros::NodeHandle rviz_nh("move_base_simple");
    goal_sub_ = rviz_nh.subscribe<geometry_msgs::PoseStamped>("goal", 1,
                                                              &DecisionNode::GoalCallback, this);
    ros::NodeHandle nh;
    random_sub_ = nh.subscribe<geometry_msgs::PointStamped>("clicked_point", 1, &DecisionNode::RandomCallback, this);

    LoadParam();
    localization_actionlib_client_.waitForServer();
    std::cout << "Localization module has been connected!" << std::endl;
    localization_goal_.command = 1;
    localization_actionlib_client_.sendGoal(localization_goal_);

//    armor_detection_actionlib_client_.waitForServer();
//    std::cout<<"Armor detection module has been connected!"<<std::endl;
//    armor_detection_goal_.command = 1;
//    armor_detection_actionlib_client_.sendGoal(armor_detection_goal_,
//                                               actionlib::SimpleActionClient<messages::ArmorDetectionAction>::SimpleDoneCallback(),
//                                               actionlib::SimpleActionClient<messages::ArmorDetectionAction>::SimpleActiveCallback(),
//                                               boost::bind(&DecisionNode::ArmorDetectionFeedbackCallback, this, _1));

    global_planner_actionlib_client_.waitForServer();
    std::cout << "Global planner module has been connected!" << std::endl;

    local_planner_actionlib_client_.waitForServer();
    std::cout << "Local planner module has been connected!" << std::endl;

    tf_ptr_ = std::make_shared<tf::TransformListener>(ros::Duration(10));

    costmap_ptr_ = std::make_shared<rrts::perception::map::CostmapInterface>("decision_costmap",
                                                                             *tf_ptr_,
                                                                             "modules/perception/map/costmap/config/costmap_parameter_config_for_global_plan.prototxt");

    gridmap_width_ = costmap_ptr_->GetCostMap()->GetSizeXCell();
    gridmap_height_ = costmap_ptr_->GetCostMap()->GetSizeYCell();
    size_ = gridmap_height_ * gridmap_width_;
    charmap_ = costmap_ptr_->GetCostMap()->GetCharMap();

    thread_ = std::thread(&DecisionNode::Execution, this);
  }

  void LoadParam() {
    rrts::decision::DecisionConfig decision_config;
    std::string file_name = "modules/decision/config/decision.prototxt";
    rrts::common::ReadProtoFromTextFile(file_name, &decision_config);

    //patrol goal config
    unsigned int point_size = decision_config.point().size();
    patrol_goals_.resize(point_size);
    for (int i = 0; i != point_size; i++) {

      patrol_goals_[i].pose.position.x = decision_config.point(i).x();
      patrol_goals_[i].pose.position.y = decision_config.point(i).y();
      patrol_goals_[i].pose.position.z = decision_config.point(i).z();

      tf::Quaternion quaternion = tf::createQuaternionFromRPY(decision_config.point(i).roll(),
                                                              decision_config.point(i).pitch(),
                                                              decision_config.point(i).yaw());
      patrol_goals_[i].pose.orientation.x = quaternion.x();
      patrol_goals_[i].pose.orientation.y = quaternion.y();
      patrol_goals_[i].pose.orientation.z = quaternion.z();
      patrol_goals_[i].pose.orientation.w = quaternion.w();
    }
    patrol_goals_iter_ = patrol_goals_.begin();

    //random generate goal config

  }
  void RandomCallback(const geometry_msgs::PointStamped::ConstPtr &point) {

    if (random_count_ % 2 == 0){
      x1_ = point->point.x;
      y1_= point->point.y;
    }else{

      if(x1_<=point->point.x){
        x2_=point->point.x;
      }else{
        x2_=x1_;
        x1_=point->point.x;
      }
      if(y1_<=point->point.y){
        y2_=point->point.y;
      }else{
        y2_=y1_;
        y1_=point->point.y;
      }
    }
    random_count_++;

  }
  void GoalCallback(const geometry_msgs::PoseStamped::ConstPtr & goal){
    global_planner_goal_.goal = *goal;

    rviz_goal_ = true;
  }
  void RandomSample(float x1, float x2, float y1, float y2, geometry_msgs::PoseStamped& random_goal){
    random_goal.header.frame_id = "map";
    float goal_yaw,goal_x,goal_y;
    unsigned int goal_cell_x, goal_cell_y;

    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_real_distribution<float> x_uni_dis(x1,x2);
    std::uniform_real_distribution<float> y_uni_dis(y1,y2);
    std::uniform_real_distribution<float> yaw_uni_dis(-3.14f, 3.14f);

    while (true) {
      goal_x = x_uni_dis(gen);
      goal_y = y_uni_dis(gen);
      costmap_ptr_->GetCostMap()->World2Map(goal_x,
                                            goal_y,
                                            goal_cell_x,
                                            goal_cell_y);
      unsigned int goal_index = costmap_ptr_->GetCostMap()->GetIndex(goal_cell_x, goal_cell_y);
      if (charmap_[goal_index] < 240) {
        break;
      }
    }
    goal_yaw = yaw_uni_dis(gen);
    tf::Quaternion quaternion = tf::createQuaternionFromRPY(0,0,goal_yaw);
//        costmap_ptr_->GetCostMap()->Index2Cells(goal_index, goal_cell_x, goal_cell_y);
//        costmap_ptr_->GetCostMap()->Map2World(goal_cell_x, goal_cell_y, random_goal.pose.position.x, random_goal.pose.position.y);
    random_goal.pose.position.x = goal_x;
    random_goal.pose.position.y = goal_y;
    random_goal.pose.orientation.w = quaternion.w();
    random_goal.pose.orientation.x = quaternion.x();
    random_goal.pose.orientation.y = quaternion.y();
    random_goal.pose.orientation.z = quaternion.z();
  }
  void Execution(){


    while(ros::ok()) {
      // 1. rviz mark nav goal
      if (rviz_goal_) {
//        patrol_goals_iter_--;
        global_planner_actionlib_client_.sendGoal(global_planner_goal_,
                                                  boost::bind(&DecisionNode::GlobalPlannerDoneCallback, this, _1, _2),
                                                  boost::bind(&DecisionNode::GlobalPlannerActiveCallback, this),
                                                  boost::bind(&DecisionNode::GlobalPlannerFeedbackCallback, this, _1)
        );
        decision_state_ = rrts::common::RUNNING;
        rviz_goal_ = false;

      } // 2. patrol points read from proto.txt file
      else if(!patrol_goals_.empty() && decision_state_ == rrts::common::IDLE) {

        if (patrol_goals_iter_ == patrol_goals_.end()) {
          patrol_goals_iter_ = patrol_goals_.begin();
        }
        global_planner_goal_.goal.header.frame_id = "map";
        global_planner_goal_.goal=*patrol_goals_iter_;
        global_planner_actionlib_client_.sendGoal(global_planner_goal_,
                                                  boost::bind(&DecisionNode::GlobalPlannerDoneCallback, this, _1, _2),
                                                  boost::bind(&DecisionNode::GlobalPlannerActiveCallback, this),
                                                  boost::bind(&DecisionNode::GlobalPlannerFeedbackCallback, this, _1));
        decision_state_ = rrts::common::RUNNING;
        patrol_goals_iter_++;
      } //3. random generate valid goal
      else if(random_count_!=0 && random_count_%2 == 0 && decision_state_ == rrts::common::IDLE) {

        RandomSample(x1_,x2_,y1_,y2_,global_planner_goal_.goal);
        global_planner_actionlib_client_.sendGoal(global_planner_goal_,
                                                  boost::bind(&DecisionNode::GlobalPlannerDoneCallback, this, _1, _2),
                                                  boost::bind(&DecisionNode::GlobalPlannerActiveCallback, this),
                                                  boost::bind(&DecisionNode::GlobalPlannerFeedbackCallback, this, _1));
        decision_state_ = rrts::common::RUNNING;
      }
      //local goal
      if (new_path_) {

        local_planner_actionlib_client_.sendGoal(local_planner_goal_);
        new_path_ = false;

      }

//      actionlib::SimpleClientGoalState action_state=global_planner_actionlib_client_.getState();
//      if(action_state_ !=action_state){
//        action_state_=action_state;
//        std::cout<<"***:"<<action_state_.toString()<<std::endl;
//      }

    }

  }

  // Global Planner
  void GlobalPlannerDoneCallback(const actionlib::SimpleClientGoalState& state,  const messages::GlobalPlannerResultConstPtr& result){
    std::cout<<"Global planner "<<state.toString().c_str()<<"!"<<std::endl;
    decision_state_ = rrts::common::IDLE;
  }
  void GlobalPlannerActiveCallback(){
    LOG_INFO<<"Global planner server has recived the goal!";
  }
  void GlobalPlannerFeedbackCallback(const messages::GlobalPlannerFeedbackConstPtr& feedback){
    if (feedback->error_code != ErrorCode::OK) {
      std::cout<<"Global planner: "<<feedback->error_msg<<std::endl;
    }
    if (!feedback->path.poses.empty()) {
      local_planner_goal_.route = feedback->path;
      new_path_=true;
    }
  }

  // Armor Detection
  void ArmorDetectionFeedbackCallback(const messages::ArmorDetectionFeedbackConstPtr& feedback){
    if (feedback->error_code != ErrorCode::OK) {
    }
    else {
      //new_path_=true;
    }
  }

  ~DecisionNode(){
//    localization_goal_.command = '3';
//    localization_actionlib_client_.sendGoal(localization_goal_);
//    armor_detection_goal_.command = '3';
//    armor_detection_actionlib_client_.sendGoal(armor_detection_goal_);
    if(thread_.joinable()){
      thread_.join();
    }

  }
 private:
  actionlib::SimpleActionClient<messages::GlobalPlannerAction> global_planner_actionlib_client_;
  actionlib::SimpleActionClient<messages::LocalPlannerAction> local_planner_actionlib_client_;
  actionlib::SimpleActionClient<messages::LocalizationAction> localization_actionlib_client_;
  actionlib::SimpleActionClient<messages::ArmorDetectionAction> armor_detection_actionlib_client_;

  messages::LocalizationGoal localization_goal_;
  messages::ArmorDetectionGoal armor_detection_goal_;
  messages::GlobalPlannerGoal global_planner_goal_;
  messages::LocalPlannerGoal local_planner_goal_;

  bool rviz_goal_;
  bool new_path_;
  std::thread thread_;
  rrts::common::NodeState decision_state_;
  //! costmap
  std::shared_ptr<tf::TransformListener> tf_ptr_;
  std::shared_ptr<rrts::perception::map::CostmapInterface> costmap_ptr_;
  unsigned int gridmap_height_;
  unsigned int gridmap_width_;
  unsigned int size_;
  unsigned char * charmap_;

  ros::Subscriber goal_sub_;

  std::vector<geometry_msgs::PoseStamped> patrol_goals_;
  std::vector<geometry_msgs::PoseStamped>::iterator patrol_goals_iter_;

  //random generate goal
  ros::Subscriber random_sub_;
  unsigned int random_count_;
  float x1_,x2_,y1_,y2_;


  actionlib::SimpleClientGoalState action_state_;//for test.
};

}// namespace decision
}// namespace rrts

int main(int argc, char **argv) {
  ros::init(argc, argv, "decision_node");
  rrts::decision::DecisionNode global_planner_client;
  ros::spin();
  return 0;
}
