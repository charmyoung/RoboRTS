
#include <iostream>
#include <random>
#include <memory>

#include "ros/ros.h"
#include <tf/transform_listener.h>
#include "geometry_msgs/PoseStamped.h"
#include "modules/perception/map/costmap/costmap_interface.h"

int main(int argc, char **argv){
  ros::init(argc,argv,"decision_test");

  ros::NodeHandle rviz_nh("move_base_simple");
  ros::Publisher goal_pub = rviz_nh.advertise<geometry_msgs::PoseStamped>("goal",100);
  //first policy: random goal and judge valid, if valid then send goal, see if use global is wrong.
  std::shared_ptr<tf::TransformListener> tf_ptr_ = std::make_shared<tf::TransformListener>(ros::Duration(10));

  std::shared_ptr<rrts::perception::map::CostmapInterface> costmap_ptr_ = std::make_shared<rrts::perception::map::CostmapInterface>("decision_costmap",
                                                                           *tf_ptr_,
                                                                           "modules/perception/map/costmap/config/costmap_parameter_config_for_global_plan.prototxt");


  unsigned int gridmap_height_ =costmap_ptr_->GetCostMap()->GetSizeXCell();
  unsigned int gridmap_width_ = costmap_ptr_->GetCostMap()->GetSizeYCell();
  unsigned int size_ = gridmap_height_*gridmap_width_;
  unsigned char * charmap_ = costmap_ptr_->GetCostMap()->GetCharMap();

  unsigned int goal_index,goal_cell_x,goal_cell_y;
  geometry_msgs::PoseStamped goal;
  goal.pose.orientation.w = 1;
  goal.header.frame_id = "map";


  ros::Rate loop_rate(10);

  while (ros::ok()) {
    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_int_distribution<int> uni_dis(0, size_-1);
    while (true) {
      unsigned int random_index = uni_dis(gen);
      if (charmap_[random_index] < 240) {
        goal_index=random_index;
        break;
      }
    }
    costmap_ptr_->GetCostMap()->Index2Cells(goal_index, goal_cell_x, goal_cell_y);
    costmap_ptr_->GetCostMap()->Map2World(goal_cell_x, goal_cell_y, goal.pose.position.x, goal.pose.position.y);
    goal_pub.publish(goal);
    loop_rate.sleep();
  }
}