
#include <iostream>

#include "ros/ros.h"
#include "geometry_msgs/PoseStamped.h"

int main(int argc, char **argv){
  ros::init(argc,argv,"decision_test");

  ros::NodeHandle rviz_nh("move_base_simple");
  ros::Publisher goal_pub = rviz_nh.advertise<geometry_msgs::PoseStamped>("goal",100);
  //first policy: random goal and judge valid, if valid then send goal, see if use global is wrong.
  tf_ptr_ = std::make_shared<tf::TransformListener>(ros::Duration(10));

  costmap_ptr_ = std::make_shared<rrts::perception::map::CostmapInterface>("decision_costmap",
                                                                           *tf_ptr_,
                                                                           "modules/perception/map/costmap/config/costmap_parameter_config_for_global_plan.prototxt");
  unsigned char * charmap_ = costmap_ptr_->GetCostMap()->GetCharMap();
  ros::Rate loop_rate(10);
  while(ros::ok()){

    loop_rate.sleep();
  }
}