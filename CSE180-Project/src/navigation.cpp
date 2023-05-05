#include <rclcpp/rclcpp.hpp> 
#include <navigation/navigation.hpp>
#include <iostream>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <std_msgs/msg/float32.hpp>

nav_msgs::msg::OccupancyGrid::SharedPtr original_map;
bool original = true;
rclcpp::Node::SharedPtr nodeh;

void mapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg){
        std::vector<int> update_index;
        int filter = 95;
    
        if(original) {
            original_map = msg;
            original = false;
        }
        else {
            for(uint i = 0; i < msg->data.size(); i++){
                if((int)msg->data[i] - (int)original_map->data[i] > filter){
                    update_index.push_back(i);
                }
            }
        }
        RCLCPP_INFO_STREAM(nodeh->get_logger(), "Size of Changes: " << update_index.size());
        
        for (uint i = 0; i < update_index.size(); i++) {
            int x = update_index % msg->info.width;
            int y = update_index / msg->info.height;

            RCLCPP_INFO_STREAM(nodeh->get_logger(), "2D Map: (" << x << "," << y << ")";

            x_coordinate = x * msg->info.resolution - 10.0;
            y_coordinate = y * msg->info.resolution - 10.0;
            float x_coordinate = x * msg->info.resolution - 10.0;
            float y_coordinate = y * msg->info.resolution - 10.0;

            RCLCPP_INFO_STREAM(nodeh->get_logger(), "COORD. FRAME: (" << x_coordinate << "," << y_coordinate << ")";
        }
}
int main(int argc, char **argv) {

  rclcpp::init(argc,argv); // initialize ROS 
  Navigator navigator(true,false); // create node with debug info but not verbose
  rclcpp::init(argc, argv); // initialize ROS 
  Navigator navigator(true, false); // create node with debug info but not verbose

  // first: it is mandatory to initialize the pose of the robot
  geometry_msgs::msg::Pose::SharedPtr init = std::make_shared<geometry_msgs::msg::Pose>();

  init->position.x = -2;
  init->position.y = -0.5;
  init->orientation.w = 1;
  navigator.SetInitialPose(init);

  // wait for navigation stack to become operationale
  // wait for navigation stack to become operational
  navigator.WaitUntilNav2Active();
    navigator.Spin();
  while ( ! navigator.IsTaskComplete() ) {
    // busy waiting for task to be completed
  }
  geometry_msgs::msg::Pose::SharedPtr goal_pos = std::make_shared<geometry_msgs::msg::Pose>();
  goal_pos->position.x = 2;
  goal_pos->position.y = 1;
  goal_pos->orientation.w = 0.707;
  goal_pos->orientation.z = 0.707;
  // move to new pose
  navigator.GoToPose(goal_pos);
  while ( ! navigator.IsTaskComplete() ) {
    
  }
  goal_pos->position.x = 2;
  goal_pos->position.y = -1;
  goal_pos->orientation.w = 1;
  navigator.GoToPose(goal_pos);
  // move to new pose
  while ( ! navigator.IsTaskComplete() ) {
    
  }
  // backup of 0.15 m (deafult distance)
  navigator.Backup();
  while ( ! navigator.IsTaskComplete() ) {
    
  }


 /* // set up goal pose for robot to explore the environment
  auto goal_pose = std::make_shared<geometry_msgs::msg::Pose>();
  goal_pose->position.x = 2.0;
  goal_pose->position.y = 1.0;
  goal_pose->orientation.w = 0.707;
  goal_pose->orientation.z = 0.707;

  // get path to goal pose
  //auto path = navigator.GetPath(goal_pose);

  while (rclcpp.ok()) {}

  goal_pos->position.x = 2;
  goal_pos->position.y = -1;
  goal_pos->orientation.w = 1;
  navigator.GoToPose(goal_pos);
  // move to new pose
  while ( ! navigator.IsTaskComplete() ) {
    
  }
  // backup of 0.15 m (deafult distance)
  navigator.Backup();
  while ( ! navigator.IsTaskComplete() ) {
    
  }*/
  rclcpp::shutdown(); // shutdown ROS
  return 0;
}
