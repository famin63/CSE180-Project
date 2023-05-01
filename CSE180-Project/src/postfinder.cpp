/*
Copyright 2023 Stefano Carpin

Licensed under the Apache License, Version 2.0 (the "License");
you may not use this file except in compliance with the License.
You may obtain a copy of the License at

    http://www.apache.org/licenses/LICENSE-2.0

Unless required by applicable law or agreed to in writing, software
distributed under the License is distributed on an "AS IS" BASIS,
WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
See the License for the specific language governing permissions and
limitations under the License.
*/

#include <rclcpp/rclcpp.hpp>
#include <navigation/navigation.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <iostream>

// callback function for the laser scan data
void laserCallback(const sensor_msgs::msg::LaserScan::SharedPtr scan_data) {
  // process the laser scan data here
}

int main(int argc, char **argv) {
  rclcpp::init(argc, argv); // initialize ROS
  Navigator navigator(true, false); // create node with debug info but not verbose

  // first: it is mandatory to initialize the pose of the robot
  geometry_msgs::msg::Pose::SharedPtr init = std::make_shared<geometry_msgs::msg::Pose>();
  init->position.x = -2;
  init->position.y = -0.5;
  init->orientation.w = 1;
  navigator.SetInitialPose(init);

  // wait for navigation stack to become operational
  navigator.WaitUntilNav2Active();

  // subscribe to the laser scan data
  auto laser_sub = navigator.create_subscription<sensor_msgs::msg::LaserScan>(
    "/scan", 10, laserCallback);

  // spin in place of 90 degrees (default parameter)
  navigator.Spin();
  while (!navigator.IsTaskComplete()) {
    // busy waiting for task to be completed
  }

  geometry_msgs::msg::Pose::SharedPtr goal_post = std::make_shared<geometry_msgs::msg::Pose>();
  for (float y = 1.5; y >= -1.5; y--) {
    for (float x = -2; x <= 2; x++) {
      goal_post->position.x = x;
      goal_post->position.y = y;
      goal_post->orientation.w = 1;
      // move to new pose
      navigator.GoToPose(goal_post);
      while (!navigator.IsTaskComplete()) {
        // busy waiting for task to be completed
      }
    }
  }

  // backup of 0.15 m (default distance)
  navigator.Backup();
  while (!navigator.IsTaskComplete()) {
    // busy waiting for task to be completed
  }

  // complete here....

  rclcpp::spin_some(navigator.get_node_base_interface());
  rclcpp::shutdown(); // shutdown ROS
  return 0;
}

