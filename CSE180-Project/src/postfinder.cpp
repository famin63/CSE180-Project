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
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <tf2_ros/transform_listener.h>
#include <iostream>

using namespace std;

// global variables
nav_msgs::msg::OccupancyGrid map_data;
geometry_msgs::msg::Pose amcl_pose;
bool amcl_pose_received = false;
bool map_received = false;
bool laser_received = false;
geometry_msgs::msg::TransformStamped laser_tf;

// callback function for the laser scan data
void laserCallback(const sensor_msgs::msg::LaserScan::SharedPtr scan_data) {
  laser_received = true;
}

// callback function for the occupancy grid data
void mapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr map) {
  map_data = *map;
  map_received = true;
}

// callback function for the AMCL pose
void amclPoseCallback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr pose) {
  amcl_pose = pose->pose.pose;
  amcl_pose_received = true;
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

  // subscribe to the occupancy grid data
  auto map_sub = navigator.create_subscription<nav_msgs::msg::OccupancyGrid>(
    "/map", 10, mapCallback);

  // subscribe to the AMCL pose
  auto amcl_pose_sub = navigator.create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
    "/amcl_pose", 10, amclPoseCallback);

  // create a transform listener for the laser scan tf
  tf2_ros::Buffer tf_buffer;
  tf2_ros::TransformListener tf_listener(tf_buffer);

  // move to initial position
  while (!navigator.IsTaskComplete()) {
    // busy waiting for task to be completed
  }

  // backup of 0.15 m (default distance)
  navigator.Backup();
  while (!navigator.IsTaskComplete()) {
    // busy waiting for task to be completed
  }

  // patrol the map
  while (rclcpp::ok()) {
    if (!amcl_pose_received || !map_received || !laser_received) {
      // wait for all the required data to be received
      rclcpp::spin_some(navigator.get_node_base_interface());
      continue;
    }

    // get the laser scan tf
    try {
      laser_tf = tf_buffer.lookupTransform("base_link", "laser", tf2::TimePoint());
    }
    catch (tf2::TransformException &ex) {
      RCLCPP_WARN(navigator.get_logger(), "Failed to get laser scan tf: %s", ex.what());
      continue;
    }

    // loop over all the cells in the map
    for (int i = 0; i < map_data.info.width * map_data.info.height; i++) {
// convert cell index to coordinates
int x = i % map_data.info.width;
int y = i / map_data.info.width;
double map_x = (x - (double)map_data.info.width/2) * map_data.info.resolution;
double map_y = (y - (double)map_data.info.height/2) * map_data.info.resolution;
  // check if the cell is occupied in the map
  if (map_data.data[i] > 65) {
    // calculate distance from robot to cell
    double dist = sqrt(pow(amcl_pose.position.x - map_x, 2) + pow(amcl_pose.position.y - map_y, 2));

    // get range measurement from laser scan data
    double angle = atan2(map_y - amcl_pose.position.y, map_x - amcl_pose.position.x) - tf2::getYaw(amcl_pose.orientation);
    double range = scan_data->ranges[angle / scan_data->angle_increment];

    // check if there is a mismatch between the map and the laser data
    if (range < dist - 0.1) {
      // log the mismatch
      RCLCPP_INFO(navigator.get_logger(), "Mismatch found: Map says occupied at (%f, %f), but laser data indicates free space.", map_x, map_y);
    }
  }
}

// wait for next laser scan data
laser_received = false;
while (!laser_received && rclcpp::ok()) {
  rclcpp::spin_some(navigator.get_node_base_interface());
}
}

rclcpp::shutdown(); // shutdown ROS
return 0;
}
