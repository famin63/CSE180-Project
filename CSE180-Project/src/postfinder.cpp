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
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <my_custom_msgs/msg/extra_post.hpp> // assuming custom message type is defined in my_custom_msgs package
#include <iostream>

// global variables for map and laser scan data
nav_msgs::msg::OccupancyGrid::SharedPtr map_data;
sensor_msgs::msg::LaserScan::SharedPtr laser_data;

// callback function for the map data
void mapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr map) {
  // store the map data for later use
  map_data = map;
}

// callback function for the laser scan data
void laserCallback(const sensor_msgs::msg::LaserScan::SharedPtr scan_data) {
  // store the laser scan data for later use
  laser_data = scan_data;

  // process the laser scan data here to find the extra post
  // (assuming the map data has already been received)
  if (map_data) {
    float map_resolution = map_data->info.resolution;
    float map_origin_x = map_data->info.origin.position.x;
    float map_origin_y = map_data->info.origin.position.y;
    float max_range = scan_data->range_max;
    float angle_min = scan_data->angle_min;
    float angle_increment = scan_data->angle_increment;

    // loop through all laser scan ranges
    for (size_t i = 0; i < scan_data->ranges.size(); i++) {
      float range = scan_data->ranges[i];
      if (range >= max_range) {
        // range is too far, skip it
        continue;
      }

      float angle = angle_min + i * angle_increment;
      float x = scan_data->ranges[i] * cos(angle);
      float y = scan_data->ranges[i] * sin(angle);

      // convert laser scan data to map coordinates
      int map_x = (int) round((x + map_origin_x) / map_resolution);
      int map_y = (int) round((y + map_origin_y) / map_resolution);

      // check if there's an obstacle at this location in the map
      if (map_x < 0 || map_x >= map_data->info.width ||
          map_y < 0 || map_y >= map_data->info.height) {
        // map coordinates are out of bounds, skip them
        continue;
      }
      int map_index = map_x + map_y * map_data->info.width;
      int map_value = map_data->data[map_index];
      if (map_value > 0) {
        // found an obstacle, check if it's an extra post
        if (map_value == 100) {
          // found an extra post, publish its position
          rclcpp::NodeOptions options;
          auto node = std::make_shared<rclcpp::Node>("extra_post_publisher", options);
          auto publisher = node->create_publisher<my_custom_msgs::msg::ExtraPost>("extra_post", 10);

          float extra_post_x = map_x * map_resolution - map_origin_x;
          float extra_post_y = map_y * map_resolution - map_origin_y;
          my_custom_msgs::msg::ExtraPost extra_post;
          extra_post.position.x = extra_post_x;
          extra_post.position.y = extra_post_y;
          publisher->publish(extra_post);

          RCLCPP_INFO(node->get_logger(), "Found extra post at (%f, %f)", extra_post_x, extra_post_y);
        }
      }
    }
  }
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

