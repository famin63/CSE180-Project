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

// #include <rclcpp/rclcpp.hpp>
// #include <navigation/navigation.hpp>
// #include <sensor_msgs/msg/laser_scan.hpp>
// #include <geometry_msgs/msg/pose_stamped.hpp>
// #include <nav_msgs/msg/occupancy_grid.hpp>
// #include <tf2_ros/transform_listener.h>
// #include <tf2_geometry_msgs/tf2_geometry_msgs.h>

// void laserCallback(const sensor_msgs::msg::LaserScan::SharedPtr scan_msg, Navigator& navigator, nav_msgs::msg::OccupancyGrid& map) {
//   // Retrieve the transform of the laser with respect to the base_link
//   tf2_ros::Buffer tf_buffer;
//   tf2_ros::TransformListener tf_listener(tf_buffer);
//   geometry_msgs::msg::TransformStamped transformStamped;
//   try {
//     transformStamped = tf_buffer.lookupTransform("base_link", scan_msg->header.frame_id, scan_msg->header.stamp, rclcpp::Duration(1.0));
//   } catch (tf2::TransformException& ex) {
//     RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "%s", ex.what());
//     return;
//   }

//   // Compute the position of the laser in the map frame
//   geometry_msgs::msg::PoseStamped laser_pose;
//   laser_pose.pose.position.x = transformStamped.transform.translation.x;
//   laser_pose.pose.position.y = transformStamped.transform.translation.y;
//   laser_pose.pose.orientation = transformStamped.transform.rotation;
//   laser_pose.header.frame_id = "base_link";
//   laser_pose.header.stamp = scan_msg->header.stamp;

//   // Transform the laser pose to the map frame
//   geometry_msgs::msg::PoseStamped map_pose;
//   try {
//     tf_buffer.transform(laser_pose, map_pose, "map", rclcpp::Duration(1.0));
//   } catch (tf2::TransformException& ex) {
//     RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "%s", ex.what());
//     return;
//   }

//   // Compute the map cell corresponding to the laser pose
//   int x = (int)((map_pose.pose.position.x - map.info.origin.position.x) / map.info.resolution);
//   int y = (int)((map_pose.pose.position.y - map.info.origin.position.y) / map.info.resolution);

//   // Check if the map cell is occupied or free
//   if (x >= 0 && x < map.info.width && y >= 0 && y < map.info.height) {
//     int index = y * map.info.width + x;
//     if (map.data[index] > 65) {
//       RCLCPP_WARN(rclcpp::get_logger("rclcpp"), "Laser scan reports occupied space at (%d,%d) but map says it is free", x, y);
//     } else if (map.data[index] < 20) {
//       RCLCPP_WARN(rclcpp::get_logger("rclcpp"), "Laser scan reports free space at (%d,%d) but map says it is occupied", x, y);
//     }
//   }
// }
#include <rclcpp/rclcpp.hpp>
#include <navigation/navigation.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

void laserCallback(const sensor_msgs::msg::LaserScan::SharedPtr scan_msg, Navigator& navigator, nav_msgs::msg::OccupancyGrid& map, const geometry_msgs::msg::PoseStamped::SharedPtr amcl_pose) {
  // Retrieve the transform of the laser with respect to the base_link
  static tf2_ros::Buffer tf_buffer;
  static tf2_ros::TransformListener tf_listener(tf_buffer);

  geometry_msgs::msg::TransformStamped transformStamped;
  try {
    transformStamped = tf_buffer.lookupTransform("base_link", scan_msg->header.frame_id, scan_msg->header.stamp, rclcpp::Duration(1.0));
  } catch (tf2::TransformException& ex) {
    RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "%s", ex.what());
    return;
  }

  // Compute the position of the laser in the map frame
  geometry_msgs::msg::PoseStamped laser_pose;
  laser_pose.pose.position.x = transformStamped.transform.translation.x;
  laser_pose.pose.position.y = transformStamped.transform.translation.y;
  laser_pose.pose.orientation = transformStamped.transform.rotation;
  laser_pose.header.frame_id = "base_link";
  laser_pose.header.stamp = scan_msg->header.stamp;

  // Transform the laser pose to the map frame using the amcl_pose
  geometry_msgs::msg::PoseStamped map_pose;
  try {
    geometry_msgs::msg::TransformStamped amcl_transform = tf_buffer.lookupTransform("map", amcl_pose->header.frame_id, amcl_pose->header.stamp, rclcpp::Duration(1.0));
    tf2::doTransform(amcl_pose->pose, map_pose.pose, amcl_transform);
    map_pose.header = amcl_pose->header;
  } catch (tf2::TransformException& ex) {
    RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "%s", ex.what());
    return;
  }

  // Compute the map cell corresponding to the laser pose
  int x = (int)((map_pose.pose.position.x - map.info.origin.position.x) / map.info.resolution);
  int y = (int)((map_pose.pose.position.y - map.info.origin.position.y) / map.info.resolution);

  // Check if the map cell is occupied or free
  if (x >= 0 && x < map.info.width && y >= 0 && y < map.info.height) {
    int index = y * map.info.width + x;
    if (map.data[index] > 65) {
      RCLCPP_WARN(rclcpp::get_logger("rclcpp"), "Laser scan reports occupied space at (%d,%d) but map says it is free", x, y);
    } else if (map.data[index] < 20) {
      RCLCPP_WARN(rclcpp::get_logger("rclcpp"), "Laser scan reports free space at (%d,%d) but map says it is occupied", x, y);
    }
  }
}

int main(int argc, char **argv) {
  rclcpp::init(argc, argv); // initialize ROS
  Navigator navigator(true, false); // create node with debug info but not verbose

  // Subscribe to the map
  auto map_sub = navigator.create_subscription<nav_msgs::msg::OccupancyGrid>(
    "/map", 10,
    [&](const nav_msgs::msg::OccupancyGrid::SharedPtr map_msg) {
      // Store the map
      navigator.SetMap(map_msg);
    });

  // Subscribe to the laser scan
  auto laser_sub = navigator.create_subscription<sensor_msgs::msg::LaserScan>(
    "/scan", 10,
    [&](const sensor_msgs::msg::LaserScan::SharedPtr scan_msg) {
      // Call laser callback function
      laserCallback(scan_msg, navigator, navigator.GetMap(), amcl_pose);
    });

  // Spin the node
  rclcpp::spin(navigator.get_node_base_interface());

  // Shut down ROS
  rclcpp::shutdown();

  return 0;
}
