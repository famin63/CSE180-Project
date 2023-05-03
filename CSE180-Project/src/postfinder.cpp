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
                    update_index.push_back(i)
                }
            }
        }
    
//         uint counter = 0;
//         for (uint i = 0; i < msg->data.size(); i++) {
//             if (-1 != msg->data[i]) {
//                 counter++;
//             }
//         }
}

int main(int argc,char **argv) {
 
  rclcpp::init(argc,argv); // initialize ROS 
  nodeh = rclcpp::Node::make_shared("map_parser");
  auto sub = nodeh->create_subscription<nav_msgs::msg::OccupancyGrid>
       ("global_costmap/costmap", 1000, &mapCallback)
}
