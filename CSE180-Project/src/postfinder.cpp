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
int main(int argc,char **argv) {
 
  rclcpp::init(argc,argv); // initialize ROS 
  nodeh = rclcpp::Node::make_shared("map_parser");
  auto sub = nodeh->create_subscription<nav_msgs::msg::OccupancyGrid>
       ("global_costmap/costmap", 1000, &mapCallback)
}
