#include <sensor_msgs/msg/laser_scan.hpp>
#include <std_msgs/msg/float32.hpp>

nav_msgs::msg::OccupancyGrid::SharedPtr original_map;
bool original = true;

rclcpp::Node::SharedPtr nodeh;

void mapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg){
        RCLCPP_INFO(nodeh->get_logger(),"Size of Map: %d", msg->data.size());
        RCLCPP_INFO(nodeh->get_logger(), "Resolution: %f", msg->info.resolution());
        RCLCPP_INFO(nodeh->get_logger(), "Width: %d", msg->info.width());
        RCLCPP_INFO(nodeh->get_logger(), "Height: %d", msg->info.height());
        std::vector<int> update_index;
        int filter = 95;

        uint counter = 0;
        for (uint i = 0; i < msg->data.size(); i++) {
            if (-1 != msg->data[i]) {
                counter++;
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

int main(int argc,char **argv) {
 
  rclcpp::init(argc,argv); // initialize ROS 
  nodeh = rclcpp::Node::make_shared("map_parser");
  auto sub = nodeh->create_subscription<nav_msgs::msg::OccupancyGrid>
       ("global_costmap/costmap", 1000, &mapCallback)
}
