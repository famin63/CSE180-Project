#include <rclcpp/rclcpp.hpp> 

#include <iostream>
#include <nav_msgs/msg/occupancy_grid.hpp>

rclcpp::Node::SharedPtr nodeh;

// callback function called every time a message is received from the
// topic "message"
void callback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
    nav_msgs::msg::OccupancyGrid MapProba;
    MapProba.data = msg->data;
  // process the message: just print it to the screen
  RCLCPP_INFO(nodeh->get_logger(),"size of map: %d",msg->data.size());
  RCLCPP_INFO(nodeh->get_logger(),"Resolution: %f",msg->info.resolution);
  RCLCPP_INFO(nodeh->get_logger(),"Resolution: %d",msg->info.height);
RCLCPP_INFO(nodeh->get_logger(),"Resolution: %d",msg->info.width);

uint counter =0;
for(uint i =0; i< msg->data.size(); i++){
    if(-1 !=msg->data[i]){
        counter++;
    }
MapProba.data[i] = msg->data[i];
    //RCLCPP_INFO(nodeh->get_logger(),"free space: %i",msg->data[i]);
}
RCLCPP_INFO(nodeh->get_logger(),"free space: %u",counter);
RCLCPP_INFO(nodeh->get_logger(),"free space: %d",MapProba.data.size());

}

int main(int argc,char **argv) {

  rclcpp::init(argc,argv); // initialize ROS subsystem
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr sub;
  nodeh = rclcpp::Node::make_shared("listener"); // create node instance
  // subscribe to topic "message" and register the callback function
  sub = nodeh->create_subscription<nav_msgs::msg::OccupancyGrid>
                                             ("map",10,&callback);
  rclcpp::spin(nodeh);  // wait for messages and process them
 
  rclcpp::shutdown();
  return 0;
  
}
