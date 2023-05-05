 #include <rclcpp/rclcpp.hpp> 
 #include <navigation/navigation.hpp>
 #include <iostream>
 #include <sensor_msgs/msg/laser_scan.hpp>
 #include <std_msgs/msg/float32.hpp>

 int main(int argc,char **argv) {

   rclcpp::init(argc,argv); // initialize ROS 
   Navigator navigator(true,false); // create node with debug info but not verbose

   // first: it is mandatory to initialize the pose of the robot
   geometry_msgs::msg::Pose::SharedPtr init = std::make_shared<geometry_msgs::msg::Pose>();

   init->position.x = -2;
   init->position.y = -0.5;
   init->orientation.w = 1;
   navigator.SetInitialPose(init);

   // wait for navigation stack to become operationale
   navigator.WaitUntilNav2Active();

   auto goal_pose = std::make_shared<geometry_msgs::msg::Pose>();
   goal_pose->position.x = 2.0;
   goal_pose->position.y = 1.0;
   goal_pose->orientation.w = 0.707;
   goal_pose->orientation.z = 0.707;

   auto path = navigator.GetPath(goal_pose);

   while (rclcpp.ok()) {}

   rclcpp::shutdown(); // shutdown ROS
   return 0;
 }
