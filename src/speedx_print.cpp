#include <string>
#include <iostream>
#include <ros/ros.h>                           // 包含ROS的头文件
#include <sensor_msgs/JointState.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <boost/asio.hpp>                  //包含boost库函数
#include <boost/bind.hpp>
#include <math.h>
#include <std_msgs/Float64.h>

void speedxcallback(const std_msgs::Float64& speedmsg)
{
  ROS_INFO("speed_x: %lf",speedmsg.data);
}

int main(int argc, char** argv){
 
  ros::init(argc,argv,"speedx_print");
  ros::NodeHandle n;
  
  ros::Subscriber sub = n.subscribe("speedx",100,speedxcallback);
  ros::spin();
  return 0;



}
