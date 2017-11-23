#include <ros/ros.h>
#include <tf/transform_broadcaster.h>

int main(int argc, char** argv){
  ros::init(argc, argv, "robot_tf_publisher");
  ros::NodeHandle n;

  ros::Rate r(20);
 
 // tf::TransformBroadcaster broadcaster;
  tf::TransformBroadcaster broadcaster_laser;
  tf::TransformBroadcaster broadcaster_imu;
  while(n.ok()){
   /* broadcaster.sendTransform(
      tf::StampedTransform(
        tf::Transform(tf::Quaternion(0, 0, 0, 1), tf::Vector3(0.0, 0.0, 0.0)),
        ros::Time::now(),"base_footprint", "base_link"));*/
   
    broadcaster_laser.sendTransform(  
    tf::StampedTransform(
        tf::Transform(tf::Quaternion(0, 0, 0, 1), tf::Vector3(0.0, 0.0, 0.28)),
        ros::Time::now(),"base_link", "base_laser"));

    broadcaster_imu.sendTransform(  
    tf::StampedTransform(
        tf::Transform(tf::Quaternion(0, 0, 0, 1), tf::Vector3(0.0, 0.0, 0.28)),
        ros::Time::now(),"base_link", "base_imu"));
   
    r.sleep();
  }
}
