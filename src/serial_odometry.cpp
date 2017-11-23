#include <string>
#include <iostream>
#include <ros/ros.h>                           // 包含ROS的头文件
#include <sensor_msgs/JointState.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <boost/asio.hpp>                  //包含boost库函数
#include <boost/bind.hpp>
#include <math.h>
//#include "std_msgs/String.h"              //ros定义的String数据类型


using namespace std;
using namespace boost::asio;           //定义一个命名空间，用于后面的读写操作

#define read_len   14
#define write_len  15
#define write_flag 0xfe
#define read_flag  0xfd
#define write_endflag 0x80
unsigned char read_buf[read_len];                      //定义字符串长度
unsigned char write_buf[write_len];
bool write_prepare = false;

double vel_x = 0.0;
double vel_y = 0.0;
double rot_w = 0.0;
double pos_x = 0.0;
double pos_y = 0.0;
double pos_w = 0.0;

typedef struct{
 union{
    float x;
    unsigned char data[4]; 
   }vel_x;
  union{
    float y;
    unsigned char data[4]; 
   }vel_y;
  union{
    float w;
    unsigned char data[4]; 
   }rot_w;

}CMD_VEL;

CMD_VEL cmd_vel;


typedef struct{
  union{
    float velocity_x;
    unsigned char data[4]; 
   }vel_x;
  union{
    float velocity_y;
    unsigned char data[4]; 
   }vel_y;
  union{
    float rotate_w;
    unsigned char data[4]; 
   }rot_w;
}STM32_ODOMETRY;

STM32_ODOMETRY stm32_odom;

void cmd_velCallback(const geometry_msgs::Twist &twist_vel){
  cmd_vel.vel_x.x = (float)twist_vel.linear.x;
  cmd_vel.vel_y.y = (float)twist_vel.linear.y;
  cmd_vel.rot_w.w = (float)twist_vel.angular.z;

  write_buf[0] = write_flag;
  write_buf[1] = 12;   //data length
  
   for(int i=0;i<4;i++)
   	{
      write_buf[i+2] = cmd_vel.vel_x.data[i];
	  }
   for(int i=4;i<8;i++)
    {
      write_buf[i+2] = cmd_vel.vel_y.data[i-4];
	  }
   for(int i=8;i<12;i++)
   	{
      write_buf[i+2] = cmd_vel.rot_w.data[i-8];
	  }
   
    write_buf[14] = write_endflag;
    write_prepare = true;
}


void print_data(void){
     //ROS_INFO("read_buf: %x",read_buf[0]);//打印
     //ROS_INFO("stm32_vel_x: %f", stm32_odom.vel_x.velocity_x);//打印
     //ROS_INFO("stm32_vel_y: %f", stm32_odom.vel_y.velocity_y);//打印
     //ROS_INFO("stm32_rot_w: %f", stm32_odom.rot_w.rotate_w);//打印
     ROS_INFO("vel_x: %lf", vel_x);//打印
     ROS_INFO("vel_y: %lf", vel_y);//打印
     ROS_INFO("rot_w: %lf", rot_w);//打印
	 
	 //ROS_INFO("delta_x: %lf", delta_x);//打印
	 //ROS_INFO("delta_y: %lf", delta_y);//打印
	 //ROS_INFO("delta_w: %lf", delta_w);//打印
   ROS_INFO("linear_x: %lf", cmd_vel.vel_x.x);//打印
   ROS_INFO("linear_y: %lf", cmd_vel.vel_y.y);//打印
   ROS_INFO("angular_w: %lf", cmd_vel.rot_w.w);//打印
   
}



int main(int argc, char** argv) {

    ros::init(argc, argv, "stm32_odom");       //初始化节点
    ros::NodeHandle n1;
    ros::NodeHandle n2;
    ros::Publisher odom_pub = n1.advertise<nav_msgs::Odometry>("odom", 10);
    ros::Subscriber cmd_vel_sub = n2.subscribe("cmd_vel", 10, cmd_velCallback);        
 
    tf::TransformBroadcaster broadcaster;
    geometry_msgs::TransformStamped odom_trans;
	  odom_trans.header.frame_id = "odom";
	  odom_trans.child_frame_id = "base_link";
    
    
	 
    io_service iosev;
    serial_port sp(iosev, "/dev/ttyUSB1");         //定义传输的串口
    sp.set_option(serial_port::baud_rate(115200));   
    sp.set_option(serial_port::flow_control(serial_port::flow_control::none));
    sp.set_option(serial_port::parity(serial_port::parity::none));
    sp.set_option(serial_port::stop_bits(serial_port::stop_bits::one));
    sp.set_option(serial_port::character_size(8));

    ros::Rate loop_rate(20);     //20hz
   
    ros::Time current_time;
	  ros::Time last_time;
    current_time = ros::Time::now();                
	  last_time = ros::Time::now();
    
    while (ros::ok()) {
     
     read (sp,buffer(read_buf));
               
     if(read_buf[0]==read_flag){
         for(int i=0;i<4;i++)
   	    {
          stm32_odom.vel_x.data[i] = read_buf[i+2] ;
	      }
         for(int i=4;i<8;i++)
     	  {
          stm32_odom.vel_y.data[i-4] = read_buf[i+2] ;
	      }
         for(int i=8;i<12;i++)
   	    {
          stm32_odom.rot_w.data[i-8] = read_buf[i+2] ;
	      }
        
      }

     last_time = current_time;
     current_time = ros::Time::now();  
     double dt = (current_time-last_time).toSec();

	   vel_x = (double)stm32_odom.vel_x.velocity_x;
	   vel_y = (double)stm32_odom.vel_y.velocity_y;
	   rot_w = (double)stm32_odom.rot_w.rotate_w;
     
     double delta_x = (vel_x * cos(pos_w) - vel_y * sin(pos_w)) * dt;
     double delta_y = (vel_x * sin(pos_w) + vel_y * cos(pos_w)) * dt;
     double delta_w = rot_w * dt;

     pos_x += delta_x;
     pos_y += delta_y;
     pos_w += delta_w;

     geometry_msgs::Quaternion odom_quat;	
		 odom_quat = tf::createQuaternionMsgFromRollPitchYaw(0,0,pos_w);
     odom_trans.header.stamp = current_time; 
		 odom_trans.transform.translation.x = pos_x; 
		 odom_trans.transform.translation.y = pos_y; 
		 odom_trans.transform.translation.z = 0.0;
		 odom_trans.transform.rotation = tf::createQuaternionMsgFromYaw(pos_w);

		 //filling the odometry
		 nav_msgs::Odometry odom;
		 odom.header.stamp = current_time;
		 odom.header.frame_id = "odom";
		 odom.child_frame_id = "base_link";

		 // position
		 odom.pose.pose.position.x = pos_x;
		 odom.pose.pose.position.y = pos_y;
		 odom.pose.pose.position.z = 0.0;
		 odom.pose.pose.orientation = odom_quat;

		//velocity
		 odom.twist.twist.linear.x = vel_x;
		 odom.twist.twist.linear.y = vel_y;
		 odom.twist.twist.linear.z = 0.0;
		 odom.twist.twist.angular.x = 0.0;
		 odom.twist.twist.angular.y = 0.0;
		 odom.twist.twist.angular.z = rot_w;


     ros::spinOnce();
     
     if(write_prepare == true){
       write(sp,buffer(write_buf,write_len));
       write_prepare = false;  
     }

     print_data();

     broadcaster.sendTransform(odom_trans);
		 odom_pub.publish(odom);
     
     loop_rate.sleep();

    }

    iosev.run(); 
    return 0;
}

