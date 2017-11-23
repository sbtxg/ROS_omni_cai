#include <string>
#include <iostream>
#include <ros/ros.h>                           // 包含ROS的头文件
#include <sensor_msgs/JointState.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Imu.h>
#include <boost/asio.hpp>                  //包含boost库函数
#include <boost/bind.hpp>
#include <math.h>
//#include "std_msgs/String.h"              //ros定义的String数据类型

using namespace std;
using namespace boost::asio;           //定义一个命名空间，用于后面的读写操作

#define read_len  33
#define read_flag  0x55
unsigned char read_buf[read_len];                      //定义字符串长度
bool write_prepare = false;

short AX = 0;
short AY = 0;
short AZ = 0;
short WX = 0;
short WY = 0;
short WZ = 0;
short ROLLX = 0;
short PITCHY = 0;
short YAWZ = 0;

double ax = 0;
double ay = 0;
double az = 0;
double wx = 0;
double wy = 0;
double wz = 0;
double Rollx = 0;
double Pitchy = 0;
double Yawz = 0;

void print_data(void){ 
     
   ROS_INFO("ax: %lf", ax);//打印
   ROS_INFO("ay: %lf", ay);//打印
   ROS_INFO("az: %lf", az);//打印
	 
   ROS_INFO("wx: %lf", wx);//打印
   ROS_INFO("wy: %lf", wy);//打印
   ROS_INFO("wz: %lf", wz);//打印

   ROS_INFO("Rollx: %lf", Rollx);//打印
   ROS_INFO("Pitchy: %lf", Pitchy);//打印
   ROS_INFO("Yawz: %lf", Yawz);//打印
   
}


int main(int argc, char** argv) {

    ros::init(argc, argv, "imu_data");       //初始化节点
    ros::NodeHandle n;
    ros::Publisher Imu_pub = n.advertise<sensor_msgs::Imu>("Imu", 20);
             
    io_service iosev;
    serial_port sp(iosev, "/dev/ttyUSB0");         //定义传输的串口
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

     if(read_buf[0] == read_flag){
        AX = (read_buf[3]<<8)|read_buf[2];
				AY = (read_buf[5]<<8)|read_buf[4];
				AZ = (read_buf[7]<<8)|read_buf[6];

				ax = (double)AX/32768*16*9.8;
				ROS_INFO("ax: %lf", ax);
				ay = (double)AY/32768*16*9.8;
        az = (double)AZ/32768*16*9.8;
     }
		 else if (read_buf[0] == 0x51){
        AX = (read_buf[2]<<8)|read_buf[1];
				AY = (read_buf[4]<<8)|read_buf[3];
				AZ = (read_buf[6]<<8)|read_buf[5];

				ax = (double)AX/32768*16*9.8;
				ROS_INFO("ax: %lf", ax);
				ay = (double)AY/32768*16*9.8;
        az = (double)AZ/32768*16*9.8;
     }

		 if(read_buf[11] == read_flag){
				WX = (read_buf[14]<<8)|read_buf[13];
				WY = (read_buf[16]<<8)|read_buf[15];
				WZ = (read_buf[18]<<8)|read_buf[17];

				wx = (double)WX/32768*2000/360*6.28;
				ROS_INFO("wx: %lf", wx);
				wy = (double)WY/32768*2000/360*6.28;
        wz = (double)WZ/32768*2000/360*6.28;
     }
		  else if (read_buf[11] == 0x52){
        WX = (read_buf[13]<<8)|read_buf[12];
				WY = (read_buf[15]<<8)|read_buf[14];
				WZ = (read_buf[17]<<8)|read_buf[17];

				wx = (double)WX/32768*2000/360*6.28;
				ROS_INFO("wx: %lf", wx);
				wy = (double)WY/32768*2000/360*6.28;
        wz = (double)WZ/32768*2000/360*6.28;
     }

     if(read_buf[22] == read_flag){
				ROLLX = (read_buf[25]<<8)|read_buf[24];
				PITCHY = (read_buf[27]<<8)|read_buf[26];
				YAWZ = (read_buf[29]<<8)|read_buf[28];

				Rollx = (double)ROLLX/32768*3.14;
				ROS_INFO("Rollx: %lf", Rollx);
				Pitchy = (double)PITCHY/32768*3.14;
        Yawz = (double)YAWZ/32768*3.14;
				ROS_INFO("read_buf[32]: %d", read_buf[32]);
     }
		  else if (read_buf[22] == 0x53){
        ROLLX = (read_buf[24]<<8)|read_buf[23];
				PITCHY = (read_buf[26]<<8)|read_buf[25];
				YAWZ = (read_buf[28]<<8)|read_buf[27];

				Rollx = (double)ROLLX/32768*3.14;
				ROS_INFO("Rollx: %lf", Rollx);
				Pitchy = (double)PITCHY/32768*3.14;
        Yawz = (double)YAWZ/32768*3.14;
     }

     last_time = current_time;
     current_time = ros::Time::now();  

     geometry_msgs::Quaternion Imu_quat;	
		 Imu_quat = tf::createQuaternionMsgFromRollPitchYaw(0,0,Yawz);

		 sensor_msgs::Imu Imu;
		 Imu.header.stamp = current_time;
		 Imu.header.frame_id = "base_Imu";

		 // Imu-angular_velocity
		 Imu.angular_velocity.x = wx;
		 Imu.angular_velocity.y = wy;
		 Imu.angular_velocity.z = wz;
		 Imu.orientation = Imu_quat;

		 //Imu_acceleration
		 Imu.linear_acceleration.x = ax;
		 Imu.linear_acceleration.y = ay;
		 Imu.linear_acceleration.z = az;
		


     ros::spinOnce();  

		 Imu_pub.publish(Imu);
     //print_data();

     loop_rate.sleep();

    }

    iosev.run(); 
    return 0;
}

