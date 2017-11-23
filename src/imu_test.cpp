#include <string>
#include <iostream>
#include <unistd.h>
#include <cstdio>
#include <ros/ros.h>                           // 包含ROS的头文件
#include <sensor_msgs/JointState.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Imu.h>
#include <boost/asio.hpp>                  //包含boost库函数
#include <boost/bind.hpp>
#include <math.h>
#include <pthread.h>
#include <serial/serial.h>
#include <std_msgs/String.h>
#include <std_msgs/Empty.h>

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

serial::Serial ser; 

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

void* thread (void *arg)
{
  while(ros::ok())
  { 
    if(ser.available())
    {     
        ser.read(read_buf,33); 
        for(int i=0;i<33-8;i++)
	{
	  if(read_buf[i] == read_flag)
	  {
	    switch(read_buf[i+1]){
	     case 0x51 :
		AX = (read_buf[i+3]<<8)|read_buf[i+2];
		AY = (read_buf[i+5]<<8)|read_buf[i+4];
		AZ = (read_buf[i+7]<<8)|read_buf[i+6];

		ax = (double)AX/32768*16*9.8;
		//ROS_INFO("ax: %lf", ax);
		ay = (double)AY/32768*16*9.8;
                az = (double)AZ/32768*16*9.8;
		i += 10;
		break;
	     case 0x52:
		WX = (read_buf[i+3]<<8)|read_buf[i+2];
		WY = (read_buf[i+5]<<8)|read_buf[i+4];
		WZ = (read_buf[i+7]<<8)|read_buf[i+6];

		wx = (double)WX/32768*2000/360*6.28;
		//ROS_INFO("wx: %lf", wx);
		wy = (double)WY/32768*2000/360*6.28;
                wz = (double)WZ/32768*2000/360*6.28;
		i += 10;
		break;
	     case 0x53:
	        ROLLX = (read_buf[i+3]<<8)|read_buf[i+2];
		PITCHY = (read_buf[i+5]<<8)|read_buf[i+4];
		YAWZ = (read_buf[i+7]<<8)|read_buf[i+6];

		Rollx = (double)ROLLX/32768*3.14;
		//ROS_INFO("Rollx: %lf", Rollx);
		Pitchy = (double)PITCHY/32768*3.14;
                Yawz = (double)YAWZ/32768*3.14;
		i += 10;
		//ROS_INFO("read_buf[32]: %d", read_buf[32]);
	    }
	  }
	}
     }
   }
}



int main(int argc, char** argv) {

    ros::init(argc, argv, "imu_data");       //初始化节点
    ros::NodeHandle n;
    ros::Publisher Imu_pub = n.advertise<sensor_msgs::Imu>("Imu", 20);
    
    pthread_t th;
    int ret;
    int *thread_ret = NULL;
    int arg = 10;
         
    try{
	ser.setPort("/dev/ttyUSB0");
	ser.setBaudrate(115200);
	serial::Timeout to = serial::Timeout::simpleTimeout(20);
	ser.setTimeout(to);
	ser.open();
    }
    catch(serial::IOException& e){
	ROS_ERROR("Unable to open port");
	return -1;
    }

    if(ser.isOpen()){
	ROS_INFO("Serial Port initialized");
    }
    else{
	return -1;
    }
 
    ret = pthread_create(&th,NULL,thread,&arg);
    if(ret !=0){
    ROS_INFO("Creat thred error!");
    return -1;
    }

    ros::Rate loop_rate(20);     //20hz
    ros::Time current_time;
    ros::Time last_time;
    current_time = ros::Time::now();                
    last_time = ros::Time::now();
    //int num=0; 
    while (ros::ok()) {
        
	last_time = current_time;
        current_time = ros::Time::now();  

        geometry_msgs::Quaternion Imu_quat;	
	Imu_quat = tf::createQuaternionMsgFromRollPitchYaw(Rollx,Pitchy,Yawz);

	sensor_msgs::Imu Imu;
	Imu.header.stamp = current_time;
	Imu.header.frame_id = "base_link";

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
        print_data();
        //ROS_INFO("number:%d",num);
        //num++;
        loop_rate.sleep();

    }

    pthread_join(th,(void**)&thread_ret);   //close pthread
    ROS_INFO("pthread_join");
    ser.close();
    return 0;
}

