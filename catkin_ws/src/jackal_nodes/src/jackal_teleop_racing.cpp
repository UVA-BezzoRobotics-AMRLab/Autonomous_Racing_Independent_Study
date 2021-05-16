#include "ros/ros.h"
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/Imu.h>
#include <nav_msgs/Odometry.h>
#include <keyboard/Key.h>
#include <stdlib.h>
#include <iostream>
#include <fstream>
#include <cmath>
using namespace std;

int key;
float x_acc;
float x_vel;
float z_angular_vel;
float x_pos = 0;
float y_pos = 0;


void keyCallback(const keyboard::Key::ConstPtr& data)
{
  ROS_INFO("Key = %i", data->code);//print out the keycode
	key = data->code;
}
void gpsCallback(const geometry_msgs::Pose::ConstPtr& data)
{
	x_pos = data->position.x;
	y_pos = data->position.y;
}
void odomCallback(const nav_msgs::Odometry::ConstPtr& data)
{
	x_vel = data->twist.twist.linear.x;
	z_angular_vel = data->twist.twist.angular.z;
}
int main(int argc, char **argv)
{
	ros::init(argc, argv, "jackal_teleop_racing");
  ros::NodeHandle nh;
  ros::Publisher vel_pub;
  vel_pub = nh.advertise<geometry_msgs::Twist>("/jackal1/jackal_velocity_controller/cmd_vel", 1, true);
	ros::Subscriber gpssub = nh.subscribe("/jackal0/global_pos", 10, gpsCallback);
  ros::Subscriber keysub = nh.subscribe("/keyboard/keydown", 10, keyCallback);
  ros::Rate loop_rate(10);
	int count_file = 0;
	//ofstream MyFile("track_points_4.txt");
	geometry_msgs::Twist vel;
  vel.linear.x = 0;//linear velocity(m/s)
  vel.angular.z = 0;//angular velocity(rad/s)
  while (ros::ok())
  {
    vel_pub.publish(vel);
    ros::spinOnce();
    loop_rate.sleep();
		if(true){
		//MyFile << x_pos << " " << y_pos << "\n";
		count_file=0;
		}
		else
		{
			count_file++;
		}

    if(key==99) // C: End Program
    {
    	break;
    }
	  if(key==119) // W: Move Forward
	  {
	  	vel.linear.x = 1.0; //linear velocity(m/s)
	  	vel.angular.z = 0.0; //angular velocity(rad/s)
	  }
	  if(key==115) // S: Move Backward
	  {
	  	vel.linear.x = -1.0; //linear velocity(m/s)
	  	vel.angular.z = 0.0; //angular velocity(rad/s)
	  }
	  if(key==97) // A: Turn Left
	  {
	  	vel.linear.x = 0.8; //linear velocity(m/s)
	  	vel.angular.z = 0.8; //angular velocity(rad/s)
	  }    
	  if(key==100) // D: Turn Right
	  {
	  	vel.linear.x = 0.8; //linear velocity(m/s)
	  	vel.angular.z = -0.8; //angular velocity(rad/s)
	  }
	  if(key==120) // X: Stop moving
	  {
	  	vel.linear.x = 0.0; //linear velocity(m/s)
	 	 vel.angular.z =0.0; //angular velocity(rad/s)
	  }
 }
	//MyFile.close();
  return 0;
}
