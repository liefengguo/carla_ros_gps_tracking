#include <ros/ros.h>
#include <std_msgs/Header.h>
#include <nav_msgs/Odometry.h>
#include <iostream>
#include <string>
#include <sstream>
#include <memory>
#include <std_msgs/String.h>

#include <tf2/convert.h>
#include <tf/transform_broadcaster.h>
#include <cmath>
#include <unistd.h>

using namespace std;
FILE *fp;
string file_path_ = "/home/chen/ros_work/lidar/src/control_publisher/";
string file_name_ = "path.txt";


void callbackPosition(const nav_msgs::Odometry& msgs)
{
    double position_x = msgs.pose.pose.position.x;
    double position_y = msgs.pose.pose.position.y;
    double position_z = msgs.pose.pose.position.z;
    double orientation_x = msgs.pose.pose.orientation.x;
    double orientation_y = msgs.pose.pose.orientation.y;
    double orientation_z = msgs.pose.pose.orientation.z;
    double orientation_w = msgs.pose.pose.orientation.w;
    double yaw = tf::getYaw(msgs.pose.pose.orientation);
    
    

    if(fp == NULL)
    {
	ROS_INFO("open record data file %s failed !!!",(file_path_+file_name_).c_str());
    }
    if(fp != NULL)
    {
        fprintf(fp,"%f %f %f %f %f %f %f \n",position_x,position_y,position_z,orientation_x,orientation_y ,orientation_z, orientation_w);
        fflush(fp);
        
    }


    ROS_INFO("Position x %f", position_x);
    ROS_INFO("Position y %f", position_y);
    ROS_INFO("Position z %f", position_z);
    ROS_INFO("Position yaw %f", yaw);
}


int main(int argc, char** argv)
{
    ros::init(argc, argv, "record");
    ros::NodeHandle nh;
    nh.param<std::string>("gps_record/file_path", file_path_, "/home/user/position.txt");

    

    ros::Subscriber position_sub = nh.subscribe("/fixposition/odometry", 1, callbackPosition);
    fp = fopen((file_path_+file_name_).c_str(),"w");
    ros::Rate loop_rate(1);
    

   
    while(ros::ok())
    {

        ros::spinOnce();
        loop_rate.sleep();
    }
   //fflush(fp);    
   //fclose(fp);
   ros::spin();

    return 0;
}

