#include <ros/ros.h>
#include <algorithm>
#include <cassert>
#include <cmath>
#include <fstream>
#include <iostream>
#include <string>
#include <vector>

#include <geometry_msgs/Twist.h>
#include <nav_msgs/Path.h>
#include <tf/tf.h>
// #include <tf/transform_broadcaster.h>
#include<tf2_ros/transform_listener.h>
#include<tf2_geometry_msgs/tf2_geometry_msgs.h>
#include "geometry_msgs/PoseStamped.h"

#include "../include/loc_ang.h"
#include "../include/gnss_coordinate_convert.h"
#include "visualization_msgs/Marker.h"

#include <tf2/convert.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/Header.h>



#define PREVIEW_DIS 3 //预瞄距离

#define Ld 1.23  //轴距

// #define car_vel 0.3

using namespace std;

ros::Publisher purepersuit_;
ros::Publisher path_pub_;
nav_msgs::Path path;
ros::Publisher marker_pub;
ros::Publisher marker_car_pub;
ros::Publisher marker_pub_street;
ros::Publisher marker_pub_fuzhu;
ros::Publisher marker_pub_ext_car;
visualization_msgs::Marker marker;
visualization_msgs::Marker marker_car;
visualization_msgs::Marker marker_streer;
visualization_msgs::Marker marker_fuzhu;
visualization_msgs::Marker marker_ext_car;
std::ofstream logfile;
bool log_flag; 
double car_vel = 0.3;
double preview_dis = 0;
double k = 0.1;


// 计算四元数转换到欧拉角
std::array<float, 3> calQuaternionToEuler(const float x, const float y,
                                          const float z, const float w) {
  std::array<float, 3> calRPY = {(0, 0, 0)};
  // roll = atan2(2(wx+yz),1-2(x*x+y*y))
  calRPY[0] = atan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y));
  // pitch = arcsin(2(wy-zx))
  calRPY[1] = asin(2 * (w * y - z * x));
  // yaw = atan2(2(wx+yz),1-2(y*y+z*z))
  calRPY[2] = atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z));

  return calRPY;
}

vector<double> r_x_;
vector<double> r_y_;

int pointNum = 0;  //保存路径点的个数
int targetIndex = pointNum - 1;

vector<float> bestPoints_ = {0.0};

//计算发送给模型车的转角
void poseCallback(const nav_msgs::Odometry &currentWaypoint) {
  double x ,y,z;
  // GaussProjCal(currentWaypoint.pose.pose.position.x,currentWaypoint.pose.pose.position.y,&x,&y);
  ecefToEnu(currentWaypoint.pose.pose.position.x,currentWaypoint.pose.pose.position.y,currentWaypoint.pose.pose.position.z,&x,&y,&z);
  // cout<<"时间："<< currentWaypoint.header.stamp<<endl;
  tf2_ros::Buffer tfBuffer;
  tf2_ros::TransformListener tfListener(tfBuffer);

  geometry_msgs::TransformStamped transformStamped;
  transformStamped = tfBuffer.lookupTransform("FP_ENU", "ECEF", ros::Time(0),ros::Duration(1));
  geometry_msgs::QuaternionStamped ecefOrientation;
  ecefOrientation.quaternion.w = currentWaypoint.pose.pose.orientation.w;
  ecefOrientation.quaternion.x = currentWaypoint.pose.pose.orientation.x;
  ecefOrientation.quaternion.y = currentWaypoint.pose.pose.orientation.y;
  ecefOrientation.quaternion.z = currentWaypoint.pose.pose.orientation.z;
  geometry_msgs::QuaternionStamped enuOrientation; 
  tf2::doTransform(ecefOrientation, enuOrientation, transformStamped);

  auto currentPositionX = x;
  auto currentPositionY = y;
  auto currentPositionZ = z;
  auto currentQuaternionX = enuOrientation.quaternion.x;
  auto currentQuaternionY = enuOrientation.quaternion.y;
  auto currentQuaternionZ = enuOrientation.quaternion.z;
  auto currentQuaternionW = enuOrientation.quaternion.w;
  auto currentPositionYaw = tf::getYaw(enuOrientation.quaternion) ;
  std::array<float, 3> calRPY = calQuaternionToEuler(currentQuaternionX, currentQuaternionY,currentQuaternionZ, currentQuaternionW);

  /***********通过计算当前坐标和目标轨迹距离，找到一个最小距离的索引号***************************************************************************************/
  int index;
  vector<double> bestPoints_;
  for (int i = 0; i < pointNum; i++) {
    // float lad = 0.0;
    double path_x = r_x_[i];
    double path_y = r_y_[i];
    // 遍历所有路径点和当前位置的距离，保存到数组中
    double lad = sqrt(pow(path_x - currentPositionX, 2) +
                     pow(path_y - currentPositionY, 2));

    bestPoints_.push_back(lad);
  }
  // 找到数组中最小横向距离
  auto smallest = min_element(bestPoints_.begin(), bestPoints_.end());
  // 找到最小横向距离的索引位置
  index = distance(bestPoints_.begin(), smallest);
  int temp_index;
  for (int i = index; i < pointNum; i++) {
    //遍历路径点和预瞄点的距离，从最小横向位置的索引开始
    float dis =
        sqrt(pow(r_y_[index] - r_y_[i], 2) + pow(r_x_[index] - r_x_[i], 2));
    //判断跟预瞄点的距离
    preview_dis = k * car_vel + PREVIEW_DIS;
    if (dis < preview_dis) {
      temp_index = i;
    } else {
      break;
    }
  }
  index = temp_index;
  /**************************************************************************************************/
  float alpha_two_point = atan2(r_y_[index] - currentPositionY, r_x_[index] - currentPositionX); 
  float alpha = alpha_two_point - currentPositionYaw;
  alpha = (alpha > M_PI) ? (alpha - 2 * M_PI) : (alpha < -M_PI) ? (alpha + 2 * M_PI) : alpha;

  // 当前点和目标点的距离Id
  float dl = sqrt(pow(r_y_[index] - currentPositionY, 2) +
                  pow(r_x_[index] - currentPositionX, 2));
  
  // 发布小车运动指令及运动轨迹
  if (dl > 0.15) {
    car_vel = 0.3;
    float theta = atan(2 * Ld * sin(alpha) / dl);
    double degree = theta * 180 / M_PI;
    degree = max(min(38.6, degree), -38.6);
    double theta_send = -degree * 0.5 / 38.6;
    car_vel *= std::abs(cos(theta_send));
    double curYaw_deg = currentPositionYaw * 180 / M_PI;

    cout<<"alpha: "<<alpha* 180 /M_PI<<" degree"<<-degree<<" dis: "<<dl<<"  index:"<<index<<"x: "<<r_x_[index]<<"y: "<<r_y_[index]<<endl;
    cout<<"curr_yaw"<< curYaw_deg <<" alpha_two_point:" << alpha_two_point * 180 / M_PI<<endl;
    if(log_flag){
        if (!logfile.is_open()) {
            // 处理无法打开日志文件的情况
            std::cout<<"no open!!!!"<<std::endl;
            // return;
    }
        logfile << theta_send <<  std::endl;
    }
    geometry_msgs::Twist vel_msg;
    vel_msg.linear.x = 0.3;
    vel_msg.angular.z = theta_send;
    purepersuit_.publish(vel_msg);
    // 发布小车运动轨迹
    geometry_msgs::PoseStamped this_pose_stamped;
    this_pose_stamped.pose.position.x = currentPositionX;
    this_pose_stamped.pose.position.y = currentPositionY;

    geometry_msgs::Quaternion goal_quat = tf::createQuaternionMsgFromYaw(theta);
    this_pose_stamped.pose.orientation.x = currentQuaternionX;
    this_pose_stamped.pose.orientation.y = currentQuaternionY;
    this_pose_stamped.pose.orientation.z = currentQuaternionZ;
    this_pose_stamped.pose.orientation.w = currentQuaternionW;

    this_pose_stamped.header.stamp = ros::Time::now();

    this_pose_stamped.header.frame_id = "map";
    path.poses.push_back(this_pose_stamped);

    marker.header.frame_id = "map";
    marker.header.stamp = ros::Time::now();

    marker.pose.position.x = r_x_[index];
    marker.pose.position.y = r_y_[index];
    marker_pub.publish(marker);

    marker_car.header.stamp = ros::Time::now();
    cout<<"车："<<currentPositionX<<"  " << currentPositionY<< endl;
    marker_car.pose = this_pose_stamped.pose;
    // marker_car
    marker_car.pose.orientation = goal_quat;
    marker_car_pub.publish(marker_car);

    marker_streer.points.clear();
    marker_streer.header.stamp = ros::Time::now();
    marker_streer.points.push_back(this_pose_stamped.pose.position);
    geometry_msgs::Point end_point;
    end_point.x = currentPositionX + 10 * sin(theta);
    end_point.y = currentPositionY + 10 * cos(theta);    
    end_point.z = 0.0;
    marker_streer.points.push_back(end_point);
    // 发布Marker消息
    marker_pub_street.publish(marker_streer);


    marker_fuzhu.points.clear();
    marker_fuzhu.header.stamp = ros::Time::now();
    marker_fuzhu.points.push_back(this_pose_stamped.pose.position);
    geometry_msgs::Point end_point_fuzhu;
    end_point_fuzhu.x = r_x_[index]+ 8;
    end_point_fuzhu.y = currentPositionY;    
    end_point_fuzhu.z = 0.0;
    marker_fuzhu.points.push_back(end_point_fuzhu);
    marker_pub_fuzhu.publish(marker_fuzhu);

    double currentYaw = (currentPositionYaw > M_PI) ? 
          (currentPositionYaw - 2 * M_PI) : (currentPositionYaw < -M_PI) ? (currentPositionYaw + 2 * M_PI) : currentPositionYaw;
    double ext_car_y = abs(tan(curYaw_deg)) * abs(r_x_[index] - currentPositionX) + currentPositionY;
    marker_ext_car.points.clear();
    marker_ext_car.header.stamp = ros::Time::now();
    marker_ext_car.points.push_back(this_pose_stamped.pose.position);
    geometry_msgs::Point end_point_ext_car;
    int lenth = 8;
    end_point_ext_car.x = currentPositionX + lenth* sin(currentPositionYaw );
    end_point_ext_car.y = currentPositionY + lenth* cos(currentPositionYaw );    
    end_point_ext_car.z = 0.0;
    marker_ext_car.points.push_back(end_point_ext_car);
    marker_pub_ext_car.publish(marker_ext_car);

  } else {
    cout<<"NO: "<<dl<<endl;
    geometry_msgs::Twist vel_msg;
    vel_msg.linear.x = 0;
    vel_msg.angular.z = 0;
    purepersuit_.publish(vel_msg);
  }
  path_pub_.publish(path);
}


void pointCallback(const nav_msgs::Path &msg) {
  pointNum = msg.poses.size();

  for (int i = 0; i < pointNum; i++) {
    r_x_.push_back(msg.poses[i].pose.position.x);
    r_y_.push_back(msg.poses[i].pose.position.y);
  }
}

int main(int argc, char **argv) {
  //创建节点
  ros::init(argc, argv, "pure_pursuit");

  //创建节点句柄
  ros::NodeHandle n;
  //创建Publisher，发送经过pure_pursuit计算后的转角及速度
  purepersuit_ = n.advertise<geometry_msgs::Twist>("/cmd_vel", 20);
  marker_pub = n.advertise<visualization_msgs::Marker>("coordinate_marker", 10);
  marker_car_pub= n.advertise<visualization_msgs::Marker>("car_marker", 1);
  path_pub_ = n.advertise<nav_msgs::Path>("rvizpath", 100, true);
  marker_pub_street = n.advertise<visualization_msgs::Marker>("turn_angle_marker", 1);
  marker_pub_fuzhu = n.advertise<visualization_msgs::Marker>("fuzhu_marker", 1);
  marker_pub_ext_car = n.advertise<visualization_msgs::Marker>("ext_car_marker", 1);
  n.param<bool>("purepursuit_/log_flag", log_flag, 1);

  if (log_flag){
    std::string path = "/home/glf/log/";
    std::stringstream  filename;
    filename <<path<< "cmd_z"  << ".txt";
    std::cout<<"file path :"<<filename.str()<<std::endl;
    // 打开日志文件
    logfile.open(filename.str(), std::ios::app);
  }
  // ros::Rate loop_rate(10);
    marker.ns = "coordinates";
    marker.id = 0;
    marker.type = visualization_msgs::Marker::SPHERE;
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose.position.z = 0.0;
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = 0.5;
    marker.scale.y = 0.5;
    marker.scale.z = 0.5;
    marker.color.a = 1.0;
    marker.color.r = 0.4;
    marker.color.g = 0.7;
    marker.color.b = 1.0;

    marker_car.header.frame_id = "map";
    marker_car.ns = "car";
    marker_car.id = 1;
    marker_car.action = visualization_msgs::Marker::ADD;
    marker_car.type = visualization_msgs::Marker::CUBE;
    marker_car.scale.x = 0.5;  // 车辆模型的缩放因子
    marker_car.scale.y = 0.2;
    marker_car.scale.z = 0.1;
    marker_car.color.a = 1.0;  // 不透明度
    marker_car.color.r = 1.0;  // 颜色为红色
    marker_car.color.g = 0.3;
    marker_car.color.b = 0.2;

    marker_streer.header.frame_id = "map";
    marker_streer.header.stamp = ros::Time::now();
    marker_streer.ns = "steering";
    marker_streer.action = visualization_msgs::Marker::ADD;
    marker_streer.type = visualization_msgs::Marker::LINE_STRIP;
    marker_streer.scale.x = 0.1;  // 线的宽度
    marker_streer.color.a = 1.0;  // 不透明度
    marker_streer.color.r = 1.0;  // 颜色为紫色
    marker_streer.color.g = 0.0;
    marker_streer.color.b = 1.0;

    marker_fuzhu.header.frame_id = "map";
    marker_fuzhu.header.stamp = ros::Time::now();
    marker_fuzhu.ns = "fuzhu";
    marker_fuzhu.action = visualization_msgs::Marker::ADD;
    marker_fuzhu.type = visualization_msgs::Marker::LINE_STRIP;
    marker_fuzhu.scale.x = 0.1;  // 线的宽度
    marker_fuzhu.color.a = 1.0;  // 不透明度
    marker_fuzhu.color.r = 1.0;  // 颜色为红色
    marker_fuzhu.color.g = 0.0;
    marker_fuzhu.color.b = 0.0;

    marker_ext_car.header.frame_id = "map";
    marker_ext_car.header.stamp = ros::Time::now();
    marker_ext_car.ns = "ext_car";
    marker_ext_car.action = visualization_msgs::Marker::ADD;
    marker_ext_car.type = visualization_msgs::Marker::LINE_STRIP;
    marker_ext_car.scale.x = 0.1;  // 线的宽度
    marker_ext_car.color.a = 1.0;  // 不透明度
    marker_ext_car.color.r = 0.0;  // 颜色为绿色
    marker_ext_car.color.g = 1.0;
    marker_ext_car.color.b = 0.0;

  path.header.frame_id = "map";
  // 设置时间戳
  path.header.stamp = ros::Time::now();
  geometry_msgs::PoseStamped pose;
  pose.header.stamp = ros::Time::now();
  // 设置参考系
  pose.header.frame_id = "map";

  ros::Subscriber splinePath = n.subscribe("/splinepoints", 20, pointCallback);
  // ros::Subscriber carVel = n.subscribe("/fixposition/speed", 20, velocityCall);
  ros::Subscriber carPose = n.subscribe("/fixposition/odometry", 20, poseCallback);
  ros::spin();
  if(log_flag){
    if (logfile.is_open()) {
        std::cout<<"close!!!!"<<std::endl;
        logfile.close();
    }
  }
  return 0;
}
