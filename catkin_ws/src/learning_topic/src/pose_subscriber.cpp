#include "turtlesim/Pose.h"
#include <ros/ros.h>

// 收到订阅的信息后，进入回调函数
void poseCallback(const turtlesim::Pose::ConstPtr &msg) {
  // 打印接收到的信息
  ROS_INFO("Turtle Pose: x:%0.6f y:%0.6f", msg->x, msg->y);
}

int main(int argc, char **argv) {

  //初始化节点
  ros::init(argc, argv, "pose_subscriber");

  // 创建节点句柄
  ros::NodeHandle n;

  // 创建subscriber，订阅名为/turtle1/pose的topic，注册回调函数
  ros::Subscriber pos_sub = n.subscribe("/turtle1/pose", 10, poseCallback);

  //循环等待回调函数
  ros::spin();
}