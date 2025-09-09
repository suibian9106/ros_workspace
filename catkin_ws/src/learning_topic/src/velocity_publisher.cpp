#include<ros/ros.h>
#include<geometry_msgs/Twist.h>

int main(int argc, char** argv) {
    // 初始化节点
    ros::init(argc, argv, "velocity_publisher");

    // 创建句柄
    ros::NodeHandle n;

    // 创建Publisher,话题名/turtle1/cmd_vel，消息类型geometry_msgs::Twist，队列长度10
    ros::Publisher turtle_vel_pub = n.advertise<geometry_msgs::Twist>("/turtle1/cmd_vel", 10);

    // 循环频率
    ros::Rate loop_rate(10);

    int count = 0;
    while(ros::ok()) {

        geometry_msgs::Twist vel_msg;
        vel_msg.linear.x = 0.5;
        vel_msg.angular.z = 0.2;

        turtle_vel_pub.publish(vel_msg);
        ROS_INFO("陀螺在旋转");

        // 按照循环频率延时
        loop_rate.sleep();
    }
}