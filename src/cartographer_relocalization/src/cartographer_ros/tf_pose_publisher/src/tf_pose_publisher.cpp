#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Pose2D.h>
#include <iostream>

using namespace std;

int main(int argc, char** argv){
    ros::init(argc, argv, "tf_Odometry_Publisher");

    ros::NodeHandle node;
    // 发布Odometry消息
    ros::Publisher odom_pub = node.advertise<nav_msgs::Odometry>("odom", 10);
    
    tf::TransformListener listener;
    ros::Rate rate(10.0);

    while (ros::ok()){
        tf::StampedTransform transform;
        try{
            // 获取map和velodyne之间的变换
            listener.waitForTransform("map", "laser", ros::Time(0), ros::Duration(3.0));
            listener.lookupTransform("map", "laser", ros::Time(0), transform);
        }
        catch (tf::TransformException &ex) {
            ROS_ERROR("%s", ex.what());
            ros::Duration(1.0).sleep();
            continue;
        }
        // 提取位置
        double x = transform.getOrigin().x();
        double y = transform.getOrigin().y();
        double z = transform.getOrigin().z();
        // 提取姿态
        tf::Quaternion q = transform.getRotation();
        // 创建Odometry消息
        nav_msgs::Odometry odom_msg;
        
        // 设置header
        odom_msg.header.stamp = ros::Time::now();
        odom_msg.header.frame_id = "map";      // 父坐标系
        odom_msg.child_frame_id = "laser";  // 子坐标系

        // 设置位置
        odom_msg.pose.pose.position.x = x;
        odom_msg.pose.pose.position.y = y;
        odom_msg.pose.pose.position.z = z;
        
        // 设置姿态
        odom_msg.pose.pose.orientation.x = q.x();
        odom_msg.pose.pose.orientation.y = q.y();
        odom_msg.pose.pose.orientation.z = q.z();
        odom_msg.pose.pose.orientation.w = q.w();

        // 发布里程计消息
        odom_pub.publish(odom_msg);

        // 输出调试信息
        printf("Position: x: %.3f, y: %.3f, z: %.3f\n", x, y, z);
        printf("Orientation: qx: %.3f, qy: %.3f, qz: %.3f, qw: %.3f\n", 
               q.x(), q.y(), q.z(), q.w());

        rate.sleep();
    }
    return 0;
}