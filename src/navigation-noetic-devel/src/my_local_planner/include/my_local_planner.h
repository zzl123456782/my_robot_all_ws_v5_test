#ifndef MY_LOCAL_PLANNER_H_
#define MY_LOCAL_PLANNER_H_




#include<ros/ros.h>
#include<nav_core/base_local_planner.h>
#include<tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <visualization_msgs/Marker.h>




namespace my_local_planner{

    class MyLocalPlanner : public nav_core::BaseLocalPlanner{
        private:
            bool initialize_ = false;
            bool goal_angular_reached_ = false;
            bool goal_distance_reached_ = false;

            std::vector<geometry_msgs::PoseStamped> global_plan_;
            costmap_2d::Costmap2DROS* costmap_ros_;

            tf2_ros::Buffer* tf_;

            int target_index_;

            ros::Subscriber odom_sub_;
            geometry_msgs::Twist current_velocity_;
            bool has_odom_;


            ros::Publisher local_path_pub_;
            ros::Publisher target_point_pub_;
            
            std::string frame_id;
            // ros:NodeHandle nh_;


            




        public:
            MyLocalPlanner();
            ~MyLocalPlanner();

            void initialize(std::string name, tf2_ros::Buffer* tf, costmap_2d::Costmap2DROS* costmap_ros);
            bool setPlan(const std::vector<geometry_msgs::PoseStamped>& plan);
            bool computeVelocityCommands(geometry_msgs::Twist& cmd_vel);
            bool isGoalReached();
            bool ShouldReplan();

            void odomCallback(const nav_msgs::Odometry::ConstPtr& msg);

            void publishLocalPath();
            void publishTargetPose(const geometry_msgs::PoseStamped& target_pose);
    };


}















#endif




