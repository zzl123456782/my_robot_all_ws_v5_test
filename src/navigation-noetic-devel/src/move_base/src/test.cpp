
#include<geometry_msgs/PoseStamped.h>
#include<std_msgs/String.h>
#include<tf2/LinearMath/Quaternion.h>
#include<tf2_geometry_msgs/tf2_geometry_msgs.h>
#include<thread>
#include<iostream>
#include<nav_msgs/Odometry.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>

class Pub_test: public ros::NodeHandle
{
private:
    ros::NodeHandle nh;
    ros::Publisher goal_pub_;
    ros::Timer timer_;
    
    // 巡航点定义
    std::vector<std::tuple<double,double,double>> goal_positions_ = {
        std::make_tuple(4.0, 0.0, 0.0),
        std::make_tuple(4.0, -3.0, 0.0),
        std::make_tuple(1.0, -3.0, 0.0),
        std::make_tuple(0.0, 0.0, 0.0)
    };
    std::vector<double> goal_yaw_ = {-1.5708, 3.1416, 1.8925, 0.0};
    
    const double xy_tolerance_ = 0.5;
    int current_goal_index_ = 0;

    // 状态机枚举
    enum RobotState {
        INIT,
        NAVIGATING,     // 正在前往目标点
        REACHED_WAITING // 到达目标点，正在等待
    };
    RobotState state_ = INIT;

    // 定时器相关
    ros::Time wait_start_time_;
    const ros::Duration wait_duration_ = ros::Duration(5.0);   // 到达后等待时间
    const ros::Duration final_wait_duration_ = ros::Duration(15.0); // 最后一个点等待时间

    // TF2
    tf2_ros::Buffer tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

public:
    Pub_test(): ros::NodeHandle(){
        ROS_INFO("Initializing Multi-Point Patrol Node...");
        
        goal_pub_ = nh.advertise<geometry_msgs::PoseStamped>("move_base_simple/goal", 1);
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(tf_buffer_);
        
        // 等待 TF 就绪
        ros::Duration(1.0).sleep();
        
        // 设置定时器，10Hz 检查状态
        timer_ = nh.createTimer(ros::Duration(0.1), &Pub_test::controlLoop, this);
        
        // 初始状态
        state_ = INIT;
    }
    
    ~Pub_test() = default;

    void controlLoop(const ros::TimerEvent& event)
    {
        // 获取当前机器人位置
        double current_x = 0.0;
        double current_y = 0.0;
        if (!getRobotPose(current_x, current_y)) {
            return; // TF 失败，跳过本次循环
        }

        // 状态机逻辑
        switch (state_) {
            case INIT:
                // 初始状态，发送第一个目标点
                ROS_INFO("Starting patrol. Heading to goal %d.", current_goal_index_);
                publishGoal(current_goal_index_);
                state_ = NAVIGATING;
                break;

            case NAVIGATING:
            {
                // 检查距离
                double goal_x = std::get<0>(goal_positions_[current_goal_index_]);
                double goal_y = std::get<1>(goal_positions_[current_goal_index_]);
                double distance = std::hypot(goal_x - current_x, goal_y - current_y);

                if (distance < xy_tolerance_) {
                    ROS_INFO("Goal %d reached. Waiting...", current_goal_index_);
                    wait_start_time_ = ros::Time::now();
                    state_ = REACHED_WAITING;
                }
                // 如果需要，可以在这里添加超时重发逻辑，防止数据包丢失
                break;
            }

            case REACHED_WAITING:
            {
                ros::Duration elapsed = ros::Time::now() - wait_start_time_;
                ros::Duration required_wait = (current_goal_index_ < goal_positions_.size() - 1) ? wait_duration_ : final_wait_duration_;

                if (elapsed >= required_wait) {
                    // 等待结束，切换到下一个点
                    current_goal_index_++;
                    if (current_goal_index_ >= goal_positions_.size()) {
                        ROS_INFO("All goals visited. Restarting patrol.");
                        current_goal_index_ = 0;
                    }
                    
                    ROS_INFO("Wait finished. Heading to goal %d.", current_goal_index_);
                    publishGoal(current_goal_index_);
                    state_ = NAVIGATING;
                }
                break;
            }
        }
    }

    bool getRobotPose(double &x, double &y) {
        try {
            geometry_msgs::TransformStamped transform = 
                tf_buffer_.lookupTransform("map", "base_link", ros::Time(0));
            x = transform.transform.translation.x;
            y = transform.transform.translation.y;
            return true;
        } catch (tf2::TransformException &ex) {
            ROS_WARN_THROTTLE(2.0, "TF lookup failed: %s", ex.what());
            return false;
        }
    }

    void publishGoal(int index)
    {
        if (index < 0 || index >= goal_positions_.size()) return;

        geometry_msgs::PoseStamped goal;
        goal.header.stamp = ros::Time::now();
        goal.header.frame_id = "map";
        
        goal.pose.position.x = std::get<0>(goal_positions_[index]);
        goal.pose.position.y = std::get<1>(goal_positions_[index]);
        goal.pose.position.z = 0.0;

        // 设置方向
        double yaw = goal_yaw_[index];
        tf2::Quaternion q;
        q.setRPY(0, 0, yaw);
        goal.pose.orientation = tf2::toMsg(q);

        goal_pub_.publish(goal);
        ROS_INFO("Published goal: [%.2f, %.2f, yaw: %.2f]", 
            goal.pose.position.x, goal.pose.position.y, yaw);
    }
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "multi_patrol_node");
    Pub_test node;
    ros::spin();
    return 0;
}