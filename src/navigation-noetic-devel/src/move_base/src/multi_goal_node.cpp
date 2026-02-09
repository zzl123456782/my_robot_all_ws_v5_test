#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include <move_base_msgs/MoveBaseActionResult.h>
#include <actionlib_msgs/GoalStatus.h>
#include <queue>
#include <vector>
#include <iostream>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

class Multi_goal : public ros::NodeHandle {
private:
    ros::NodeHandle nh;
    ros::Publisher goal_pub_;
    ros::Subscriber odom_sub_;
    ros::Subscriber goal_state_sub_;
    ros::Timer timer_;

    nav_msgs::Odometry current_odom_;
    std::vector<geometry_msgs::PoseStamped> goal_list_;
    std::queue<geometry_msgs::PoseStamped> goal_queue_;  // 目标点队列
    geometry_msgs::PoseStamped current_goal_;           // 当前目标点
    
    // 状态变量
    bool is_navigating_;
    bool goal_active_;
    int goal_count_;
    int total_goals_;

    // 从yaw角度创建四元数的辅助函数
    geometry_msgs::Quaternion createQuaternionFromYaw(double yaw) {
        tf2::Quaternion q;
        q.setRPY(0, 0, yaw);  // 设置绕z轴的旋转
        geometry_msgs::Quaternion quat_msg;
        tf2::convert(q, quat_msg);
        return quat_msg;
    }

    // 从四元数获取yaw角度
    double getYawFromQuaternion(const geometry_msgs::Quaternion& quat) {
        tf2::Quaternion q(
            quat.x,
            quat.y,
            quat.z,
            quat.w
        );
        tf2::Matrix3x3 m(q);
        double roll, pitch, yaw;
        m.getRPY(roll, pitch, yaw);
        return yaw;
    }

    void odom_callback(const nav_msgs::Odometry::ConstPtr& msg) {
        // std::cout << "订阅里程计位置" << std::endl;
        current_odom_ = *msg;
    }

    void time_callback(const ros::TimerEvent& event) {
        // std::cout << "定时器" << std::endl;
        
        // 如果没有在导航，就发布目标点
        if (!is_navigating_ && !goal_queue_.empty()) {
            std::cout << "开始导航..." << std::endl;
            is_navigating_ = true;
            goal_active_ = false;
            publish_next_goal();
        }
    }

    void goal_state_callback(const move_base_msgs::MoveBaseActionResult::ConstPtr& msg) {
        std::cout << "到达目标点的状态: " << static_cast<int>(msg->status.status) << std::endl;

        switch (msg->status.status) {
            case actionlib_msgs::GoalStatus::SUCCEEDED:
                std::cout << "目标点 " << goal_count_ << " 成功到达!" << std::endl;
                goal_active_ = false;
                
                if (!goal_queue_.empty()) {
                    std::cout << "准备前往下一个目标点..." << std::endl;
                    ros::Duration(1.0).sleep(); // 等待1秒
                    publish_next_goal();
                } else {
                    std::cout << "所有目标点已完成!" << std::endl;
                    // is_navigating_ = false;
                    initialize_goal_queue();
                    goal_count_ = 0;
                    publish_next_goal();
                    
                }
                break;
                
            case actionlib_msgs::GoalStatus::ABORTED:
                std::cout << "目标点 " << goal_count_ << " 失败，尝试重新发布..." << std::endl;
                // 可以尝试重新发布当前目标点
                republish_current_goal();
                break;
                
            case actionlib_msgs::GoalStatus::PREEMPTED:
                std::cout << "目标点 " << goal_count_ << " 被抢占" << std::endl;
                goal_active_ = false;
                break;
                
            case actionlib_msgs::GoalStatus::REJECTED:
                std::cout << "目标点 " << goal_count_ << " 被拒绝" << std::endl;
                goal_active_ = false;
                break;
                
            default:
                std::cout << "目标点 " << goal_count_ << " 状态: " 
                          << static_cast<int>(msg->status.status) << std::endl;
                break;
        }
    }

    // 发布下一个目标点
    void publish_next_goal() {
        if (goal_queue_.empty()) {
            std::cout << "目标点队列已空" << std::endl;
            is_navigating_ = false;
            return;
        }
        
        // 从队列中取出下一个目标点
        current_goal_ = goal_queue_.front();
        goal_queue_.pop();
        
        // 更新目标点计数和时间戳
        goal_count_++;
        current_goal_.header.stamp = ros::Time::now();
        
        // 发布目标点
        goal_pub_.publish(current_goal_);
        goal_active_ = true;
        
        // 提取yaw角度用于显示
        double yaw = getYawFromQuaternion(current_goal_.pose.orientation);
        
        std::cout << "发布目标点 " << goal_count_ << "/" << total_goals_ 
                  << " 位置: (" << current_goal_.pose.position.x 
                  << ", " << current_goal_.pose.position.y << ")" 
                  << " 朝向: " << yaw << " 弧度"
                  << " (" << yaw * 180.0 / M_PI << " 度)" 
                  << std::endl;
    }
    
    // 重新发布当前目标点
    void republish_current_goal() {
        if (!goal_active_) {
            std::cout << "当前没有活跃的目标点" << std::endl;
            return;
        }
        
        // 更新目标点时间戳
        current_goal_.header.stamp = ros::Time::now();
        
        // 重新发布
        goal_pub_.publish(current_goal_);
        
        std::cout << "重新发布目标点 " << goal_count_ << std::endl;
    }
    
    // 初始化目标点队列
    void initialize_goal_queue() {
        // 清空队列
        while (!goal_queue_.empty()) {
            goal_queue_.pop();
        }
        
        // 将目标点列表复制到队列中
        for (const auto& goal : goal_list_) {
            goal_queue_.push(goal);
        }
        
        total_goals_ = goal_list_.size();
        std::cout << "初始化目标点队列，共 " << total_goals_ << " 个目标点" << std::endl;
    }

public:
    Multi_goal() : ros::NodeHandle(), is_navigating_(false), goal_active_(false), 
                  goal_count_(0), total_goals_(0) {
        std::cout << "Multi Goals is start" << std::endl;
        
        // 初始化发布器和订阅器
        goal_pub_ = nh.advertise<geometry_msgs::PoseStamped>("/move_base_simple/goal", 100);
        odom_sub_ = nh.subscribe<nav_msgs::Odometry>("/odom", 100, &Multi_goal::odom_callback, this);
        goal_state_sub_ = nh.subscribe<move_base_msgs::MoveBaseActionResult>("/move_base/result", 100, &Multi_goal::goal_state_callback, this);
        timer_ = nh.createTimer(ros::Duration(0.1), &Multi_goal::time_callback, this);
        
        // 初始化目标点
        initialize_goals();
    }

    // 初始化目标点
    void initialize_goals() {
        std::cout << "初始化目标点" << std::endl;
        goal_list_.clear();
        
        // 定义目标点列表 (x, y, yaw)
        std::vector<std::tuple<double, double, double>> way_points = {
            {1.0, 0.0, 0.0},      // 目标点1，朝0度方向
            {2.0, 1.0, 1.57},     // 目标点2，朝90度方向
            {1.0, 2.0, 3.14}      // 目标点3，朝180度方向
        };

        for (const auto& point : way_points) {
            geometry_msgs::PoseStamped goal;
            goal.header.frame_id = "map";
            goal.header.stamp = ros::Time::now();
            goal.pose.position.x = std::get<0>(point);
            goal.pose.position.y = std::get<1>(point);
            goal.pose.position.z = 0.0;  // 2D导航z通常为0
            
            // 使用正确的四元数设置方向
            double yaw = std::get<2>(point);
            goal.pose.orientation = createQuaternionFromYaw(yaw);
            
            goal_list_.push_back(goal);
            
            std::cout << "添加目标点: (" << std::get<0>(point) << ", " 
                      << std::get<1>(point) << "), 朝向: " << yaw 
                      << " 弧度" << std::endl;
        }
        
        // 初始化目标点队列
        initialize_goal_queue();
    }

    // 添加自定义目标点
    void add_goal(double x, double y, double yaw, const std::string& frame_id = "map") {
        geometry_msgs::PoseStamped goal;
        goal.header.frame_id = frame_id;
        goal.header.stamp = ros::Time::now();
        goal.pose.position.x = x;
        goal.pose.position.y = y;
        goal.pose.position.z = 0.0;
        goal.pose.orientation = createQuaternionFromYaw(yaw);
        
        goal_list_.push_back(goal);
        initialize_goal_queue();  // 重新初始化队列
        
        std::cout << "添加自定义目标点: (" << x << ", " << y 
                  << "), 朝向: " << yaw << " 弧度" << std::endl;
    }

    // 开始导航
    void start_navigation() {
        if (goal_list_.empty()) {
            std::cout << "目标点列表为空，无法开始导航" << std::endl;
            return;
        }
        
        std::cout << "开始导航..." << std::endl;
        is_navigating_ = true;
        goal_active_ = false;
        goal_count_ = 0;
        
        // 重新初始化队列
        initialize_goal_queue();
        
        // 发布第一个目标点
        publish_next_goal();
    }

    // 停止导航
    void stop_navigation() {
        std::cout << "停止导航" << std::endl;
        is_navigating_ = false;
        goal_active_ = false;
        
        // 清空队列
        while (!goal_queue_.empty()) {
            goal_queue_.pop();
        }
    }

    // 获取导航状态
    bool is_navigating() const {
        return is_navigating_;
    }

    // 获取当前目标点计数
    int get_current_goal_count() const {
        return goal_count_;
    }

    // 获取总目标点数
    int get_total_goals() const {
        return total_goals_;
    }

    // 获取剩余目标点数
    int get_remaining_goals() const {
        return goal_queue_.size();
    }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "multi_goal_nav");

    Multi_goal multi_goal;

    // multi_goal.add_goal(0.0, 0.0, 0.0);
    
    // 可选：延迟一段时间后开始导航
    // ros::Duration(2.0).sleep();  // 等待2秒，让系统初始化
    // multi_goal.start_navigation();
    
    // 或者通过ROS参数或服务来触发开始导航
    
    ros::spin();
    
    return 0;
}