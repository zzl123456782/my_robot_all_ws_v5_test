#include "my_local_planner.h"
#include <pluginlib/class_list_macros.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <tf2/utils.h>  // 重要：包含tf2/utils.h
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <visualization_msgs/Marker.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>  // 添加


PLUGINLIB_EXPORT_CLASS( my_local_planner::MyLocalPlanner, nav_core::BaseLocalPlanner)
namespace my_local_planner{


    MyLocalPlanner::MyLocalPlanner(){
        setlocale(LC_ALL,"");
        ros::NodeHandle nh;
        odom_sub_ = nh.subscribe<nav_msgs::Odometry>("/odom", 1, &MyLocalPlanner::odomCallback,this);

        local_path_pub_ = nh.advertise<nav_msgs::Path>("local_path",1);
        target_point_pub_ = nh.advertise<geometry_msgs::PoseStamped>("target_point",1);
        frame_id = "odom";
    }

    MyLocalPlanner::~MyLocalPlanner(){
        geometry_msgs::Twist cmd_vel;
        cmd_vel.linear.x = 0;
        cmd_vel.angular.z = 0;
        std::cout << "析构函数启动" << std::endl;
    }

    void MyLocalPlanner::initialize(std::string name, tf2_ros::Buffer* tf, costmap_2d::Costmap2DROS* costmap_ros){
        if(!initialize_){
            std::cout << "这是我的局部规划器" << std::endl;
        }
        tf_ = tf;
        costmap_ros_ = costmap_ros;
        // tf_listener_ = tf2_ros::TransformListener* tf_;
        has_odom_ = false;

        initialize_ = true;

        
        
    }


    bool MyLocalPlanner::setPlan(const std::vector<geometry_msgs::PoseStamped>& plan)
    { 
        if(!initialize_){
            std::cout << "初始化函数有问题，请查看" << std::endl;
            return false;

        }
        // target_index_ = 0;
        // 重置所有导航状态
        target_index_ = 0;
        goal_distance_reached_ = false;
        goal_angular_reached_ = false;

        global_plan_ = plan;



        return true;
    }
    
    
    // bool MyLocalPlanner::computeVelocityCommands(geometry_msgs::Twist& cmd_vel){
    //     if(!tf_){
    //         std::cout << "tf_ 指针没有初始化成功" <<std::endl;
    //         cmd_vel.linear.x = 0;
    //         cmd_vel.angular.z = 0;
    //         return false;

    //     }
    //     if(!has_odom_){
    //         std::cout << "没有接收到odom信息" <<std::endl;
    //         cmd_vel.linear.x = 0;
    //         cmd_vel.angular.z = 0;
    //         return false;

    //     }

    //     //获取代价地图数据
    //     costmap_2d::Costmap2D* costmap = costmap_ros_ -> getCostmap();
    //     unsigned char* map_data = costmap->getCharMap();
    //     unsigned int size_x = costmap->getSizeInCellsX();
    //     unsigned int size_y = costmap->getSizeInCellsY();

    //     std::cout << "map的x大小:" << size_x <<"map的y的大小:" << size_y << std::endl;




    //     // 使用 OpenCV 绘制代价地图
    //     cv::Mat map_image(size_y, size_x, CV_8UC3, cv::Scalar(128, 128, 128));
    //     for (unsigned int y = 0; y < size_y; y++)
    //     {
    //         for (unsigned int x = 0; x < size_x; x++)
    //         {
    //             int map_index = y * size_x + x;
    //             unsigned char cost = map_data[map_index];               // 从代价地图数据取值
    //             cv::Vec3b& pixel = map_image.at<cv::Vec3b>(map_index);  // 获取彩图对应像素地址
                
    //             if (cost == 0)          // 可通行区域
    //                 pixel = cv::Vec3b(128, 128, 128); // 灰色
    //             else if (cost == 254)   // 障碍物
    //                 pixel = cv::Vec3b(0, 0, 0);       // 黑色
    //             else if (cost == 253)   // 禁行区域 
    //                 pixel = cv::Vec3b(255, 255, 0);   // 浅蓝色
    //             else
    //             {
    //                 // 根据灰度值显示从红色到蓝色的渐变
    //                 unsigned char blue = 255 - cost;
    //                 unsigned char red = cost;
    //                 pixel = cv::Vec3b(blue, 0, red);
    //             }
    //         }
    //     }

    //     // 在代价地图上遍历导航路径点
    //     for(int i=0;i<global_plan_.size();i++)
    //     {
    //         geometry_msgs::PoseStamped pose_odom;
    //         global_plan_[i].header.stamp = ros::Time(0);
    //         pose_odom = tf_->transform(global_plan_[i], "odom", ros::Duration(0.1));
    //         double odom_x = pose_odom.pose.position.x;
    //         double odom_y = pose_odom.pose.position.y;

    //         double origin_x = costmap->getOriginX();
    //         double origin_y = costmap->getOriginY();
    //         double local_x = odom_x - origin_x;
    //         double local_y = odom_y - origin_y;
    //         int x = local_x / costmap->getResolution();
    //         int y = local_y / costmap->getResolution();
    //         cv::circle(map_image, cv::Point(x,y), 0, cv::Scalar(255,0,255));    // 导航路径点


    //         // 检测前方路径点是否在禁行区域或者障碍物里
    //         if(i >= target_index_ && i < target_index_ + 10)
    //         {
    //             cv::circle(map_image, cv::Point(x,y), 0, cv::Scalar(0,255,255));// 检测路径点
    //             int map_index = y * size_x + x;
    //             unsigned char cost = map_data[map_index];
    //             if(cost >= 253){
    //                 std::cout << "路径进入到未知区域" << std::endl;
    //                 return false;
    //             }else if(cost == 254){
    //                 return false;
    //             }else if(cost > 200){
    //                 std::cout << "路径进入到高代价地区" << std::endl;
    //                 return false;
    //             }
            
    //         }
    //     }

    //     map_image.at<cv::Vec3b>(size_y/2, size_x/2) = cv::Vec3b(0, 255, 0); // 机器人位置

    //     // 翻转地图
    //     cv::Mat flipped_image(size_x, size_y, CV_8UC3, cv::Scalar(128, 128, 128));
    //     for (unsigned int y = 0; y < size_y; ++y)
    //     {
    //         for (unsigned int x = 0; x < size_x; ++x)
    //         {
    //             cv::Vec3b& pixel = map_image.at<cv::Vec3b>(y, x);
    //             flipped_image.at<cv::Vec3b>((size_x - 1 - x), (size_y - 1 - y)) = pixel;
    //         }
    //     }
    //     map_image = flipped_image;

    //     // 显示代价地图
    //     cv::namedWindow("Map");
    //     cv::resize(map_image, map_image, cv::Size(size_y*5, size_x*5), 0, 0, cv::INTER_NEAREST);
    //     cv::resizeWindow("Map", size_y*5, size_x*5);
    //     cv::imshow("Map", map_image);
    //     cv::waitKey(1);



    //     //目标点位置矫正
    //     geometry_msgs::PoseStamped pose_final_;
    //     int final_index_ = global_plan_.size() - 1;
    //     global_plan_[final_index_].header.stamp = ros::Time(0);
    //     pose_final_ = tf_->transform(global_plan_[final_index_], "base_link", ros::Time(0),global_plan_[final_index_].header.frame_id,
    //                                              ros::Duration(0.1));
    //     if(goal_distance_reached_ == false){
    //         double dx = pose_final_.pose.position.x;
    //         double dy = pose_final_.pose.position.y;
    //         double dist = std::sqrt(dx*dx + dy*dy);
    //         if(dist < 0.05)
    //             goal_distance_reached_ = true;
    //     }
    //     if(goal_distance_reached_ == true){
    //         double final_yaw = tf2::getYaw(pose_final_.pose.orientation);
    //         std::cout << "调整最终的位子" << std::endl;
    //         cmd_vel.linear.x = pose_final_.pose.position.x * 1.5;
    //         cmd_vel.angular.z = final_yaw * 0.5;
    //         if(abs(final_yaw) < 0.1)
    //         {
    //             goal_angular_reached_ = true;
    //             std::cout << "到达最后的终点" << std::endl;
    //             cmd_vel.linear.x = 0;
    //             cmd_vel.angular.z = 0;
    //         }
    //         return true;
    //     }
        
        
    //     geometry_msgs::PoseStamped target_pose;



    //     for(int i = target_index_; i < global_plan_.size(); i++){
    //         geometry_msgs::PoseStamped pose_base;

    //         pose_base = tf_->transform(global_plan_[i], "base_link", ros::Time(0),global_plan_[i].header.frame_id,
    //                                              ros::Duration(0.1));

    //         double dx = pose_base.pose.position.x;
    //         double dy = pose_base.pose.position.y;
    //         double dist = std::sqrt(dx*dx + dy*dy);

    //         if(dist > 0.2){
    //             target_pose = pose_base;
    //             target_index_ = i;
    //             std::cout << "选择第" << target_index_ << "个路径点作为临时目标，距离" << dist << std::endl;
    //             break;
    //         }

    //         if(i == global_plan_.size() -1){
    //             target_pose = pose_base;
    //             std::cout << "轮到最后一个点拉" << std::endl;

    //         }
    //     }


    //     publishLocalPath();

    //     cmd_vel.linear.x = 0.2;
    //     // cmd_vel.angular.z = 0.1;

    //     // cmd_vel.linear.x = target_pose.pose.position.x * 1.5;
    //     cmd_vel.angular.z = target_pose.pose.position.y * 5.0;

    //     // // 限制速度范围
    //     // cmd_vel.linear.x = std::min(std::max(cmd_vel.linear.x, -0.5), 0.5);
    //     cmd_vel.angular.z = std::min(std::max(cmd_vel.angular.z, -0.5), 0.5);

     



    //     // 绘制路径点
    //     cv::Mat plan_image(600, 600, CV_8UC3, cv::Scalar(0, 0, 0));        
    //     for(int i = 0; i < global_plan_.size(); i++)
    //     {
    //         geometry_msgs::PoseStamped pose_base;
            
    //         try {
    //             pose_base = tf_->transform(global_plan_[i], "base_link", ros::Time(0),global_plan_[i].header.frame_id,
    //                                              ros::Duration(0.1));
                
    //             int cv_x = 300 - pose_base.pose.position.y * 100;
    //             int cv_y = 300 - pose_base.pose.position.x * 100;
    //             cv::circle(plan_image, cv::Point(cv_x, cv_y), 1, cv::Scalar(255, 0, 255)); 
    //         } catch (tf2::TransformException &ex) {
    //             ROS_WARN("绘制时TF转换失败: %s", ex.what());
    //         }
    //     }
        
    //     // 绘制坐标系
    //     cv::circle(plan_image, cv::Point(300, 300), 15, cv::Scalar(0, 255, 0));
    //     cv::line(plan_image, cv::Point(65, 300), cv::Point(510, 300), cv::Scalar(0, 255, 0), 1);
    //     cv::line(plan_image, cv::Point(300, 45), cv::Point(300, 555), cv::Scalar(0, 255, 0), 1);

    //     // 绘制当前目标点
    //     int target_x = 300 - target_pose.pose.position.y * 100;
    //     int target_y = 300 - target_pose.pose.position.x * 100;
    //     cv::circle(plan_image, cv::Point(target_x, target_y), 8, cv::Scalar(0, 0, 255), 2);

    //     // cv::namedWindow("Plan");
    //     // cv::imshow("Plan", plan_image);
    //     // cv::waitKey(1);



    //     return true;

    // }

    bool MyLocalPlanner::computeVelocityCommands(geometry_msgs::Twist& cmd_vel){
        if(!tf_){
            std::cout << "tf_ 指针没有初始化成功" << std::endl;
            cmd_vel.linear.x = 0;
            cmd_vel.angular.z = 0;
            return false;
        }
        if(!has_odom_){
            std::cout << "没有接收到odom信息" << std::endl;
            cmd_vel.linear.x = 0;
            cmd_vel.angular.z = 0;
            return false;
        }

        //目标点位置矫正
        geometry_msgs::PoseStamped pose_final_;
        int final_index_ = global_plan_.size() - 1;
        global_plan_[final_index_].header.stamp = ros::Time(0);
        pose_final_ = tf_->transform(global_plan_[final_index_], "base_link", ros::Time(0),global_plan_[final_index_].header.frame_id,
                                                ros::Duration(0.1));
        if(goal_distance_reached_ == false){
            double dx = pose_final_.pose.position.x;
            double dy = pose_final_.pose.position.y;
            double dist = std::sqrt(dx*dx + dy*dy);
            if(dist < 0.05)
                goal_distance_reached_ = true;
        }
        if(goal_distance_reached_ == true){
            double final_yaw = tf2::getYaw(pose_final_.pose.orientation);
            std::cout << "调整最终的位子" << std::endl;
            cmd_vel.linear.x = pose_final_.pose.position.x * 1.5;
            cmd_vel.angular.z = final_yaw * 0.5;
            if(abs(final_yaw) < 0.1)
            {
                goal_angular_reached_ = true;
                std::cout << "到达最后的终点" << std::endl;
                cmd_vel.linear.x = 0;
                cmd_vel.angular.z = 0;
            }
            return true;
        }

        double look_ahead_distance = std::max(0.2, current_velocity_.linear.x * 2.0);
        
        geometry_msgs::PoseStamped target_pose;

        for(int i = target_index_; i < global_plan_.size(); i++){
            geometry_msgs::PoseStamped pose_base;

            pose_base = tf_->transform(global_plan_[i], "base_link", ros::Time(0),global_plan_[i].header.frame_id,
                                                ros::Duration(0.1));

            double dx = pose_base.pose.position.x;
            double dy = pose_base.pose.position.y;
            double dist = std::sqrt(dx*dx + dy*dy);

            if(dist > look_ahead_distance){
                target_pose = pose_base;
                target_index_ = i;
                std::cout << "选择第" << target_index_ << "个路径点作为临时目标，距离" << dist << std::endl;
                break;
            }

            if(i == global_plan_.size() -1){
                target_pose = pose_base;
                std::cout << "轮到最后一个点拉" << std::endl;
            }
        }

        publishLocalPath();

        // cmd_vel.linear.x = 0.2;
        // cmd_vel.angular.z = target_pose.pose.position.y * 5.0;

        double target_yaw = std::atan2(target_pose.pose.position.y, target_pose.pose.position.x);

        if(target_yaw >1){
            cmd_vel.linear.x = 0.0;
            cmd_vel.angular.z = target_pose.pose.position.y * 1.5;

        }else{
            cmd_vel.linear.x = 0.2;
            cmd_vel.angular.z = target_pose.pose.position.y * 1.5;

        }

         // 1. 计算期望速度（基于位置）
        // double target_angular = target_pose.pose.position.y * 5.0;
        
        // // 2. 计算速度误差
        // double angular_error = target_angular - current_velocity_.angular.z;
        
        // // 3. 简单的P控制器（可扩展为完整的PID）
        // static double last_angular_error = 0;
        // static double angular_integral = 0;
        
        // // PID参数
        // double kp_ang = 3.0, ki_ang = 0.0, kd_ang = 0.0;
        
        // // 计算积分项（带抗饱和）
        
        // angular_integral += angular_error;
        // angular_integral = std::max(-1.0, std::min(1.0, angular_integral));
        
        // // 计算微分项
        // double angular_derivative = angular_error - last_angular_error;
        
        // // PID输出
        // cmd_vel.angular.z = kp_ang * angular_error + 
        //                     ki_ang * angular_integral + 
        //                     kd_ang * angular_derivative;
        
        // // 保存当前误差用于下次微分
        // last_angular_error = angular_error;
        

        // 限制速度范围
        cmd_vel.angular.z = std::min(std::max(cmd_vel.angular.z, -0.5), 0.5);

        return true;
    }

    bool MyLocalPlanner::ShouldReplan(){
        if(global_plan_.empty()) return false;
        //获取代价地图数据
        costmap_2d::Costmap2D* costmap = costmap_ros_ -> getCostmap();
        unsigned char* map_data = costmap->getCharMap();
        unsigned int size_x = costmap->getSizeInCellsX();
        unsigned int size_y = costmap->getSizeInCellsY();

        std::cout << "map的x大小:" << size_x << "map的y的大小:" << size_y << std::endl;

        // 检测前方路径点是否在禁行区域或者障碍物里
        // 优化：只检查 target_index_ 后面的一段，而不是从 0 开始检查全部
        int check_start_index = target_index_;
        int check_end_index = std::min((int)global_plan_.size(), target_index_ + 50); // 只检查前方 50 个点

        for(int i = check_start_index; i < check_end_index; i++)
        {
            geometry_msgs::PoseStamped pose_odom;
            global_plan_[i].header.stamp = ros::Time(0);
            pose_odom = tf_->transform(global_plan_[i], "odom", ros::Duration(0.1));
            double odom_x = pose_odom.pose.position.x;
            double odom_y = pose_odom.pose.position.y;

            double origin_x = costmap->getOriginX();
            double origin_y = costmap->getOriginY();
            double local_x = odom_x - origin_x;
            double local_y = odom_y - origin_y;
            int x = local_x / costmap->getResolution();
            int y = local_y / costmap->getResolution();

            // 检测前方路径点是否在禁行区域或者障碍物里
            // if(i >= target_index_ && i < target_index_ + 30)
            {
                int map_index = y * size_x + x;
                unsigned char cost = map_data[map_index];
                if(cost >= 253 || cost > 200 || cost == 254){
                    std::cout << "路径进入到高代价地区" << std::endl;
                    return true;
                }
            }
        }
        return false;

    }


    bool MyLocalPlanner::isGoalReached()
    {
        return goal_angular_reached_;
    }

    void MyLocalPlanner::odomCallback(const nav_msgs::Odometry::ConstPtr& msg){
        current_velocity_ = msg -> twist.twist;
        has_odom_ = true;
    }

    void MyLocalPlanner::publishLocalPath(){
        if(global_plan_.empty() || !tf_){
            return;
        }
        nav_msgs::Path local_path;
        local_path.header.stamp = ros::Time::now();
        local_path.header.frame_id = "odom";
        int look_ahead_count = 20;
        int start_index = target_index_;
        int end_index = std::min((int)global_plan_.size(),target_index_ + look_ahead_count);
        for(int i = start_index; i < end_index; i++){
            try {
                // 将路径点转换到odom坐标系
                geometry_msgs::PoseStamped pose_in_odom;
                tf_->transform(global_plan_[i], pose_in_odom, "odom", ros::Duration(0.1));
                local_path.poses.push_back(pose_in_odom);
            } catch (tf2::TransformException &ex) {
                ROS_WARN("发布局部路径时TF转换失败: %s", ex.what());
                break;
            }
        }
        
        // 只有有路径点时才发布
        if(!local_path.poses.empty()){
            local_path_pub_.publish(local_path);
        }
    }

    void MyLocalPlanner::publishTargetPose(const geometry_msgs::PoseStamped& target_pose){
        try {
            geometry_msgs::PoseStamped target_in_odom;
            tf_->transform(target_pose, target_in_odom, "odom", ros::Duration(0.1));
            target_point_pub_.publish(target_in_odom);
        } catch (tf2::TransformException &ex) {
            ROS_WARN("发布目标点时TF转换失败: %s", ex.what());
        }
    }




}


