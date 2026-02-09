#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include "socketcan.h"
#include "dbc_encoder_decoder.h"
#include <std_msgs/Float64MultiArray.h>
//用于发布标准里程计和TF变换
#include <nav_msgs/Odometry.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <sensor_msgs/BatteryState.h>
#include <std_msgs/UInt32.h>
#include <std_msgs/String.h>
#include <std_msgs/Float32.h>
#include <sstream>
#include <clocale>

class Can_ {
    private:
        ros::NodeHandle nh; // 建议组合优于继承，这里作为成员变量
        ros::Timer time_, rec_timer_, bms_wakeup_timer_;
        ros::Subscriber control_sub_;
        ros::Publisher vel_pub_;
        
        // 旧的发布器 (保留以兼容你之前的代码)
        ros::Publisher odometry_pub_legacy_;
        
        // 新增：标准里程计发布器 和 TF广播器
        ros::Publisher odom_pub_;
        ros::Publisher path_pub_;
        ros::Publisher bat_pub_; // 新增: 电池状态发布器
        ros::Publisher fault_pub_; // 新增: 故障信息发布器
        ros::Publisher soc_pub_;   // 新增: 独立 SOC 发布器
        tf2_ros::TransformBroadcaster odom_broadcaster_;
        tf2_ros::TransformBroadcaster static_broadcaster;

        // static ros::Time start_time;
        ros::Time now_time;
        ros::Duration esplation_;
        bool is_command_received_ = false;

        /*************Can releate variables************/
        dbc_encoder my_dbc;
        ADCU_CMD adcu_cmd;
        struct can_frame frame_0x20, frame;
        int can_socket_;
        double left_speed, right_speed;
        BMS_Data bms_data; // New BMS Data storage
        
        /*******************Wheel parameters**********/
        double track_width_ = 0.3; //m
        double wheel_radius = 0.0535; //m
        
        /********************Pose********************/
        double linear_velocity = 0, angluar_velocity = 0;
        double x = 0, y = 0, theta = 0; //m,m,rad
        ros::Time last_time_;
        nav_msgs::Path path_msg_;

        void timerCallback(const ros::TimerEvent&){
            static ros::Time start_time = ros::Time::now();
            now_time = ros::Time::now();
            esplation_ = now_time - start_time;
            // printf("esplation=%.2f",esplation_.toSec()); // 减少刷屏
            // printf("start_time=%.2f",start_time.toSec());
            start_time = now_time;
            
            if(!is_command_received_){
                // ROS_INFO_THROTTLE(1, "No command received yet"); 
            }
        }

        void controlCallback(const geometry_msgs::Twist::ConstPtr& msg){
            double linear_x = msg->linear.x;
            double angular_z = msg->angular.z * 0.727;
            // double angular_z = msg->angular.z;
            is_command_received_ = true;
            if(is_command_received_){
                my_dbc.encode0x20(linear_x, angular_z, frame_0x20, adcu_cmd);
                sendCANFrame(can_socket_, frame_0x20);
                // ROS_INFO("linear_x = %.2f, angular_z = %.2f", linear_x, angular_z);
            }
        }

        double normalizeAngle(double angle){
            while(angle > M_PI) angle -= 2.0*M_PI;
            while(angle < -M_PI) angle += 2.0*M_PI;
            return angle;
        }

    public:
        Can_() {
            ROS_INFO("can_node_start");
            can_socket_ = initializeSocketCAN("can0");
            if (can_socket_ < 0){
                ROS_WARN("Failed to initialize CAN socket ");
                throw std::runtime_error("SockerCAN initialization failed");
            }

            // 定时器
            time_ = nh.createTimer(ros::Duration(5.0), std::bind(&Can_::timerCallback, this, std::placeholders::_1));
            // 提高里程计更新频率到 50Hz (0.02s)，这对 TF 平滑度很重要
            rec_timer_ = nh.createTimer(ros::Duration(0.02), std::bind(&Can_::recCallback, this, std::placeholders::_1));

            // 订阅与发布
            control_sub_ = nh.subscribe("/cmd_vel", 10, &Can_::controlCallback, this);
            vel_pub_ = nh.advertise<geometry_msgs::Twist>("cmd_vel", 10);

            // 保留旧的发布器
            odometry_pub_legacy_ = nh.advertise<std_msgs::Float64MultiArray>("chassis_Odometry", 10);
            
            // 新增：发布标准 odom 话题
            odom_pub_ = nh.advertise<nav_msgs::Odometry>("odom", 50);

            //traj topic
            path_pub_ = nh.advertise<nav_msgs::Path>("trajectory", 10);
            path_msg_.header.frame_id = "odom";
            
            // BMS 发布器
            bat_pub_ = nh.advertise<sensor_msgs::BatteryState>("bms/battery_state", 10);
            fault_pub_ = nh.advertise<std_msgs::String>("bms/fault_info", 10);
            soc_pub_ = nh.advertise<std_msgs::Float32>("bms/soc", 10);
            
            // BMS 唤醒定时器 (2.0s 周期)
            bms_wakeup_timer_ = nh.createTimer(ros::Duration(2.0), std::bind(&Can_::bmsWakeupCallback, this, std::placeholders::_1));

            last_time_ = ros::Time::now();
        }

        ~Can_(){
            // 发送停止指令
            double linear_x = 0, angular_z = 0;
            my_dbc.encode0x20(linear_x, angular_z, frame_0x20, adcu_cmd);
            // 稍微减少发送次数并增加延时，防止堵塞
            for(int i = 0; i < 5; i++){
                sendCANFrame(can_socket_, frame_0x20);
                usleep(1000); 
            }
            std::cout << "delete cannode success" << std::endl;
            if(can_socket_ >= 0){
                close(can_socket_);
            }
        }

        // BMS 唤醒/心跳发送回调
        void bmsWakeupCallback(const ros::TimerEvent&) {
            struct can_frame wakeup_frame;
            my_dbc.encodeBMSControl(wakeup_frame);
            sendCANFrame(can_socket_, wakeup_frame);
            // ROS_INFO("Sent BMS Wakeup Frame");
        }

        void recCallback(const ros::TimerEvent&){
            std_msgs::Float64MultiArray msg;
            
            // 接收CAN数据并更新速度
            // Update signature to pass bms_data
            receiveCANFrame(can_socket_, left_speed, right_speed, bms_data);
            
            // Publish BMS Data
            sensor_msgs::BatteryState bat_msg;
            bat_msg.header.stamp = ros::Time::now();
            bat_msg.voltage = bms_data.info0.sum_voltage;
            bat_msg.current = bms_data.info0.current; 
            // Protocol: 0.1%. So value 500 = 50.0%.
            // ROS BatteryState percentage is float [0..1].
            bat_msg.percentage = (bms_data.info0.soc) / 100.0; 
            
            bat_msg.charge = bms_data.status2.remain_capacity / 1000.0; // mAH -> Ah
            bat_msg.capacity = 100.0; // Assume 100Ah or retrieve if available (Design Cap?)
            bat_msg.design_capacity = 100.0;
            
            bat_msg.power_supply_health = sensor_msgs::BatteryState::POWER_SUPPLY_HEALTH_GOOD; // Default
            if (bms_data.info0.life < 80) bat_msg.power_supply_health = sensor_msgs::BatteryState::POWER_SUPPLY_HEALTH_UNSPEC_FAILURE;
            
            bat_msg.power_supply_status = sensor_msgs::BatteryState::POWER_SUPPLY_STATUS_UNKNOWN;
            // Check status flags (e.g. charging state from chg_state if we parsed it, or current direction)
            if (bms_data.info0.current > 0.5) bat_msg.power_supply_status = sensor_msgs::BatteryState::POWER_SUPPLY_STATUS_CHARGING;
            else if (bms_data.info0.current < -0.5) bat_msg.power_supply_status = sensor_msgs::BatteryState::POWER_SUPPLY_STATUS_DISCHARGING;
            else bat_msg.power_supply_status = sensor_msgs::BatteryState::POWER_SUPPLY_STATUS_NOT_CHARGING;
            
            // 发布消息
            bat_pub_.publish(bat_msg);
            // 发布独立 SOC
            std_msgs::Float32 soc_msg;
            if(bms_data.info0.soc>0){
                soc_msg.data = bms_data.info0.soc; // 0.1% -> 实际值? 协议说是 0.1%，即 500 = 50.0%。
                // 这里我们直接发布转换后的浮点数 (例如 50.0)
                soc_pub_.publish(soc_msg);
                std::cout << "Published SOC: " << soc_msg.data << "%" << std::endl;
            }

            // 发布故障信息x
            std_msgs::String fault_msg;
            fault_msg.data = bms_data.fault_data.getFaultDescription();
            fault_pub_.publish(fault_msg);
            
            // 如果存在故障，打印到控制台 (每2秒最多一次)
            if (fault_msg.data != "systerm normal") {
                ROS_WARN_THROTTLE(2.0, "BMS error warning: %s", fault_msg.data.c_str());
            } else {
                ROS_INFO_THROTTLE(5.0, "BMS staus: %s", fault_msg.data.c_str());
            }
            
            // Debug 输出 (可选，注释掉以减少终端刷屏)
            // std::cout << "BMS V:" << bms_data.info0.sum_voltage << " C:" << bms_data.info0.current << " SOC:" << bms_data.info0.soc << std::endl; 
            
            double left_speed_ms = (left_speed/60*2*M_PI*wheel_radius);
            double right_speed_ms = (right_speed/60*2*M_PI*wheel_radius);
            // 计算线速度和角速度
            linear_velocity = (left_speed_ms + right_speed_ms) / 2; // m/s
            angluar_velocity = (right_speed_ms - left_speed_ms) / track_width_; // rad/s
            
            ros::Time current_time = ros::Time::now();
            double dt = (current_time - last_time_).toSec();
            if(dt < 0) return; // 防止时间倒流
            
            // 航位推算 (Dead Reckoning)
            double delta_theta = angluar_velocity * dt;
            double min_theta = theta + delta_theta / 2.0; // 中值积分
            double delta_x = linear_velocity * std::cos(min_theta) * dt;
            double delta_y = linear_velocity * std::sin(min_theta) * dt;

            x += delta_x;
            y += delta_y;
            theta += delta_theta;
            theta = normalizeAngle(theta); 
            
            last_time_ = current_time;

            // 1. 发布旧格式数据 (保留兼容性)
            msg.data.push_back(current_time.toSec());
            msg.data.push_back(x);
            msg.data.push_back(y);
            msg.data.push_back(theta);
            msg.data.push_back(linear_velocity);
            msg.data.push_back(angluar_velocity);
            odometry_pub_legacy_.publish(msg);
            std::cout <<"time, x, y, theta, " << current_time.toSec()<< ", " << x << ", "<< y << ", " << theta << std::endl;
            std::cout << "dt, right_speed, left_speed, linear_velo, angluar_velo: " << dt << ", "<< right_speed << ", " << left_speed << ", "<< linear_velocity << ", " << angluar_velocity << std::endl;
            // ==========================================
            // 2. 新增：发布 RViz 需要的 TF 和 Odom 消息
            // ==========================================
            // 2.1 将欧拉角 (Yaw) 转换为 四元数 (Quaternion)
            tf2::Quaternion q;
            q.setRPY(0,0,theta);
            geometry_msgs::Quaternion odom_quat = tf2::toMsg(q);
            // // 2.2 广播 TF 变换 (odom -> base_link)
            geometry_msgs::TransformStamped odom_trans;
            odom_trans.header.stamp = current_time;
            odom_trans.header.frame_id = "odom";       // 父坐标系：里程计原点
            odom_trans.child_frame_id = "base_link";   // 子坐标系：机器人底盘中心

            odom_trans.transform.translation.x = x;
            odom_trans.transform.translation.y = y;
            odom_trans.transform.translation.z = 0.0;
            odom_trans.transform.rotation = odom_quat;

            // 发送 TF
            odom_broadcaster_.sendTransform(odom_trans);


            // 2.3 发布 nav_msgs/Odometry 消息
            nav_msgs::Odometry odom;
            odom.header.stamp = current_time;
            odom.header.frame_id = "odom";
            odom.child_frame_id = "base_link";

            // 设置位置 (Pose)
            odom.pose.pose.position.x = x;
            odom.pose.pose.position.y = y;
            odom.pose.pose.position.z = 0.0;
            odom.pose.pose.orientation = odom_quat;

            // 设置速度 (Twist) - 相对于子坐标系 (base_link)
            odom.twist.twist.linear.x = linear_velocity;
            odom.twist.twist.linear.y = 0;
            odom.twist.twist.angular.z = angluar_velocity;

            // 发布消息
            odom_pub_.publish(odom);

            //==========================
            // Pub Path 
            //====================
            geometry_msgs::PoseStamped this_pose_stamped;
            this_pose_stamped.header.stamp = current_time;
            this_pose_stamped.header.frame_id = "odom";
            this_pose_stamped.pose.position.x = x;
            this_pose_stamped.pose.position.y = y;
            this_pose_stamped.pose.position.z = 0;
            this_pose_stamped.pose.orientation = odom_quat;

            path_msg_.header.stamp = current_time;
            path_msg_.header.frame_id = "odom";
            path_msg_.poses.push_back(this_pose_stamped);

            path_pub_.publish(path_msg_);

            // Debug 输出 (可选，注释掉以减少终端刷屏)
            // std::cout <<"x: " << x << ", y: "<< y << ", theta: " << theta << std::endl;
        }

        void static_tf_broadcast(){
            geometry_msgs::TransformStamped base_to_laser;
            geometry_msgs::TransformStamped base_to_letfwheel;
            geometry_msgs::TransformStamped base_to_rightwheel;
            //机器人坐标系到雷达坐标系
            base_to_laser.header.frame_id = "base_link";
            base_to_laser.header.stamp = ros::Time::now();
            base_to_laser.child_frame_id = "laser";

            base_to_laser.transform.translation.x = 0.0;
            base_to_laser.transform.translation.y = 0.0;
            base_to_laser.transform.translation.z = 0.43;

            //四元数
            tf2::Quaternion laser;
            laser.setRPY(0,0,0);
            base_to_laser.transform.rotation.x = laser.getX();
            base_to_laser.transform.rotation.y = laser.getY();
            base_to_laser.transform.rotation.z = laser.getZ();
            base_to_laser.transform.rotation.w = laser.getW();
            static_broadcaster.sendTransform(base_to_laser);
        }
};

int main(int argc, char** argv){
    setlocale(LC_ALL, "");
    ros::init(argc, argv, "can_node");
    Can_ node;
    ros::spin();
    // ros::shutdown(); // spin 退出后会自动 shutdown，显式调用也可以
    return 0;
}