更新时间：2026.1.09

#项目介绍
    1、本项目通过使用cartographer中的纯定位功能替换move_base中的amcl定位模块，实现了使用cartographer进行导航的功能；
    2、本项目使用pp局部路径规划算法，替换了move_base中常用的几个局部路径规划算法；
    3、可通过.sh文件实现一键启动功能（建图、定位、导航）；



#命令运行
    cartographer导航：. cartographer_localization.sh & roslaunch move_base my_robot_pure_nav_syj.launch
    amcl导航：. normal_nav.sh & roslaunch move_base use_my_local_planner_nav.launch
    建图：. map_2d.sh & roslaunch cartographer_ros self_backpack_2d.launch
    定位：. localization_2d.sh & roslaunch cartographer_ros self_backpack_2d_localization.launch


2026.01.12
#测试问题
    检测到障碍物后，没有触发全局重规划功能；


更新时间 ：2026.01.13
#修改
    1、nav::BaseLocalPlanner 添加障碍物区域判断函数ShouldReplan();
    2、在mylocalplanner中给判断函数写逻辑；
    3、在move_base 的controlling状体下，判断障碍物函数指针的真假，为真的话就回溯到planning状态，再次进行规划

更新时间 ：2026.01.19
#修改
    1、在move_base 的controlling状体下，判断障碍物函数指针的真假，为真的话不再回溯到planning状态，直接触发规划线程再次进行规划；
    2、在global_costmap配置文件中，修改为原来仿真的配置，相当于加入一个障碍物层给静态地图的规划里；


更新时间 ：2026.01.23
#修改
    1、my_local_planner代码中修改了躲避障碍物策略，修复了跟随的局部目标点角度变化太大时，转圈的现象
    2、加入了底盘启动文件（雷达、imu、底盘control）；

更新时间：2026.01.30
#修改
    1、编写了一个cartographer_map.sh 建图一键启动程序；
    2、编写了一个run.sh 多点导航一键启动程序；(启动不了的话，可逐步启动 . start_drivers.sh  、 . cartrographer_localization.sh 、 rosrun move_base test_switch 命令)
    3、将cartographer_ros中的map_2d.sh取出来了；

    更新时间：2026.02.03
#修改
    1、修改多点导航程序，添加了目标点切换时间以及发布目标点时间频率

    更新时间：2026.02.28
#修改
    1、修改自制局部路径规划中的前坛点数（从30降低到了15）；
    2、修改了自己局部路进规划中的判断高代价地区的条件，去除掉了cost>200的情况，只有在cost>=253时触发；   
    3、修改了多点巡航代码，添加了目标点到达的话题发布以及人声相应的标志位检测；
    4、降低了局部代价地图的膨胀半径inflation_radius从0.55降低到0.35，缩短避障时的安全距离；

更新时间：2026.03.01
#地图构建
    1、记得修改仿真时间为false
    2、地图保存命令(
        rosservice call /write_state "{filename: '/home/xiaocaiji/zzl/robot/my_robot_v06/src/cartographer_relocalization/src/cartographer_ros/cartographer_ros/map/2d.pbstream', include_unfinished_submaps: true}"
    )
    3、直接保存的地图不能在move_base中直接使用，需要使用命令转化一下(
        rosrun cartographer_ros cartographer_pbstream_to_ros_map \-pbstream_filename (地图名字) \-map_filestem (转化的地图名字) \-resolution 0.05
    )
#修改
    1、move_base源码中添加了触发恢复行为的行为标志id发布；
    2、move_base源码中修改了重规划的逻辑，注释了重置计数的操作，避免影响恢复行为触发；
    
