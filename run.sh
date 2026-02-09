
# 1. 定义清理函数：当脚本接收到退出信号时执行
cleanup() {
    echo "接收到关闭信号，正在清理所有后台进程..."
    # kill $(jobs -p) 会杀死当前脚本启动的所有后台进程
    kill $(jobs -p)
    echo "所有进程已关闭。"
}

# 2. 捕捉 SIGINT (Ctrl+C) 和 SIGTERM (kill命令) 信号
# 一旦捕捉到这些信号，立即执行 cleanup 函数
trap cleanup SIGINT SIGTERM


source /opt/ros/noetic/setup.bash
echo "=====启动多点巡航所有程序====="
source devel_isolated/setup.bash
roslaunch robot_bringup drivers.launch &
sleep 5
roslaunch move_base my_robot_pure_nav_syj.launch &
sleep 5
rosrun move_base  multi_patrol_node&
echo "===所有节点已经启动了，脚本正在运行==="
wait
