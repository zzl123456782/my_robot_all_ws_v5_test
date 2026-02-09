# catkin_make_isolated --install --use-ninja

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

echo "===cartographer 建图==="
source devel_isolated/setup.bash
roslaunch robot_bringup drivers.launch &
sleep 5
roslaunch cartographer_ros self_backpack_2d.launch