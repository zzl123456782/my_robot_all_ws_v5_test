#!/usr/bin/env python3
import rospy
import numpy as np
from sensor_msgs.msg import PointCloud2
from nav_msgs.msg import OccupancyGrid
from std_msgs.msg import Header
import sensor_msgs.point_cloud2 as pc2

class PointCloudToGridMap:
    def __init__(self):
        rospy.init_node('pointcloud_to_gridmap')
        
        # 参数配置
        self.resolution = rospy.get_param('~resolution', 0.05)  # 地图分辨率 (m/cell)
        self.width = rospy.get_param('~width', 200)            # 地图宽度 (cells)
        self.height = rospy.get_param('~height', 200)           # 地图高度 (cells)
        self.z_min = rospy.get_param('~z_min', -0.1)           # 最低有效高度 (m)
        self.z_max = rospy.get_param('~z_max', 0.5)             # 最高有效高度 (m)
        
        # 初始化栅格地图
        self.grid_map = OccupancyGrid()
        self.grid_map.header.frame_id = "map"
        self.grid_map.info.resolution = self.resolution
        self.grid_map.info.width = self.width
        self.grid_map.info.height = self.height
        self.grid_map.info.origin.position.x = -self.width * self.resolution / 2.0
        self.grid_map.info.origin.position.y = -self.height * self.resolution / 2.0
        self.grid_map.data = [-1] * (self.width * self.height)
        
        # 订阅点云话题 (根据你的Fast-LIO配置)
        self.pc_sub = rospy.Subscriber("/cloud_registered", PointCloud2, self.pc_callback)
        
        # 发布地图话题
        self.map_pub = rospy.Publisher("/map", OccupancyGrid, queue_size=1, latch=True)
        
    def pc_callback(self, msg):
        try:
            # 处理点云数据
            points = list(pc2.read_points(msg, skip_nans=True, field_names=("x", "y", "z")))
            
            # 重置地图为未知
            self.grid_map.data = [-1] * (self.width * self.height)
            self.grid_map.header.stamp = rospy.Time.now()
            
            # 将点云投影到2D栅格地图
            for (x, y, z) in points:
                if self.z_min <= z <= self.z_max:  # 高度过滤
                    # 坐标转换到地图坐标系
                    grid_x = int((x - self.grid_map.info.origin.position.x) / self.resolution)
                    grid_y = int((y - self.grid_map.info.origin.position.y) / self.resolution)
                    
                    # 在地图范围内
                    if 0 <= grid_x < self.width and 0 <= grid_y < self.height:
                        index = grid_y * self.width + grid_x
                        self.grid_map.data[index] = 100  # 占据点
            
            self.map_pub.publish(self.grid_map)
            
        except Exception as e:
            rospy.logerr("点云处理错误: %s" % str(e))

if __name__ == '__main__':
    converter = PointCloudToGridMap()
    rospy.spin()