#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from sensor_msgs.msg import PointCloud2 as pc2
from sensor_msgs.msg import LaserScan
from laser_geometry import LaserProjection
 
class Laser2PC():
    def __init__(self):
        self.laserProj = LaserProjection()
        self.pcPub = rospy.Publisher("/frontlaserPointCloud", pc2, queue_size=10) #转变后发布的点云话题
        self.laserSub = rospy.Subscriber("scan_filtered", LaserScan, self.laserCallback) #接收到的雷达消息
 
    def laserCallback(self,data):
 
        cloud_out = self.laserProj.projectLaser(data)
 
        self.pcPub.publish(cloud_out)
 
if __name__ == '__main__':
    rospy.init_node("frontlaser2PointCloud")
    l2pc = Laser2PC()
    rospy.spin()