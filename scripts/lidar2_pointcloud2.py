#!/usr/bin/env python
# -*- coding: utf-8 -*- 

import rospy
from sensor_msgs.msg import PointCloud2 as pc2
from sensor_msgs.msg import LaserScan
from laser_geometry import LaserProjection
 
class Laser2PC():
    def __init__(self):
        self.laserProj = LaserProjection()
        self.pcPub = rospy.Publisher("/backlaserPointCloud", pc2, queue_size=10) #转变后发布的点云话题
        self.laserSub = rospy.Subscriber("back_laser_2", LaserScan, self.laserCallback) #接收到的雷达消息
 
    def laserCallback(self,data):
 
        cloud_out = self.laserProj.projectLaser(data)
 
        self.pcPub.publish(cloud_out)
 
if __name__ == '__main__':
    rospy.init_node("backlaserPointCloud")
    l2pc = Laser2PC()
    rospy.spin()