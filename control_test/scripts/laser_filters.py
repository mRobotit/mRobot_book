#!/usr/bin/env  python

from __future__ import print_function
import sys
import rospy
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String

class ScanFilter:   
    def __init__(self):

        self.pub = rospy.Publisher("scan", LaserScan, queue_size = 10)
        self.newdata = LaserScan()
        self.newdata.ranges = [0] * 360
        self.newdata.intensities = [0] * 360
        self.newdata.header.frame_id = "laser_link"

        self.newdata.angle_min = -1.57
        self.newdata.angle_max = 1.57
        self.newdata.angle_increment = 3.14 * 2 / 360
        self.newdata.time_increment = (1 / 100) / (360)
        self.newdata.range_min = 0.0
        self.newdata.range_max = 100.0

    def pub(self):
        self.newdata.header.stamp = rospy.Time.now()
        for x in range(0, 360):
            self.newdata.ranges[x] = 1
            self.newdata.intensities[x] = 1

        self.pub.publish(self.newdata)

if __name__ == "__main__":
    rospy.init_node("scan_filter", anonymous = True)
    r = rospy.Rate(100)
    lidar = ScanFilter()

    while not rospy.is_shutdown():
        lidar.pub()
        r.sleep()