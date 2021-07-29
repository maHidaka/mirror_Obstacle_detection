#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from geometry_msgs.msg import PoseStamped
from visualization_msgs.msg import Marker
from rospy.names import canonicalize_name
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String
from std_msgs.msg import Header
from nav_msgs.msg import Path

import numpy as np
from matplotlib import pyplot as plt


class Fitting():
    def __init__(self):
        # Subscriberの作成
        self.sub = rospy.Subscriber('scan_L', LaserScan, self.callback)
        # Publisherの作成
        self.pub = rospy.Publisher('fit_L', Marker, queue_size=1)

    def callback(self, data):
        # callback関数の処理をかく
        # print(data)
        marker = self.calc_fitting_curve(data)

        print(marker)
        #marker.header.stamp = data.header.stamp
        # self.Publisher(marker)

    def Publisher(self, data):
        self.pub.publish(data)

    def calc_fitting_curve(self, data):
        angle = data.angle_min
        x = np.empty(0)
        y = np.empty(0)
        for range in data.ranges:
            angle = angle + data.angle_increment
            #print(range, angle)
            #print(self.getXY(range, angle))
            position = self.getXY(range, angle)
            x.append(position[0])
            y.append(position[1])

    def getXY(self, r, rad):
        # 度をラジアンに変換
        x = r * np.cos(rad)
        y = r * np.sin(rad)
        return x, y

    def calc_marker(self, position, id, time):
        marker_data = Marker()
        marker_data.header.frame_id = "laser"
        marker_data.ns = "soiya"
        marker_data.id = id
        marker_data.header.stamp = time
        marker_data.action = Marker.ADD

        marker_data.pose.position.x = position[0]
        marker_data.pose.position.y = position[1]
        marker_data.pose.position.z = 0.0

        marker_data.pose.orientation.x = 0.0
        marker_data.pose.orientation.y = 0.0
        marker_data.pose.orientation.z = 1.0
        marker_data.pose.orientation.w = 0.0

        marker_data.color.r = 1.0
        marker_data.color.g = 0.0
        marker_data.color.b = 0.0
        marker_data.color.a = 1.0

        marker_data.scale.x = 0.1
        marker_data.scale.y = 0.01
        marker_data.scale.z = 0.01

        marker_data.lifetime = rospy.Duration()
        marker_data.type = 0
        #marker_data.header.stamp = data.header.stamp
        return marker_data


if __name__ == '__main__':
    rospy.init_node('fitting')
    node = Fitting()
    while not rospy.is_shutdown():
        rospy.sleep(0.1)
