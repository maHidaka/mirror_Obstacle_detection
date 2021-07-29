#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from geometry_msgs.msg import Point
from visualization_msgs.msg import Marker
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String
from std_msgs.msg import Header

import numpy as np
from matplotlib import pyplot as plt


class Fitting():
    def __init__(self):
        # Subscriberの作成
        self.sub = rospy.Subscriber('scan_L', LaserScan, self.callback)
        # Publisherの作成
        self.pub = rospy.Publisher('fit_L', Marker, queue_size=1)

        self.count = 0

    def callback(self, data):
        fit_func = self.calc_fitting_curve(data)
        marker = self.calc_marker(fit_func, data.header.stamp)
        self.Publisher(marker)

    def Publisher(self, data):
        self.pub.publish(data)

    def calc_fitting_curve(self, data):
        angle = data.angle_min
        x = []
        y = []
        for range in data.ranges:
            angle = angle + data.angle_increment
            position = self.getXY(range, angle)

            if not np.isnan(position[0]):
                x.append(position[0])
                y.append(position[1])
            else:
                print("range value is NaN")

        func = np.polyfit(np.array(x), np.array(y), 1)
        print(func)
        pos = list(func)
        pos.extend([x[0], y[0], x[-1], y[-1]])
        return pos

    def getXY(self, r, rad):
        # 度をラジアンに変換
        x = r * np.cos(rad)
        y = r * np.sin(rad)
        return x, y

    def calc_marker(self, pos, time):
        self.count += 1
        marker_data = Marker()
        marker_data.header.frame_id = "laser"
        marker_data.ns = "soiya"
        marker_data.id = 0
        marker_data.header.stamp = time
        marker_data.action = Marker.ADD

        marker_data.pose.position.x = 0.0
        marker_data.pose.position.y = 0.0
        marker_data.pose.position.z = 0.0

        marker_data.pose.orientation.x = 0.0
        marker_data.pose.orientation.y = 0.0
        marker_data.pose.orientation.z = 0.0
        marker_data.pose.orientation.w = 1.0

        marker_data.color.r = 0.0
        marker_data.color.g = 1.0
        marker_data.color.b = 0.0
        marker_data.color.a = 1.0

        marker_data.scale.x = 0.005
        marker_data.scale.y = 0.005
        marker_data.scale.z = 0.005

        #marker_data.lifetime = 0
        marker_data.lifetime = rospy.Duration()
        marker_data.type = Marker.LINE_STRIP

        marker_data.points = []

        first_point = Point()
        first_point.x = pos[2]
        first_point.y = pos[0]*pos[2]+pos[1]
        first_point.z = 0.0
        marker_data.points.append(first_point)

        second_point = Point()
        second_point.x = pos[4]
        second_point.y = pos[0]*pos[4]+pos[1]
        second_point.z = 0.0
        marker_data.points.append(second_point)

        return marker_data


if __name__ == '__main__':
    rospy.init_node('fitting')
    node = Fitting()
    while not rospy.is_shutdown():
        rospy.sleep(0.1)
