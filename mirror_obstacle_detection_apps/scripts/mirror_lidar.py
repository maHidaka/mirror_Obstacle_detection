#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from geometry_msgs.msg import Point
from rospy.topics import Publisher
from visualization_msgs.msg import Marker
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String
from std_msgs.msg import Header

import copy
import numpy as np
from matplotlib import pyplot as plt


class MirrorLiDAR():
    def __init__(self):

        self.sub_scan = rospy.Subscriber('scan', LaserScan, self.callback)
        self.pub_scan_front = rospy.Publisher(
            'scan_front', LaserScan, queue_size=1)
        self.pub_marker = rospy.Publisher('fit_line', Marker, queue_size=1)

    def callback(self, data):
        front_data = self.divide_scan_data(data, -1, 1)
        right_data = self.divide_scan_data(data, 1.6, 2.09)
        left_data = self.divide_scan_data(data, -2.05, -1.5)
        self.Publisher(left_data)

    def Publisher(self, data):
        self.pub_scan_front.publish(data)

    def divide_scan_data(self, data, begin, end):
        input_data = copy.deepcopy(data)
        print(len(data.ranges))
        angle_rates = np.arange(
            input_data.angle_min, input_data.angle_max, input_data.angle_increment)
        target_index = np.where((angle_rates >= begin) & (angle_rates <= end))
        target_index = target_index[0]
        target_ranges = input_data.ranges[target_index[0]:target_index[-1]]
        input_data.ranges = target_ranges
        input_data.angle_min = begin
        input_data.angle_max = end
        return input_data

    def calc_fitting_curve(self, data):
        return soiya

    def calc_marker(self, data):
        return soiya

    def convert_3d(self, data):
        return soiya

    def calc_base_vector(self, data):
        return soiya

    def coordinate_transform(self, data):
        return soiya


if __name__ == '__main__':
    rospy.init_node('fitting')
    node = MirrorLiDAR()
    while not rospy.is_shutdown():
        rospy.sleep(0.1)
