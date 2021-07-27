#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Header
from nav_msgs.msg import Path

import numpy as np
from matplotlib import pyplot as plt


class Fitting():
    def __init__(self):
        # Subscriberの作成
        self.sub = rospy.Subscriber('scan_L', String, self.callback)
        # Publisherの作成
        self.pub = rospy.Publisher('fit_L', String, queue_size=1)

    def callback(self, data):
        # callback関数の処理をかく

        Publisher(data)

    def Publisher(self, data):
        self.pub.publish(data)


if __name__ == '__main__':
    rospy.init_node('fitting')

    time.sleep(3.0)
    node = Fitting()

    while not rospy.is_shutdown():
        rospy.sleep(0.1)
