#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from geometry_msgs.msg import Point
from rospy.topics import Publisher
from visualization_msgs.msg import Marker
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Header

import copy
import numpy as np


class MirrorLiDAR():
    def __init__(self):

        self.sub_scan = rospy.Subscriber('scan', LaserScan, self.callback)
        self.pub_scan_front = rospy.Publisher(
            'scan_front', LaserScan, queue_size=1)
        self.pub_scan_right = rospy.Publisher(
            'scan_right', LaserScan, queue_size=1)
        self.pub_scan_left = rospy.Publisher(
            'scan_left', LaserScan, queue_size=1)
        self.pub_fit_r = rospy.Publisher('fit_line_R', Marker, queue_size=1)
        self.pub_fit_l = rospy.Publisher('fit_line_L', Marker, queue_size=1)

    def callback(self, data):
        front_data = self.divide_scan_data(data, -1, 1)
        right_data = self.divide_scan_data(data, 1.6, 2.09)
        left_data = self.divide_scan_data(data, -2.05, -1.5)

        # calc fit line L,R
        func_R, x_R = self.calc_fitting_curve(right_data)
        line_pos_R = self.calc_function(func_R, x_R[0], x_R[1])
        #fit_r = self.calc_marker(line_pos_R, data.header.stamp)
        func_L, x_L = self.calc_fitting_curve(left_data)
        line_pos_L = self.calc_function(func_L, x_L[0], x_L[1])
        #fit_l = self.calc_marker(line_pos_L, data.header.stamp)

        bottom_r = self.convert_3d(func_R, x_R[0], x_R[1], 1)
        fit_r = self.calc_marker(bottom_r[0], data.header.stamp)

        bottom_l = self.convert_3d(func_L, x_L[0], x_L[1], -1)
        fit_l = self.calc_marker(bottom_l[0], data.header.stamp)

        A = self.calc_base_vector(bottom_r[1], bottom_l[1])
        print(A)

        self.pub_scan_front.publish(front_data)
        self.pub_scan_right.publish(right_data)
        self.pub_scan_left.publish(left_data)
        self.pub_fit_r.publish(fit_r)
        self.pub_fit_l.publish(fit_l)

    def Publisher(self, front, right, left):
        self.pub_scan_front.publish(front)
        self.pub_scan_right.publish(right)
        self.pub_scan_left.publish(left)


#   divide_scan_data
#   スキャンデータから取得角度に基づいてターゲット範囲のスキャンデータを取り出す
#   引数：divide_scan_data(LaserScan data, float begin, float end)
#       data    LiDARのスキャン生データ
#       begin   ターゲットの開始角度
#       end     ターゲットの終了角度
#   返り値: LaserScan input_data
#

    def divide_scan_data(self, data, begin, end):
        input_data = copy.deepcopy(data)
        angle_rates = np.arange(
            input_data.angle_min, input_data.angle_max, input_data.angle_increment)
        target_index = np.where((angle_rates >= begin) & (angle_rates <= end))
        target_index = target_index[0]
        target_ranges = input_data.ranges[target_index[0]:target_index[-1]]
        input_data.ranges = target_ranges
        input_data.angle_min = begin
        input_data.angle_max = end
        return input_data


#   getXY
#   極座標から直行座標へ変換
#   引数：getXY(any r, any rad)
#       r       動径
#       rad     極角(radian)
#   返り値: float x, float y
#       x   x座標
#       y   y座標
#

    def getXY(self, r, rad):
        x = r * np.cos(rad)
        y = r * np.sin(rad)
        return x, y

#   calc_fitting_curve
#   与えられたスキャンデータを一次最小二乗法でフィッテング
#   引数：calc_fitting_curve(LaserScan data)
#       data    フィッティングしたい点群を含むスキャンデータ
#   返り値:list func[float slope,float intercept], list pos[float x_begin, float x_end]
#       func[slope, intercept]      フィッティングした関数の傾きslopeと切片interceptをリストで返す
#       pos[x_begin, x_end]         近似直線の表示用にスキャンデータの直交座標系でのX座標の始点と終点をリストで返す
#
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

        fit_line = np.polyfit(np.array(x), np.array(y), 1)
        #pos = list(fit_line)
        #pos.extend([x[0], x[-1]])
        func = list(fit_line)
        pos = [x[0], x[-1]]

        return func, pos


#   calc_function
#   一次関数の傾きと切片、x座標からy座標を求める
#   引数：calc_function(list func[slope, intercpt], list x[x_begin, x_end])
#       data    フィッティングしたい点群を含むスキャンデータ
#   返り値:list func[float slope,float intercept], list pos[float x_begin, float x_end]
#       func[slope, intercept]      フィッティングした関数の傾きslopeと切片interceptをリストで返す
#       pos[x_begin, x_end]         近似直線の表示用にスキャンデータの直交座標系でのX座標の始点と終点をリストで返す
#


    def calc_function(self, func, x):
        a = func[0]
        b = func[1]

        y = a * x + b

        return y


#   calc_marker
#   markerトピックのデータ作成
#   引数：calc_marker( list pos[x1,y1,z1,x2,y2,z2], time)
#       data    フィッティングしたい点群を含むスキャンデータ
#   返り値:list func[float slope,float intercept], list pos[float x_begin, float x_end]
#       func[slope, intercept]      フィッティングした関数の傾きslopeと切片interceptをリストで返す
#       pos[x_begin, x_end]         近似直線の表示用にスキャンデータの直交座標系でのX座標の始点と終点をリストで返す
#

    def calc_marker(self, pos, std_msgs/Header headertime):
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

        marker_data.lifetime = rospy.Duration()
        marker_data.type = Marker.LINE_STRIP

        marker_data.points = []

        first_point = Point()
        first_point.x = pos[0]
        first_point.y = pos[1]
        first_point.z = pos[2]
        marker_data.points.append(first_point)

        second_point = Point()
        second_point.x = pos[3]
        second_point.y = pos[4]
        second_point.z = pos[5]
        marker_data.points.append(second_point)
        return marker_data

    def convert_3d(self, func, pos1, pos2, dir):
        mirror_d = 0.04 * dir
        a = func[0] * dir
        b = (func[1] - mirror_d) * dir

        x1 = pos1
        y1 = mirror_d
        z1 = a * pos1 + b
        x2 = pos2
        y2 = mirror_d
        z2 = a * pos2 + b
        position = [x1, y1, z1, x2, y2, z2]
        func_3d = [a, b]

        return position, func_3d

    def calc_base_vector(self, func_R, func_L):
        mirror_d = 0.04
        Ar = func_R[0]
        Br = func_R[1]
        Al = func_L[0]
        Bl = func_L[1]
        Ax = (Ar + Al)/2
        ytan = -Ax
        th_y = np.degrees(np.arctan(ytan))

        xtan = (Br - Bl) / (2*mirror_d)
        th_x = np.degrees(np.arctan(xtan))

        return th_x, th_y

    def coordinate_transform(self, front_scan_data, th_x, th_y):

        return soiya


if __name__ == '__main__':
    rospy.init_node('fitting')
    node = MirrorLiDAR()
    while not rospy.is_shutdown():
        rospy.sleep(0.1)
