#!/usr/bin/env python
# -*- coding:utf-8 -*-
# limo_application/scripts/lane_detection/control.py
# WeGo LIMO Pro를 이용한 주행 코드

import rospy

from std_msgs.msg import Int32, String, Bool, Float32
from geometry_msgs.msg import Twist
from object_msgs.msg import ObjectArray
from ar_track_alvar_msgs.msg import AlvarMarkers
from limo_base.msg import LimoStatus
from dynamic_reconfigure.server import Server
from limo_application.cfg import controlConfig
from playsound import playsound
import time
import math


class LimoController:
    '''
    차선 인식, 횡단보도 인식, LiDAR 기반 장애물 인식, YOLO 기반 신호등 및 표지판 인식
    위 기능을 통합한 전체 주행 코드
    Private Params --> control_topic_name
    < Subscriber >
    limo_status (LimoStatus) --> LIMO의 Motion Model 확인용
    /limo/lane_x (Int32) --> 인식된 차선 (카메라 좌표계 x)
    /limo/crosswalk_y (Int32) --> 인식된 횡단보도 (카메라 좌표계 y)
    /limo/traffic_light (String) --> YOLO 기반 인식 결과
    /limo/lidar_warning (String) --> LiDAR 기반 장애물 검출 결과
    < Publisher >
    /cmd_vel (Twist) --> Default 출력, LIMO 제어를 위한 Topic
    '''

    def __init__(self):
        rospy.init_node('limo_control', anonymous=True)
        self.LIMO_WHEELBASE = 0.2
        self.distance_to_ref = 0
        self.crosswalk_detected = False
        self.yolo_object = "green"
        self.e_stop = "Safe"
        self.is_pedestrian_stop_available = True
        self.pedestrian_stop_time = 5.0
        self.pedestrian_stop_last_time = rospy.Time.now().to_sec()
        self.yolo_object_last_time = rospy.Time.now().to_sec()
        self.bbox_size = [0, 0]
        self.marker_id = -1
        self.bool = False
        self.first_time = rospy.get_time()
        self.limo_mode = "ackermann"

        srv = Server(controlConfig, self.reconfigure_callback)
        rospy.Subscriber("/ar_pose_marker", AlvarMarkers, self.marker_CB)
        rospy.Subscriber("limo_status", LimoStatus, self.limo_status_callback)
        rospy.Subscriber("/limo/lane_x", Int32, self.lane_x_callback)
        rospy.Subscriber("/limo/crosswalk_y", Int32, self.crosswalk_y_callback)
        rospy.Subscriber("/limo/yolo_object", ObjectArray, self.yolo_object_callback)
        rospy.Subscriber("/limo/lidar_warning", String, self.lidar_warning_callback)
        self.drive_pub = rospy.Publisher(rospy.get_param("~control_topic_name", "/cmd_vel"), Twist, queue_size=1)
        rospy.Timer(rospy.Duration(0.03), self.drive_callback)

    def calcTimeFromDetection(self, _last_detected_time):
        return rospy.Time.now().to_sec() - _last_detected_time

    def marker_CB(self, data):
        if len(data.markers) != 0:
            for marker in data.markers:
                marker_x = marker.pose.pose.position.x
                marker_y = marker.pose.pose.position.y
                marker_z = marker.pose.pose.position.z
                self.distance = (marker_x ** 2 + marker_y ** 2 + marker_z ** 2) ** 0.5
                print(self.distance)

                if self.bool == False:
                    self.first_time = rospy.get_time()
                    self.marker_id = marker.id

    # ==============================================
    #               Callback Functions
    # ==============================================

    def limo_status_callback(self, _data):
        if _data.motion_mode == 1:
            if self.limo_mode == "ackermann":
                pass
            else:
                self.limo_mode = "ackermann"
                # rospy.loginfo("Mode Changed --> Ackermann")
        else:
            if self.limo_mode == "diff":
                pass
            else:
                self.limo_mode = "diff"
                # rospy.loginfo("Mode Changed --> Differential Drive")

    def lidar_warning_callback(self, _data):
        self.e_stop = _data.data

    def yolo_object_callback(self, _data):
        if len(_data.Objects) == 0:
            pass
        else:
            self.yolo_object = _data.Objects[0].class_name
            self.yolo_object_last_time = rospy.Time.now().to_sec()
            self.bbox_size = [_data.Objects[0].xmin_ymin_xmax_ymax[2] - _data.Objects[0].xmin_ymin_xmax_ymax[0],
                              _data.Objects[0].xmin_ymin_xmax_ymax[3] - _data.Objects[0].xmin_ymin_xmax_ymax[
                                  1]]  # 왼쪽 상단 x y, 오른쪽 하단 x, y

    def lane_x_callback(self, _data):
        if _data.data == -1:
            self.distance_to_ref = 0
        else:
            self.distance_to_ref = self.REF_X - _data.data

    def crosswalk_y_callback(self, _data):
        if _data.data == -1:
            self.crosswalk_detected = False
            self.crosswalk_distance = _data.data
        else:
            self.crosswalk_detected = True
            self.crosswalk_distance = _data.data

    def reconfigure_callback(self, _config, _level):
        self.BASE_SPEED = _config.base_speed
        self.LATERAL_GAIN = float(_config.lateral_gain * 0.001)
        self.REF_X = _config.reference_lane_x
        self.PEDE_STOP_WIDTH = _config.pedestrian_width_min
        return _config

    def drive_callback(self, _event):
        drive_data = Twist()
        drive_data.linear.x = self.BASE_SPEED
        drive_data.angular.z = self.distance_to_ref * self.LATERAL_GAIN
        rospy.loginfo("로봇의 선속도: {}, 회전 속도: {}".format(drive_data.linear.x, drive_data.angular.z))

        # rospy.loginfo("OFF_CENTER, Lateral_Gain = {}, {}".format(self.distance_to_ref, self.LATERAL_GAIN))
        # rospy.loginfo("Bbox Size = {}, Bbox_width_min = {}".format(self.bbox_size, self.PEDE_STOP_WIDTH))

        try:

            if self.marker_id == 0 and self.distance < 1.1:
                pass_time = rospy.get_time() - self.first_time
                if pass_time > 10:
                    self.bool = False
                elif pass_time < 5.5:
                    self.bool = True
                    drive_data.linear.x = 0.0
                    drive_data.angular.z = 0.0
                    print("stop")
                    # MP3 파일 재생
                    playsound('/home/wego/Desktop/정지.mp3')
                    time.sleep(5)

            '''
            # 우회전 세번쨰 위치(우회전 한번)(곡선)
            if self.marker_id == 1 and self.distance < 0.85:
                pass_time = rospy.get_time() - self.first_time
                if pass_time >1.2:
                    self.bool = False
                elif pass_time < 1.2:
                    self.bool = True
                    drive_data.linear.x = 0.6
                    drive_data.angular.z = -1.25
                    print("turn right")
            '''
            # 우회전 세번쨰 위치(우회전 한번)(제자리 회전)
            if self.marker_id == 1 and self.distance < 0.875:
                pass_time = rospy.get_time() - self.first_time
                if pass_time > 5.5:
                    self.bool = False
                elif pass_time < 2.5:
                    self.bool = True
                    drive_data.linear.x = 0.2
                    drive_data.angular.z = 0.0
                elif pass_time < 3.6:
                    self.bool = True
                    drive_data.linear.x = 0.0
                    drive_data.angular.z = -1.55
                elif pass_time < 5.5:
                    self.bool = True
                    drive_data.linear.x = 0.3
                    drive_data.angular.z = 0.0

                    print("turn right")

            # 우회전 직후 주차
            if self.marker_id == 3 and self.distance < 1.4:
                pass_time = rospy.get_time() - self.first_time
                if pass_time > 4.5:
                    self.bool = False
                elif pass_time < 0.5:
                    self.bool = True
                    drive_data.linear.x = 0.4
                    drive_data.angular.z = 0.0
                elif pass_time < 1.5:
                    self.bool = True
                    drive_data.linear.x = 0.0
                    drive_data.angular.z = 1.5
                elif pass_time < 2.75:
                    self.bool = True
                    drive_data.linear.x = -0.3
                    drive_data.angular.z = 0.0
                elif pass_time < 4.5:
                    self.bool = True
                    drive_data.linear.x = 0.0
                    drive_data.angular.z = 0.0
                    print("parking")
                    # MP3 파일 재생

            '''
            # 주차 앞에 좌회전 마커 있을 때
            if self.marker_id == 3 and self.distance < 0.8:
                pass_time = rospy.get_time() - self.first_time
                if pass_time > 3.25:
                    self.bool = False
                elif pass_time < 1:
                    self.bool = True
                    drive_data.linear.x = 0.4
                    drive_data.angular.z = -1.5
                elif pass_time < 1.5:
                    self.bool = True
                    drive_data.linear.x = 0.5
                    drive_data.angular.z = 0.0
                elif pass_time < 2.5:
                    self.bool = True
                    drive_data.linear.x = 0.0
                    drive_data.angular.z = -4.5
                elif pass_time < 3.0:
                    self.bool = True
                    drive_data.linear.x = 0.0
                    drive_data.angular.z = 0.0
                    # MP3 파일 재생
            '''

            if self.e_stop == "Warning":
                drive_data.linear.x = 0.0
                drive_data.angular.z = 0.0
                # rospy.logwarn("Obstacle Detected, Stop!")
                playsound('/home/wego/Desktop/tts limo.mp3')

            if self.limo_mode == "diff":
                self.drive_pub.publish(drive_data)
            elif self.limo_mode == "ackermann":
                if drive_data.linear.x == 0:
                    drive_data.angular.z = 0
                else:
                    drive_data.angular.z = \
                        math.tan(drive_data.angular.z / 2) * drive_data.linear.x / self.LIMO_WHEELBASE
                    self.drive_pub.publish(drive_data)


        except Exception as e:
            rospy.logwarn(e)


def run():
    new_class = LimoController()
    rospy.spin()


if __name__ == '__main__':
    try:
        run()
    except KeyboardInterrupt:
        print("program down")
