#!/usr/bin/env python3
# -*- coding:utf-8 -*-

import rospy
from std_msgs.msg import Int32, String, Bool, Float32
from geometry_msgs.msg import Twist
from object_msgs.msg import ObjectArray
from limo_base.msg import LimoStatus
from ar_track_alvar_msgs.msg import AlvarMarkers
from dynamic_reconfigure.server import Server
from limo_application.cfg import controlConfig
import pygame
import math


class LimoController:
    def __init__(self):
        rospy.init_node('limo_control', anonymous=True)
        self.LIMO_WHEELBASE = 0.2
        self.distance_to_ref = 0
        self.crosswalk_detected = False
        self.bool = False
        self.bool2 = False
        self.bool3 = False
        self.marker_id = -1
        self.yolo_object = "green"
        self.e_stop = "Safe"
        self.is_pedestrian_stop_available = True
        self.pedestrian_stop_time = 5.0
        self.pedestrian_stop_last_time = rospy.Time.now().to_sec()
        self.yolo_object_last_time = rospy.Time.now().to_sec()
        self.bbox_size = [0, 0]
        self.distance = 0.0
        self.sound = False
        self.limo_mode = "ackermann"
        self.loop_time = rospy.get_time()
        self.wait_flag = False
        pygame.mixer.init()
        srv = Server(controlConfig, self.reconfigure_callback)
        rospy.Subscriber("limo_status", LimoStatus, self.limo_status_callback)
        rospy.Subscriber("/limo/lane_x", Int32, self.lane_x_callback)
        rospy.Subscriber("/limo/crosswalk_y", Int32, self.crosswalk_y_callback)
        rospy.Subscriber("/limo/yolo_object", ObjectArray, self.yolo_object_callback)
        rospy.Subscriber("/limo/lidar_warning", String, self.lidar_warning_callback)
        rospy.Subscriber("/ar_pose_marker", AlvarMarkers, self.marker_CB)
        self.drive_pub = rospy.Publisher(rospy.get_param("~control_topic_name", "/cmd_vel"), Twist, queue_size=1)
        rospy.Timer(rospy.Duration(0.03), self.drive_callback)

    def play_mp3(self, file_path):
        pygame.mixer.music.load(file_path)
        pygame.mixer.music.play(0)

    # return float
    def calcTimeFromDetection(self, _last_detected_time):
        '''
            마지막 검출 시간부터 흐른 시간 확인하는 함수
        '''
        return rospy.Time.now().to_sec() - _last_detected_time
    def limo_status_callback(self, _data):
        '''
            LIMO의 상태가 Ackermann인지, Differential인지 확인하여, 모드 변경
            최종 출력의 angular.z 값이 달라지므로, 이와 같은 처리가 필요
        '''
        if _data.motion_mode == 1:
            if self.limo_mode == "ackermann":
                pass
            else:
                self.limo_mode = "ackermann"
                rospy.loginfo("Mode Changed --> Ackermann")
        else:
            if self.limo_mode == "diff":
                pass
            else:
                self.limo_mode = "diff"
                rospy.loginfo("Mode Changed --> Differential Drive")

    def lidar_warning_callback(self, _data):
        '''
            장애물 유무 저장
        '''
        self.e_stop = _data.data

    def yolo_object_callback(self, _data):
        '''
            신호등 또는 표지판 상태 저장
            중간에 일부 끊어질 수 있으므로, 마지막으로 인식된 시간도 함께 저장
        '''
        if len(_data.Objects) == 0:
            pass
        else:
            self.yolo_object = _data.Objects[0].class_name
            self.yolo_object_last_time = rospy.Time.now().to_sec()
            self.bbox_size = [_data.Objects[0].xmin_ymin_xmax_ymax[2] - _data.Objects[0].xmin_ymin_xmax_ymax[0], _data.Objects[0].xmin_ymin_xmax_ymax[3] - _data.Objects[0].xmin_ymin_xmax_ymax[1]]  # 왼쪽 상단 x y, 오른쪽 하단 x, y

    def lane_x_callback(self, _data):
        '''
            실제 기준 좌표와 검출된 차선과의 거리 저장
        '''
        if _data.data == -1:
            self.distance_to_ref = 0
        else:
            self.distance_to_ref = self.REF_X - _data.data
            if self.distance_to_ref > 50.0:
                self.distance_to_ref = 0

    def crosswalk_y_callback(self, _data):
        '''
            횡단보도 검출 여부 및 거리 저장
        '''
        if _data.data == -1:
            self.crosswalk_detected = False
            self.crosswalk_distance = _data.data
        else:
            self.crosswalk_detected = True
            self.crosswalk_distance = _data.data

    def reconfigure_callback(self, _config, _level):
        '''
            Dynamic_Reconfigure를 활용
            차량 제어 속도 (BASE_SPEED)
            횡방향 제어 Gain (LATERAL_GAIN)
            차선 기준 좌표 (카메라 픽셀 좌표계 기준) (REF_X)
        '''
        self.BASE_SPEED = _config.base_speed
        self.LATERAL_GAIN = float(_config.lateral_gain * 0.001)
        self.REF_X = _config.reference_lane_x
        self.PEDE_STOP_WIDTH = _config.pedestrian_width_min
        return _config

    def marker_CB(self, data):
        if len(data.markers) != 0:
            for marker in data.markers:
                marker_x = marker.pose.pose.position.x
                marker_y = marker.pose.pose.position.y
                marker_z = marker.pose.pose.position.z
                self.distance = (marker_x **2 + marker_y **2 + marker_z **2)**0.5
                print(self.distance)

                if self.bool == False:
                    self.loop_time = rospy.get_time()
                    self.marker_id = marker.id
                    
    def drive_callback(self, _event):
        drive_data = Twist()
        drive_data.linear.x = self.BASE_SPEED
        drive_data.angular.z = self.distance_to_ref * self.LATERAL_GAIN
        # rospy.loginfo("OFF_CENTER, Lateral_Gain = {}, {}".format(self.distance_to_ref, self.LATERAL_GAIN))
        # rospy.loginfo("Bbox Size = {}, Bbox_width_min = {}".format(self.bbox_size, self.PEDE_STOP_WIDTH))

        try:
            if self.marker_id == 0 and self.distance  < 0.98246:
                pass_time = rospy.get_time() - self.loop_time
                if pass_time > 9:
                    self.bool = False  
                    self.sound = False            
                elif pass_time < 5:
                    self.bool = True
                    drive_data.linear.x = 0.0
                    drive_data.angular.z = 0.0
                    if not self.sound:
                        self.play_mp3('/home/wego/hi.mp3')
                        self.sound = True
                    # print("stop")
            
            elif self.marker_id == 1 and self.distance < 0.86:
                pass_time = rospy.get_time() - self.loop_time
                if pass_time > 3.0:
                    self.bool = False
                    self.sound = False  
                    self.bool3 = False            
                elif pass_time < 1.8:
                    self.bool = True
                    self.bool3 = True
                    drive_data.linear.x = 0.3
                    drive_data.angular.z = 0.0
                    if not self.sound:
                        self.play_mp3('/home/wego/right.mp3')
                        self.sound = True
                elif pass_time > 1.8 and pass_time < 2.85:
                    drive_data.linear.x = 0.0
                    drive_data.angular.z = -1.6
            
            elif self.marker_id == 3 and self.distance < 0.86:
                pass_time = rospy.get_time() - self.loop_time
                if pass_time > 10.0:
                    self.bool = False
                    self.sound = False
                elif pass_time < 1.5:
                    self.bool = True
                    drive_data.linear.x = 0.3
                    drive_data.angular.z = 0.0
                    if not self.sound:
                        self.play_mp3('/home/wego/parking.mp3')
                        self.sound = True
                elif pass_time > 1.5 and pass_time < 2.5:
                    drive_data.linear.x = 0.0
                    drive_data.angular.z = 1.6
                elif pass_time > 2.5 and pass_time < 4.1:
                    drive_data.linear.x = -0.3
                    drive_data.angular.z = 0.0
                elif pass_time > 3.9 and pass_time < 9.6:
                    drive_data.linear.x = 0.0
                    drive_data.angular.z = 0.0
                
            if self.e_stop == "Warning" and self.bool2 == False:
                self.loop_time = rospy.get_time()
            if self.e_stop == "Warning" and self.bool3 == False:
                pass_time = rospy.get_time() - self.loop_time
                if pass_time > 22.5:
                    self.bool2 = False
                elif pass_time < 20:
                    self.bool2 = True
                    drive_data.linear.x = 0.0
                    drive_data.angular.z = 0.0
                elif pass_time > 20 and pass_time < 21.8:
                    drive_data.linear.x = 0.0
                    drive_data.angular.z = -4.0


                
            if self.limo_mode == "diff":
                self.drive_pub.publish(drive_data)
            elif self.limo_mode == "ackermann":
                if drive_data.linear.x == 0:
                    drive_data.angular.z = 0
                else:
                    drive_data.angular.z = \
                        math.tan(drive_data.angular.z / 2) * drive_data.linear.x / self.LIMO_WHEELBASE
                    # 2를 나눈 것은 Differential과 GAIN비율을 맞추기 위함
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
