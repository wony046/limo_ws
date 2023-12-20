#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
from std_msgs.msg import Int32
from ar_track_alvar_msgs.msg import AlvarMarkers


# 클래스 생성
class ARMarkerDetection:
    # 클래스 초기화 함수 정의
    def __init__(self):
        # ROS 노드 초기화
        rospy.init_node("ar_marker")
        rospy.Subscriber("/ar_pose_marker", AlvarMarkers, self.marker_CB)
        self.ARstate = rospy.Publisher("/limo/ar_marker", Int32, queue_size=5)
	self.ARtype = -1
	self.marker_distnace = 0

    # 클래스의 메서드 정의
    def marker_CB(self, data):
        # data.markers 문자열의 길이가 0이 아닐 경우 조건문 실행
        if len(data.markers) != 0:
            # data.markers 에 있는 마커 정보를 처리
            for marker in data.markers:
		self.marker_distnace = marker.pose.pose.position.x
		if self.marker_distnace < 1.20:
                    # id가 0번일 경우
                    if marker.id == 0:
                        self.ARtype = 0
                    # id가 1번일 경우
                    elif marker.id == 1:
                        self.ARtype = 1
                    # id가 2번일 경우
                    elif marker.id == 2:
                        self.ARtype = 2
                    # id가 3일 경우
                    elif marker.id == 3:
                        self.ARtype = 3
        # 인식 되지 않았을 경우
        else:
            self.ARtype = -1

	self.ARstate.publish(self.ARtype)


# 메인 메서드 정의
def run():
    new_class = ARMarkerDetection()
    rospy.spin()

if __name__ == '__main__':
    try:
        run()
    except KeyboardInterrupt:
        print("program down")
