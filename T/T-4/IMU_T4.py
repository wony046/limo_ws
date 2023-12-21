#!/usr/bin/env python
# -*- coding:utf-8 -*-
#imu 센싱
import rospy
import math
from sensor_msgs.msg import Imu
from std_msgs.msg import String, Float32

class limo_imu:
    def __init__(self):
        rospy.init_node('imu', anonymous=True)
        self.linear_acceleration_x = 0
        self.linear_acceleration_z = 0
        self.linear_orientation_z = 0
        self.pitch = 0
        self.start_bump = math.pi / 40
        self.last_bump = -(math.pi / 40)
        self.loop_time = 0
        rospy.Subscriber("/imu", Imu, self.pitch_calculate_callback)
        self.imu_pub = rospy.Publisher("/limo/imu_pitch", String, queue_size=2)
        self.imu_orientation_pub = rospy.Publisher("/limo/imu_roll", Float32, queue_size=2)
    
    def pitch_calculate_callback(self, data):
        #self.wait_time = rospy.get_time()
        self.linear_acceleration_x = data.linear_acceleration.x
        self.linear_acceleration_z = data.linear_acceleration.z
        self.linear_orientation_z = data.orientation.z
        self.imu_orientation_pub.publish(self.linear_orientation_z)
        if ((self.linear_acceleration_x >= 2 or self.linear_acceleration_x <= -2) and self.linear_acceleration_z != 0):
            self.pitch = math.atan(self.linear_acceleration_x / self.linear_acceleration_z)
            if (self.pitch >= self.start_bump or self.pitch <= self.last_bump):
                #if self.wait_time - self.loop_time >= 0.5:
                self.imu_pub.publish("bump")
                    #self.loop_time = rospy.get_time()
                    #self.imu_pub.publish("not_bump")
            else:
                self.imu_pub.publish("not_bump")
        else:
            self.imu_pub.publish("not_bump")

def run():
    new_class = limo_imu()
    rospy.spin()

if __name__ == '__main__':
    try:
        run()
    except rospy.ROSInterruptException:
        pass
