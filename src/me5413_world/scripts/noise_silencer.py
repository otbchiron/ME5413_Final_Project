#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import Imu, NavSatFix
from geometry_msgs.msg import Twist

class NoiseSilencer:
    def __init__(self):
        rospy.init_node("noise_silencer")

        self.sensor_en = False

        self.cmd_vel_subscriber = rospy.Subscriber("/cmd_vel", Twist, self.cmd_vel_callback)
        
        # Original topics from gazebo
        self.imu_subscriber = rospy.Subscriber("/imu/data/raw", Imu, self.imu_callback)
        self.gps_subscriber = rospy.Subscriber("/navsat/fix/raw", NavSatFix, self.gps_callback)

        # Silenced topics
        self.imu_publisher = rospy.Publisher("/imu/data", Imu, queue_size=10)
        self.gps_publisher = rospy.Publisher("/navsat/fix", NavSatFix, queue_size=10)

    def imu_callback(self, msg):
        if self.sensor_en:
            self.imu_publisher.publish(msg)
    
    def gps_callback(self, msg):
        if self.sensor_en:
            self.gps_publisher.publish(msg)

    def cmd_vel_callback(self, msg):
        self.sensor_en = False if msg.linear.x == 0.0 and msg.angular.z == 0.0 else True

if __name__ == "__main__":
    noise_silencer = NoiseSilencer()

    rospy.spin()