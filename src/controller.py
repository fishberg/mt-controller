#!/usr/bin/env python2

import numpy as np
import rospy
from sensor_msgs.msg import LaserScan
from ackermann_msgs.msg import AckermannDriveStamped

class Racecar:
    SCAN_TOPIC = '/scan'
    DRIVE_TOPIC = '/drive'

    def __init__(self):
        self.sub_scan = rospy.Subscriber(self.SCAN_TOPIC,
                                         LaserScan,
                                         callback=self.scan_callback)
        self.pub_drive = rospy.Publisher(self.DRIVE_TOPIC,
                                         AckermannDriveStamped,
                                         queue_size=1)


    def scan_callback(self, msg):
        self.scan = msg.ranges

    def drive(self, speed, angle):
        msg = AckermannDriveStamped()
        msg.drive.speed = speed
        msg.drive.steering_angle = angle
        self.pub_drive.publish(msg)

    def stop(self):
        self.drive(0,0)

    def scan(self):
        return self.scan

rospy.init_node('controller')
rate = rospy.Rate(60)
rc = Racecar()

while not rospy.is_shutdown():
    # TODO implement controller logic here
    rc.drive(1,0)
    rate.sleep()
