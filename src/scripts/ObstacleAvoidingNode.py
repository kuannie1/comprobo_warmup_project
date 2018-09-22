#!/usr/bin/env python
from __future__ import print_function, division
from std_msgs.msg import Header
import rospy
import rviz
from geometry_msgs.msg import Vector3, Point, PointStamped, Twist
from nav_msgs.msg import Odometry
import math 
import tf.transformations as tft
from tf.transformations import euler_from_quaternion, quaternion_from_euler
import numpy as np
from sensor_msgs.msg import LaserScan


class ObstacleAvoidingNode(object):
    def __init__(self):
        rospy.init_node('ObstacleAvoidingNode')
        self.pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.sub_laser = rospy.Subscriber('/scan', LaserScan, self.process_scan)
        self.distances = None


    def process_scan(self, m):
        """ Takes in the messages & assigns all the class variables """
        self.distances = m.ranges
    
    def rotate(self, angle):
        # make sure you 'break' out of it when the acceptable angle is reached
        """ Calculates the angle needed to twist continuously """
        currentTime = rospy.Time.now()
        stopTime = currentTime + rospy.Duration(math.radians(angle))
        while currentTime<stopTime:
            m = Twist(angular=Vector3(x=0, y= 0, z=1))
            currentTime = rospy.Time.now()
            self.pub.publish(m)
        self.pub.publish(Twist())

    def has_obstacle(self, first_group, second_group):
        total_list = first_group + second_group
        for element in total_list:
            if (element != 0) and (element <= 1.0):
                return True
        return False

    def move_tentatively(self):
        currentTime = rospy.Time.now()
        stopTime = currentTime + rospy.Duration(0.5)
        while (currentTime < stopTime):
            currentTime = rospy.Time.now()
            self.pub.publish(Twist(linear=Vector3(x=.1, y=0,z=0)))
        return
        

    def run(self):
        """ Performs a clockwise 90-deg turn and goes forward until the object isn't detected anymore, then turn a counterclockwise 90 then go forward"""
        r = rospy.Rate(10)
        while not rospy.is_shutdown():
            if self.distances != None:
                first_group = self.distances[:45]
                second_group = self.distances[-45:]
                if self.has_obstacle(first_group, second_group):
                    self.rotate(90)
                    self.move_tentatively()
                else:
                    self.pub.publish(Twist(linear=Vector3(x=.2, y=0,z=0)))
                r.sleep()

if __name__ == '__main__':
    node = ObstacleAvoidingNode()
    node.run()