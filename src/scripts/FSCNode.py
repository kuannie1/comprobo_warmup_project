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
from neato_node.msg import Bump 


class FSCNode(object):
    def __init__(self):
        rospy.init_node('FSCNode')
        self.pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.sub_laser = rospy.Subscriber('/scan', LaserScan, self.process_scan_lidar)
        self.sub_bump = rospy.Subscriber('/bump', Bump, self.process_scan_bump)
        self.distances = None
        self.bump = False

    def process_scan_bump(self, m):
        bump1 = m.leftSide
        bump2 = m.rightSide
        bump3 = m.leftFront
        bump4 = m.rightFront
        self.bump = bump1 or bump2 or bump3 or bump4
    
    def process_scan_lidar(self, m):
        self.distances = m.ranges

    def draw_square(self):
        # have a while loop that breaks when the bump sensor is triggered
        self.bump = False
        while not self.bump:
            self.moveStraight()
            self.rotate(90)
        print("transitioning to obstacle_avoidance")
        return

    def moveStraight(self):
        currentTime = rospy.Time.now()
        stopTime = currentTime+rospy.Duration(3)
        while currentTime<stopTime and (not self.bump):
            print("moveStraight()")
            m = Twist(linear = Vector3(x=0.2, y=0, z=0))
            currentTime = rospy.Time.now()
            self.pub.publish(m)
        self.pub.publish(Twist())
    
    def rotate(self, angle):
        # make sure you 'break' out of it when the acceptable angle is reached
        """ Calculates the angle needed to twist continuously """
        currentTime = rospy.Time.now()
        stopTime = currentTime + rospy.Duration(math.radians(angle))
        # check for bump sensor in below while loop
        while (currentTime<stopTime and (not self.bump)):
            print("rotate()")
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

    def obstacle_avoidance(self):
        # have a while loop that breaks when the bump sensor is triggered
        print("made it to obstacle_avoidance()")
        r = rospy.Rate(10)
        self.bump = False
        while not rospy.is_shutdown():
            if self.bump:
                return
            if self.distances != None:
                first_group = self.distances[:45]
                second_group = self.distances[-45:]
                if self.has_obstacle(first_group, second_group):
                    print("obstacle_avoidance detected an obstacle")
                    self.rotate(90)
                    self.move_tentatively()
                else:
                    print("obstacle_avoidance moving forward")
                    self.pub.publish(Twist(linear=Vector3(x=.2, y=0,z=0)))
            r.sleep()
        # return

    def run(self):
        # r = rospy.Rate(10)
        while not rospy.is_shutdown():
            self.draw_square()
            self.obstacle_avoidance()
            # r.sleep()


if __name__ == '__main__':
    node = FSCNode()
    node.run()