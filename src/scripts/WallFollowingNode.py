#!/usr/bin/env python
from __future__ import print_function, division
from std_msgs.msg import Header
import rospy
import rviz
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist, Vector3
import math

class WallFollowingNode(object):
    """ This node moves forward until it detects an obstacle, rotates itself to be parallel to the obstacle, and moves forward again """
    def __init__(self):
        rospy.init_node('wall_follower_node')
        self.sub = rospy.Subscriber('/scan', LaserScan, self.process_scan)
        self.pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        # checks left and right of robot so it knows where to turn AWAY from
        self.a90 = None
        self.frontrange = None
        self.a270 = None
        
    def proportional(self, k_p, p0, ideal_value):
        """ implementing proportional control equation: p_out = k_p*e(t) +p0 """
        error = output-ideal_value
        return k_p*error + p0
        # incomplete!!!

    
    def process_scan(self, m):
        # gets distance at index 90 degrees counterclockwise. if its not zero, update value.
        if m.ranges[90] != 0.0:
            self.a90 = m.ranges[90]
        # gets distance at -90 degrees
        if m.ranges[270] != 0.0:
            self.a270 = m.ranges[270]
        # seperate angles in quadrants. combining them together
        firstrange = list(reversed(m.ranges[:90]))
        secondrange = list(reversed(m.ranges[270:]))
        rawrange = firstrange + secondrange
        #filtering zeros out with max distance 
        self.frontrange = [10000 if (num == 0) else num for num in rawrange]
        self.smallestangle = self.find_angle(self.frontrange)
        print("smallestangle: ", self.smallestangle)


    def run(self):
        r = rospy.Rate(10)
        while not rospy.is_shutdown():
            r.sleep()

    def find_angle(self, anglerange):
        """ Finds the index of the smallest distance and returns the index/angle of that smallest distance"""
        minDistance = min(anglerange)
        return anglerange.index(minDistance)


if __name__ == '__main__':
    node = WallFollowingNode()
    node.run()