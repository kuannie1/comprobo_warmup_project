#!/usr/bin/env python
from __future__ import print_function, division
from std_msgs.msg import Header
import rospy
import rviz
from geometry_msgs.msg import Twist, Vector3

class SquareNode(object):
    """ This node moves a square path with a length of 1 meter. """
    def __init__(self):
        rospy.init_node('square_node')
        self.pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10) 

    def run(self):
        r = rospy.Rate(10)
        while not rospy.is_shutdown():
            for i in range(4):
                self.moveStraight()
                self.rotate()
                
            r.sleep()
    def moveStraight(self):
        currentTime = rospy.Time.now()
        stopTime = currentTime+rospy.Duration(3)
        while currentTime<stopTime:
            m = Twist(linear = Vector3(x=0.2, y=0, z=0))
            currentTime = rospy.Time.now()
            self.pub.publish(m)
        self.pub.publish(Twist())
    
    def rotate(self):
        currentTime = rospy.Time.now()
        stopTime = currentTime + rospy.Duration(1.57)
        while currentTime<stopTime:
            m = Twist(angular=Vector3(x=0, y= 0, z=1))
            currentTime = rospy.Time.now()
            self.pub.publish(m)
        self.pub.publish(Twist())

if __name__ == '__main__':
    node = SquareNode()
    node.run()
