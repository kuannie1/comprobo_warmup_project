#!/usr/bin/env python
from __future__ import print_function, division
from std_msgs.msg import Header
import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist, Vector3
from geometry_msgs.msg import PointStamped 
# access this page for help with timer: http://wiki.ros.org/roscpp_tutorials/Tutorials/Timers 
# general idea: make sure to twist until the angle is achieved

class RotateNode(object, k):
    """ This node rotates a factor of k*pi """
    def __init__(self, theta):
		self.objdistance = 0
		rospy.init_node('rotate_node')
		rospy.Subscriber('/scan', LaserScan, self.process_scan)
		self.pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

    def process_scan(self, m):
        pass

    def run(self):
		r = rospy.Rate(10)
		while not rospy.is_shutdown():
			pass
			r.sleep()

if __name__ == '__main__':
    node = RotateNode(1) # 180 deg turn
    node.run()
