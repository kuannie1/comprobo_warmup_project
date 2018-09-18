#!/usr/bin/env python
from __future__ import print_function, division
from std_msgs.msg import Header
import rospy
import rviz
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist, Vector3
import math
import tf.transformations as tft
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from nav_msgs.msg import Odometry

class WallFollowingNode(object):
    """ This node moves forward until it detects an obstacle, rotates itself to be parallel to the obstacle, and moves forward again """
    def __init__(self):
        rospy.init_node('wall_follower_node')
        self.sub = rospy.Subscriber('/scan', LaserScan, self.process_scan)
        self.pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        # self.command = Twist()
        self.current_rad = 0.0 # yaw in radian form
        self.kp = 0.3
        self.obj_distance = None
        self.threshold = 1.0

    def process_scan(self, m):
        """ Method that saves the distances at 90, 270, and angle with smallest distance """
        #filtering zeros out with max distance 
        self.frontrange = [10000 if (num == 0) else num for num in m.ranges]
        self.obj_distance, self.smallestangle = self.find_angle(self.frontrange)


    def run(self):
		r = rospy.Rate(10)
		while not rospy.is_shutdown():
			if (self.obj_distance > self.threshold):
				m = Twist(linear=Vector3(x=0.2, y=0, z=0), angular=Vector3(x=0, y= 0, z=0))
				self.pub.publish(m)
			else:
				self.pub.publish(Twist())
			r.sleep()

    # def rotate(self, target_rad, acceptableErrorDeg = 2.0):
    #     """ Calculates the angle needed to twist continuously """
    #     r = rospy.Rate(10)
    #     while not rospy.is_shutdown():
    #         angleerror = ((target_rad - self.current_rad) + math.pi) % (2*math.pi) - math.pi  # Ensure angle is between -pi and pi
    #         if abs(angleerror) < math.radians(acceptableErrorDeg):
    #             break
    #         else:
    #             self.command.angular.z = (self.kp * angleerror) 
    #             self.pub.publish(self.command)
    #             # printing the message at 2Hz (once per 0.5 seconds)
    #             rospy.loginfo_throttle(0.5, "target={} current={} error={}".format(math.degrees(target_rad), math.degrees(self.current_rad), angleerror))
    #         r.sleep()

    def find_angle(self, anglerange):
        """ Finds the index of the smallest distance and returns the index/angle of that smallest distance"""
        minDistance = min(anglerange)
        return (minDistance, anglerange.index(minDistance))

    # def get_rotation(self, m):
    #     """ Obtaining variables about current state """
    #     orientation_q = m.pose.pose.orientation
    #     orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
    #     (roll, pitch, yaw) = euler_from_quaternion(orientation_list)
    #     self.current_rad = yaw
    #     current_deg = math.degrees(self.current_rad)
    #     rospy.loginfo_throttle(0.5, "current_deg: {}".format(current_deg))



if __name__ == '__main__':
    node = WallFollowingNode()
    node.run()