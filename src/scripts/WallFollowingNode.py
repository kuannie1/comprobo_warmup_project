#!/usr/bin/env python
from __future__ import print_function, division
from std_msgs.msg import Header
import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist, Vector3
from geometry_msgs.msg import PointStamped 
from nav_msgs.msg import Odometry
import math 
import tf.transformations as tft
from tf.transformations import euler_from_quaternion, quaternion_from_euler
import numpy as np

def get_min_distance(values):
    values_np = np.asarray(values)
    values_np[values_np == 0] = np.nan # or use np.nan
    return np.amin(values_np)

class WallFollowingNode(object):
    """ This node moves forward at a fixed pace and stops when it detects an object within a certain distance. """
    def __init__(self, threshold):
        self.objdistance = 2
        self.distances = [0] * 361
        rospy.init_node('wallfollow_node')
        self.sub_laser = rospy.Subscriber('/scan', LaserScan, self.process_scan)
        self.sub_odom = rospy.Subscriber('/odom', Odometry, self.get_rotation)
        self.pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.threshold = threshold # distance
        
        self.current_rad = 0.0

        # variables from RotateNode.py file:
        # self.target_deg = 90
        self.kp = 0.3
        
        self.command = Twist()
        

    def process_scan(self, m):
        self.objdistance = get_min_distance(m.ranges)
        self.angle = m.ranges.index(self.objdistance)
        self.distances = m.ranges


    def run(self):
        r = rospy.Rate(10)
        while not rospy.is_shutdown():
            rospy.loginfo_throttle(0.5, "threshold={} objdistance={}".format(self.threshold, self.objdistance))
            if (self.objdistance > self.threshold):
                m = Twist(linear=Vector3(x=.1, y=0, z=0), angular=Vector3(x=0, y= 0, z=0))
                self.pub.publish(m)
            else:
                self.orient_wall()
                # self.rotate()
            r.sleep()


    def get_rotation(self, m):
        """ Obtaining variables about current state """
        orientation_q = m.pose.pose.orientation
        orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
        (roll, pitch, yaw) = euler_from_quaternion(orientation_list)
        self.current_rad = yaw
        current_deg = math.degrees(self.current_rad)
        rospy.loginfo_throttle(0.5, "current_deg: {}".format(current_deg))
    
    # def rotate(self, target. acceptableErrorDeg = 2.0):
    #     """ Calculates the angle needed to twist continuously """
    #     r = rospy.Rate(10)
        
    #     while not rospy.is_shutdown():
    #         target_rad = math.radians(target)
    #         angleerror = ((target_rad - self.current_rad) + math.pi) % (2*math.pi) - math.pi  # Ensure angle is between -pi and pi
            
    #         if abs(angleerror) < math.radians(acceptableErrorDeg):
    #             return
    #         else:
    #             self.command.angular.z = (self.kp * angleerror) 
    #             self.pub.publish(self.command)
    #             # printing the message at 2Hz (once per 0.5 seconds)
    #             rospy.loginfo_throttle(0.5, "target={} current={} error={}".format(target, math.degrees(self.current_rad), angleerror))
    #         r.sleep()

    def orient_wall(self, ang_range=10, acceptableErrorDist = 0.1):
        """ Uses the distance between angles 90+30 and 90-30 to control the robot's orientation """
        dist1 = self.distances[90+ang_range]
        dist2 = self.distances[90-ang_range]
        while not rospy.is_shutdown():
            disterror = math.fabs(dist1-dist2)
            if disterror > acceptableErrorDist:
                self.command.angular.z = 0.1
                self.pub.publish(self.command)
                # printing the message at 2Hz (once per 0.5 seconds)
                rospy.loginfo_throttle(0.5, "dist1={} dist2={} difference={}".format(dist1, dist2, disterror))
            else:
                return

if __name__ == '__main__':
    node = WallFollowingNode(1)
    node.run()
