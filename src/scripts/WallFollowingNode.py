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

def get_first_nonzero_val(values):
    for i in range(len(values)):
        num = values[i]
        if num != 0:
            return num
    return 0

class WallFollowingNode(object):
    """ This node moves forward at a fixed pace and stops when it detects an object within a certain distance. """
    def __init__(self, threshold):
        self.objdistance = 2
        rospy.init_node('wallfollow_node')
        self.sub = rospy.Subscriber('/scan', LaserScan, self.process_scan)
        self.pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.threshold = threshold # distance
        
        self.current_rad = 0.0

        # variables from RotateNode.py file:
        self.target_deg = 90
        self.kp = 0.3
        
        self.command = Twist()

    def process_scan(self, m):
        self.objdistance = get_first_nonzero_val(m.ranges)
        self.angle = m.ranges.index(self.objdistance)
        print("angle: ", self.angle)


    def run(self):
        r = rospy.Rate(10)
        while not rospy.is_shutdown():
            print("objdistance: ", self.objdistance)
            if (self.objdistance > self.threshold):
                m = Twist(linear=Vector3(x=.1, y=0, z=0), angular=Vector3(x=0, y= 0, z=0))
                self.pub.publish(m)
            else:
                self.rotate()
            r.sleep()


    def get_rotation(self, m):
        """ Obtaining variables about current state """
        orientation_q = m.pose.pose.orientation
        orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
        (roll, pitch, yaw) = euler_from_quaternion(orientation_list)
        self.current_rad = yaw
        current_deg = math.degrees(self.current_rad)
        rospy.loginfo_throttle(0.5, "current_deg: {}".format(current_deg))
    
    def rotate(self, acceptableErrorDeg = 2.0):
        """ Calculates the angle needed to twist continuously """
        r = rospy.Rate(10)
        self.sub = rospy.Subscriber('/odom', Odometry, self.get_rotation)
        while not rospy.is_shutdown():
            target_rad = math.radians(self.target_deg)
            angleerror = ((target_rad - self.current_rad) + math.pi) % (2*math.pi) - math.pi  # Ensure angle is between -pi and pi
            
            if abs(angleerror) < math.radians(acceptableErrorDeg):
                return
            else:
                self.command.angular.z = (self.kp * angleerror) 
                self.pub.publish(self.command)
                # printing the message at 2Hz (once per 0.5 seconds)
                rospy.loginfo_throttle(0.5, "target={} current={} error={}".format(self.target_deg, math.degrees(self.current_rad), angleerror))
            r.sleep()


if __name__ == '__main__':
    node = WallFollowingNode(1)
    node.run()
