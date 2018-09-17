#!/usr/bin/env python
from __future__ import print_function, division
from std_msgs.msg import Header
import rospy
import rviz
import tf.transformations as tft
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from geometry_msgs.msg import Twist, Vector3, TwistWithCovariance
import math
from nav_msgs.msg import Odometry

# got code from here: http://www.theconstructsim.com/ros-qa-135-how-to-rotate-a-robot-to-a-desired-heading-using-feedback-from-odometry/ 
class RotateNode(object):
    """ This node takes in an angle (in positive or negative degrees) and rotates the neato by that amount """
    def __init__(self, deg):
        rospy.init_node('rotateNode')
        self.offset = None
        self.target_deg = deg
        self.target_rad = math.radians(self.target_deg)
        self.current_rad = 0.0
        self.kp = 0.3
        self.sub = rospy.Subscriber('/odom', Odometry, self.get_rotation)
        self.pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)
        self.command = Twist()


    def get_rotation(self, m):
        orientation_q = m.pose.pose.orientation
        orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
        (roll, pitch, yaw) = euler_from_quaternion(orientation_list)
        self.current_rad = yaw
        current_deg = math.degrees(self.current_rad)
        rospy.loginfo_throttle(0.5, "current_deg: {}".format(current_deg))
    
    def run(self, acceptableErrorDeg = 2.0):
        r = rospy.Rate(10)
        while not rospy.is_shutdown():
            angleerror = ((self.target_rad - self.current_rad) + math.pi) % (2*math.pi) - math.pi  # Ensure angle is between -pi and pi
            
            # error = math.fabs(self.target_deg - self.current_deg) # making sure degrees are within error
            # complimentError = math.fabs(360.0 - self.target_deg + self.current_deg)
            # print("error {}, complimentError {}".format(error, complimentError))
            if abs(angleerror) < math.radians(acceptableErrorDeg):
                break
            else:
                self.command.angular.z = (self.kp * angleerror) #literally mod-ing 2pi... what is this
                # print("z: ", self.command.angular.z)
                self.pub.publish(self.command)
                rospy.loginfo_throttle(0.5, "target={} current={} error={}".format(math.degrees(self.target_rad), math.degrees(self.current_rad), angleerror))
            r.sleep()

if __name__ == '__main__':
    node = RotateNode(180.0)
    node.run()