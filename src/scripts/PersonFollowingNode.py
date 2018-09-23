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

from sklearn.cluster import MeanShift, estimate_bandwidth

class PersonFollowingNode(object):
    def __init__(self):
        rospy.init_node('ObstacleAvoidingNode')
        self.distances = None
        self.pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.sub_laser = rospy.Subscriber('/scan', LaserScan, self.process_scan)
        self.middle_angle = self.get_angle()
        self.current_rad = 0.0
    

    def process_scan(self, m):
        """ Takes in the messages & assigns all the class variables """
        self.distances = np.array(m.ranges)
        

    def get_angle(self):
        """ Uses clustering to find 2 equal clusters & middle angle (update self.middle_angle)"""
        r = rospy.Rate(10)
        while not rospy.is_shutdown():
            if (self.distances != None):
                
                ms = MeanShift(bin_seeding=True)
                ms.fit(self.distances.reshape(-1,1))
                labels = ms.labels_
                cluster_labels = np.unique(labels)
                cluster_index_mapping = {}
                print(labels)
                # get the index (angle) associated with the cluster
                for cluster_label in cluster_labels:
                    indices = [i for i, x in enumerate(labels) if x == cluster_label]
                    if len(indices) < 20:
                        cluster_index_mapping[cluster_label] = np.average(indices)
                    # np.average(indices)
                print(cluster_index_mapping)
                return np.average(cluster_index_mapping.values())
            r.sleep()
        return 0


    def rotate(self, angle):
        # make sure you 'break' out of it when the acceptable angle is reached
        """ Calculates the angle needed to twist continuously """
        currentTime = rospy.Time.now()
        stopTime = currentTime + rospy.Duration(math.radians(angle))
        while (currentTime<stopTime):
            m = Twist(angular=Vector3(x=0, y= 0, z=1))
            currentTime = rospy.Time.now()
            self.pub.publish(m)
        self.pub.publish(Twist())

    def travel_to_point(self):
        """ Orients itself to middle_angle and moves forward in that direction """
        if (self.middle_angle != None):
            self.rotate(self.middle_angle)
            self.pub.publish(Twist(linear=Vector3(x=.1, y=0, z=0)))


    def run(self):
        """ Uses Mean Shift clustering algorithm to find clusters and move to that centroid"""
        while not rospy.is_shutdown():
            self.travel_to_point()

if __name__ == '__main__':
    node = PersonFollowingNode()
    node.run()