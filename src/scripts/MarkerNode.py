#!/usr/bin/env python
from __future__ import print_function, division
from std_msgs.msg import Header
import rospy
import rviz
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Vector3, Point, PointStamped

class MarkerNode(object):
    """ This node draws a sphere at x = 1 m, y = 2 m in the odom coordinate system in rviz --Incomplete!! """
    def __init__(self):
        rospy.init_node('sphere_marker')
        self.pub = rospy.Publisher('/visualization_marker', Marker, queue_size=10) 
    
    def run(self):
        r = rospy.Rate(10)
        while not rospy.is_shutdown():

            my_header = Header(stamp=rospy.Time.now(), frame_id="odom")
            
            sphere = Marker(header=my_header)
            sphere.type = Marker.SPHERE
            sphere.scale.x = 1
            sphere.scale.y = 1
            sphere.scale.z = 1
            sphere.pose.position.x = 1
            sphere.pose.position.y = 2
            sphere.pose.position.z = 0
            sphere.color.r = 0.0
            sphere.color.g = 1.0
            sphere.color.b = 0.0
            sphere.color.a = 1.0
            # sphere.header=my_header

            self.pub.publish(sphere)
            r.sleep()

if __name__ == '__main__':
    node = MarkerNode()
    node.run()