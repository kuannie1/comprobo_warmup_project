#!/usr/bin/env python

from __future__ import print_function
import rospy
from geometry_msgs.msg import Twist
import tty
import select
import sys
import termios

class TeleopNode(object):
    def __init__(self):
        rospy.init_node('teleop_node')
        self.pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        self.settings = termios.tcgetattr(sys.stdin)
        self.speed = rospy.get_param("~speed", 0.5)
        self.turn = rospy.get_param("~turn", 1.0)
        self.x = 0
        self.y = 0
        self.z = 0
        self.th = 0
        self.status = 0
        self.key = None
        self.moveBindings = {
            'i':(1,0,0,0),
            'o':(1,0,0,-1),
            'j':(0,0,0,1),
            'l':(0,0,0,-1),
            'u':(1,0,0,1),
            ',':(-1,0,0,0),
            '.':(-1,0,0,1),
            'm':(-1,0,0,-1),
            'O':(1,-1,0,0),
            'I':(1,0,0,0),
            'J':(0,1,0,0),
            'L':(0,-1,0,0),
            'U':(1,1,0,0),
            '<':(-1,0,0,0),
            '>':(-1,-1,0,0),
            'M':(-1,1,0,0),
            't':(0,0,1,0),
            'b':(0,0,-1,0),
	    }
    
    def getKey(self, key=None):
        """
            Takes in keyboard character or input variable and saves it under self.key
        """
        tty.setraw(sys.stdin.fileno())
        select.select([sys.stdin], [], [], 0)
        if key==None:
            self.key = sys.stdin.read(1)
        else:
            self.key = key
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)

    def run(self, key=None):
        """
            updates the self.key variable and decides how to move in the linear and angular directions
        """
        while not rospy.is_shutdown():
            self.getKey(key) 
            if (self.key != '\x03'):
                if self.key in self.moveBindings.keys():
                    self.x, self.y, self.z, self.th = self.moveBindings[self.key]
                else:
                    self.x, self.y, self.z, self.th = (0,0,0,0) # if the charcter isn't there, then stop it
                twist = Twist()
                twist.linear.x = self.x*self.speed
                twist.linear.y = self.y*self.speed
                twist.linear.z = self.z*self.speed
                twist.angular.x = 0
                twist.angular.y = 0
                twist.angular.z = self.th*self.turn
                self.pub.publish(twist)

            else: #exit in case of ctrl+C
                self.pub.publish(Twist())
                rospy.signal_shutdown('Quit')


if __name__ == '__main__':
    node = TeleopNode()
    node.run()