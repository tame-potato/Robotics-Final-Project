#!/usr/bin/env python
# license removed for brevity
# EECE_5560 Team Final Project: Quidditch Robots
# Ariel Pena-Martinez

import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import std_msgs
import time
import math
import numpy as np
from tf.transformations import *

class Quaffle:
    def __init__(self):
        # ------------ Subscriptions -------------
        self.sub = rospy.Subscriber('/robot_14/odom', Odometry, self.saveOdomSelf)      # quaffle
        self.sub = rospy.Subscriber('/robot_2/odom', Odometry, self.saveOdomRobot0)     # chaser Team A
        self.sub = rospy.Subscriber('/robot_3/odom', Odometry, self.saveOdomRobot1)     # chaser Team B
        # self.subGame = rospy.Subscriber("Game", Announcements, self.callback_Announcements) # game Announcer, Need to know what is it and msg type / info
        # ------------ end of Subscriptions --------

        # Publish move commands
        self.pub = rospy.Publisher('/robot_14/cmd_vel', Twist, queue_size=10)

        # ------- initialize objects odometry and twist and game flags -----
        self.my_odom = Odometry()
        self.robot_0_odom = Odometry()      # chaser Team A odometry
        self.robot_1_odom = Odometry()      # chaser Team B odometry
        self.twist = Twist()    
        self.acquired = None       # reset flag to False on new game
        self.distance0= 0   # distance between quaffle and chaser Team A
        self.distance1= 0   # distance between quaffle and chaser Team B
        self.counter = 0
         
        # --------- end of initialization ----------------------

        # ----------- Callbacks -------------------
    def saveOdomSelf(self, msg):        # robot_10 quaffle
        self.my_odom = msg
    def saveOdomRobot0(self, msg):      # robot_0 chaser Team A
        self.robot_0_odom = msg
    def saveOdomRobot1(self, msg):      # robot_1 chaser Team B
        self.robot_1_odom = msg
        # ----------- end of Callbacks ------------

    def get_twist_to_desired_bearing(self, desired_bearing):
        x = self.my_odom.pose.pose.orientation.x
        y = self.my_odom.pose.pose.orientation.y
        z = self.my_odom.pose.pose.orientation.z
        w = self.my_odom.pose.pose.orientation.w
        heading = euler_from_quaternion([x, y, z, w])[2]

        bearing = desired_bearing

        if heading - bearing < -np.pi:
            return -1
        elif heading - bearing < 0:
            return 1
        elif heading - bearing < np.pi:
            return -1
        else:
            return 1

    def calc_bearing(self, pos1, pos2):
        return np.arctan2((pos1.pose.pose.position.y - pos2.pose.pose.position.y), (pos1.pose.pose.position.x - pos2.pose.pose.position.x))

    def get_twist_to_waypoint(self, waypoint_odom):
        bearing = self.calc_bearing(waypoint_odom, self.my_odom)
        return self.get_twist_to_desired_bearing(bearing)

    def dist(self, x1, y1, x2, y2):
        return math.sqrt((x2 - x1) ** 2 + (y2 - y1) ** 2)

    def get_x_and_y_from_robots(self):
        x1 = self.my_odom.pose.pose.position.x
        y1 = self.my_odom.pose.pose.position.y
        x2 = self.robot_0_odom.pose.pose.position.x
        y2 = self.robot_0_odom.pose.pose.position.y
        #print('Red dist:', self.dist(x1, y1, x2, y2))
        if self.dist(x1, y1, x2, y2) > 4:
            self.acquired = None
        elif self.dist(x1, y1, x2, y2) < 3 or self.acquired == 'red':
            #print('Red acquired')
            self.acquired = 'red'
            x = self.robot_0_odom.pose.pose.orientation.x
            y = self.robot_0_odom.pose.pose.orientation.y
            z = self.robot_0_odom.pose.pose.orientation.z
            w = self.robot_0_odom.pose.pose.orientation.w
            heading = euler_from_quaternion([x, y, z, w])[2]
            return 1 * np.cos(heading), 1 * np.sin(heading)

        x2 = self.robot_1_odom.pose.pose.position.x
        y2 = self.robot_1_odom.pose.pose.position.y
        #print('Blue dist:', self.dist(x1, y1, x2, y2))
        if self.dist(x1, y1, x2, y2) > 4:
            self.acquired = None
        elif self.dist(x1, y1, x2, y2) < 3 or self.acquired == 'blue':
            #print('Red acquired')
            self.acquired = 'blue'
            x = self.robot_1_odom.pose.pose.orientation.x
            y = self.robot_1_odom.pose.pose.orientation.y
            z = self.robot_1_odom.pose.pose.orientation.z
            w = self.robot_1_odom.pose.pose.orientation.w
            heading = euler_from_quaternion([x, y, z, w])[2]
            return 1 * np.cos(heading), 1 * np.sin(heading)

        return 0, 0

    def process(self):
        # ------ Main logic ------------
        # check Announcements: New play?  (Needs to be defined)
        # Did I get caught? go check
        speed = 7
        while True:
            self.twist.linear.x, self.twist.linear.y = self.get_x_and_y_from_robots()
            self.pub.publish(self.twist)
            time.sleep(0.1)

if __name__ == '__main__':
    try:
        rospy.init_node('quaffle', anonymous=True)
        quaffle = Quaffle()
        while True:
            quaffle.process()
    except rospy.ROSInterruptException:
        pass
