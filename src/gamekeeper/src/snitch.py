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

class Snitch:
    def __init__(self):
        # ------------ Subscriptions -------------
        self.sub = rospy.Subscriber('/robot_10/odom', Odometry, self.saveOdomSelf)      # snitch
        self.sub = rospy.Subscriber('/robot_4/odom', Odometry, self.saveOdomRobot4)     # seeker Team A
        self.sub = rospy.Subscriber('/robot_5/odom', Odometry, self.saveOdomRobot5)     # seeker Team B
        # self.subGame = rospy.Subscriber("Game", Announcements, self.callback_Announcements) # game Announcer, Need to know what is it and msg type / info
        # ------------ end of Subscriptions --------

        # Publish move commands
        self.pub = rospy.Publisher('/robot_10/cmd_vel', Twist, queue_size=10)

        # ------- initialize objects odometry and twist and game flags -----
        self.my_odom = Odometry()
        self.robot_4_odom = Odometry()      # seeker Team A odometry
        self.robot_5_odom = Odometry()      # seeker Team B odometry
        self.random_odem = Odometry()       # I'm planning to add 10 to my_odom in order to set a go_to random location
        self.twist = Twist()    
        self.gotthesnitch_flag= False       # reset flag to False on new game
        self.distance4= 0   # distance between snitch and seeker Team A
        self.distance5= 0   # distance between snitch and seeker Team B
        self.counter = 0
        x_random = self.my_odom.pose.pose.position.x + np.random.randint(10)     # add 10 from where snitch is and go there (This is just gessing)
        y_random = self.my_odom.pose.pose.position.y + np.random.randint(10)     # add 10 from where snitch is and go there (This is just gessing)
        self.random_odem.pose.pose.position.x = x_random
        self.random_odem.pose.pose.position.y = y_random
         
        # --------- end of initialization ----------------------

        # ----------- Callbacks -------------------
    def saveOdomSelf(self, msg):        # robot_10 snitch
        self.my_odom = msg
    def saveOdomRobot4(self, msg):      # robot_4 seeker Team A
        self.robot_4_odom = msg
    def saveOdomRobot5(self, msg):      # robot_5 seeker Team B
        self.robot_5_odom = msg
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

    def get_twist_away_from_bots(self):
        r4_bearing = self.calc_bearing(self.robot_4_odom, self.my_odom)
        r5_bearing = self.calc_bearing(self.robot_5_odom, self.my_odom)
        new_theta = (r4_bearing + r5_bearing) / 2
        if new_theta < 0:
            new_theta -= np.pi
        else:
            new_theta += np.pi
        return self.get_twist_to_desired_bearing(new_theta)

        pointA = [x1, y1]          # define pointA position
        pointB = [x2, y2]          # define pointB position
        distance = math.sqrt((pointA[0]- pointB[0]) **2 + (pointA[1]- pointB[1]) **2)   
        return distance

    def process(self):
        # ------ Main logic ------------
        # check Announcements: New play?  (Needs to be defined)
        # Did I get caught? go check
        speed = 7
        while not self.gotthesnitch_flag:
            self.twist.linear.x = speed
            speed -= 0.1
            self.twist.angular.z = self.get_twist_away_from_bots()     # move to target location
            self.pub.publish(self.twist)
            time.sleep(0.1)

if __name__ == '__main__':
    try:
        rospy.init_node('snitch', anonymous=True)
        snitch = Snitch()
        while True:
            snitch.process()
    except rospy.ROSInterruptException:
        pass
