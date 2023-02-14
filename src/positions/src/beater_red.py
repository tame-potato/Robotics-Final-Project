#!/usr/bin/env python
# license removed for brevity
# EECE_5560 Team Final Project: Quidditch Robots
# Pablo Ruiz

import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from shared_utils.shared_functions import check_collision, get_twist_to_waypoint, check_distance, goto_location, got_the_ball, check_objective
import std_msgs
import time
import math
import numpy as np
from tf.transformations import *


class beater:
    def __init__(self):
        
        # Friendly team is referred to as Team A and enemy as Team B
       
        # ------------ Subscriptions -------------
        self.sub = rospy.Subscriber('/robot_2/odom', Odometry, self.saveOdomSelf)               # beater Team A
        self.sub = rospy.Subscriber('/robot_1/odom', Odometry, self.saveOdomChaserEnemy)        # chaser Team B
        self.sub = rospy.Subscriber('/robot_9/odom', Odometry, self.saveOdomBludger)            # bludger


#       self.subGame = rospy.Subscriber("Game", Announcements, self.callback_Announcements)     # game Announcer, Need to know what is it and msg type / info
        # ------------ end of Subscriptions --------

        # Publish move commands
        self.pub = rospy.Publisher('/robot_2/cmd_vel', Twist, queue_size=10)


        # ------- initialize objects odometry and twist and game flags ------
        self.my_odom = Odometry()           # beater Team A odometry
        self.enemyChaser_odom = Odometry()  # chaser Team B odometry
        self.bludger_odom = Odometry()      # bludger odometry
        self.last_odom = Odometry()         # previous my_odom reading
        self.stall_counter = 0              
        self.twist = Twist()                # output move self    
        self.gotBludger_flag = False     # reset flag to False on new game
        self.objective_flag = False
        # --------- end of initialization ----------------------
        

        # ----------- Callbacks -------------------
    def saveOdomSelf(self, msg):        
        self.my_odom = msg
    def saveOdomChaserEnemy(self, msg):    
        self.enemyChaser_odom = msg
    def saveOdomBludger(self, msg):
        self.bludger_odom = msg
        # ----------- end of Callbacks ------------

    def process(self):
        # ------ Main logic ------------
        # check Announcements: New play?  (Needs to be defined)
        # do I or the enemy have the bludger? go check
        self.gotBludger_flag = got_the_ball(self.bludger_odom, self.my_odom)

        if self.gotBludger_flag:         # bludger needs to follow
            #print("--- got the Bludger!!!, now time to take someone out ---")
            goto_location(self.enemyChaser_odom, self.my_odom, self.twist, self.pub)     # move towards enemy chaser
            self.objective_flag, self.last_odom, self.stall_counter = check_objective(self.my_odom, self.enemyChaser_odom, self.last_odom, self.twist, self.stall_counter, self.pub)
        else:                 
            #print("I dont have the bludger, let's go get it. Go to Bludger")                         
            # avoid Beater & Keeper, maybe implement later, lets get the bludger for now.
            goto_location(self.bludger_odom, self.my_odom, self.twist, self.pub)      # move to bludger
            self.last_odom, self.stall_counter = check_collision(self.last_odom, self.my_odom, self.twist, self.stall_counter, self.pub)

if __name__ == '__main__':
    try:

        rospy.init_node('beater', anonymous=True)

        beater = beater()

        while True:
            beater.process()

    except Exception as e:
        rospy.logwarn("Error in try block.")
