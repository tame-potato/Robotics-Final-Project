#!/usr/bin/env python
# license removed for brevity
# EECE_5560 Team Final Project: Quidditch Robots
# Edmundo Peralta

import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from shared_utils.shared_functions import check_collision, get_twist_to_waypoint, check_distance, goto_location, got_the_ball, check_objective
import std_msgs
import time
import math
import numpy as np
from tf.transformations import *


class chaser:
    def __init__(self):
        
        # Friendly team is referred to as Team A and enemy as Team B
       
        # ------------ Subscriptions -------------
        self.sub = rospy.Subscriber('/robot_0/odom', Odometry, self.saveOdomSelf)               # chaser Team A
        self.sub = rospy.Subscriber('/robot_11/odom', Odometry, self.saveOdomGoal)              # goal Team B
        self.sub = rospy.Subscriber('/robot_8/odom', Odometry, self.saveOdomQuaffle)            # quaffle


#       self.subGame = rospy.Subscriber("Game", Announcements, self.callback_Announcements)     # game Announcer, Need to know what is it and msg type / info
        # ------------ end of Subscriptions --------

        # Publish move commands
        self.pub = rospy.Publisher('/robot_0/cmd_vel', Twist, queue_size=10)


        # ------- initialize objects odometry and twist and game flags ------
        self.my_odom = Odometry()           # chaser friendly odometry
        self.enemyGoal_odom = Odometry()    # goal enemy odometry
        self.quaffle_odom = Odometry()      # quaffle odometry
        self.last_odom = Odometry()         # previous my_odom reading
        self.stall_counter = 0              
        self.twist = Twist()                # output move self    
        self.gotQuaffle_flag = False        # reset flag to False on new game
        self.objective_flag = False
        # --------- end of initialization ----------------------
        

        # ----------- Callbacks -------------------
    def saveOdomSelf(self, msg):        
        self.my_odom = msg
    def saveOdomGoal(self, msg):    
        self.enemyGoal_odom = msg
    def saveOdomQuaffle(self, msg):
        self.quaffle_odom = msg
        # ----------- end of Callbacks ------------

    def process(self):
        # ------ Main logic ------------
        # check Announcements: New play?  (Needs to be defined)
        # do I have the ball? go check
        self.gotQuaffle_flag = got_the_ball(self.quaffle_odom, self.my_odom)

        if self.gotQuaffle_flag:         # quaffle needs to follow
            #print("--- got the Quaffle!!! ---")
            goto_location(self.enemyGoal_odom, self.my_odom, self.twist, self.pub)     # move towards objective
            self.objective_flag, self.last_odom, self.stall_counter = check_objective(self.my_odom, self.enemyGoal_odom, self.last_odom, self.twist, self.stall_counter, self.pub)
        else:                 
            #print("I dont have the Quaffle, let's go get it. Go to Quaffle")                         
            # avoid Beater & Keeper, maybe implement later, lets get the bludger for now.
            goto_location(self.quaffle_odom, self.my_odom, self.twist, self.pub)      # move to bludger
            self.last_odom, self.stall_counter = check_collision(self.last_odom, self.my_odom, self.twist, self.stall_counter, self.pub)

if __name__ == '__main__':
    try:

        rospy.init_node('chaser', anonymous=True)

        chaser = chaser()

        while True:
            chaser.process()

    except Exception as e:
        rospy.logwarn("Error in try block.")
