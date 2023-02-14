#!/usr/bin/env python
# license removed for brevity
import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import std_msgs
import time
import math
import numpy as np
from tf.transformations import *

class OddKeeper:
    def __init__(self):
        self.sub = rospy.Subscriber('/robot_7/odom', Odometry, self.saveOdomSelf)         # subscribe to own (Odd Keeper's) odometry
        self.sub = rospy.Subscriber('/robot_0/odom', Odometry, self.saveOdomRobot0)       # subscribe to Even Chaser's odometry
        self.sub = rospy.Subscriber('/robot_11/odom', Odometry, self.saveOdomRobot11)     # subscribe to Odd Goal
        self.pub = rospy.Publisher('/robot_7/cmd_vel', Twist, queue_size=10)              # publish cmd_vel to make Odd Keeper move

        self.robot_7_odom = Odometry()                                                    # initialize messages for odometries and twist motion
        self.robot_0_odom = Odometry()
        self.robot_11_odom = Odometry()
        self.twist = Twist()

        self.BlockedChaser = False

        self.twist.linear.x = 1                                                            # initial movement for Stage
        self.pub.publish(self.twist)
        time.sleep(1)
        rospy.logwarn("initial movement done")        


    def saveOdomSelf(self, msg):                                                          # save own (Odd Keeper's) odometry
        self.robot_7_odom = msg

    def saveOdomRobot0(self, msg):                                                        # save Even Chaser's odometry
        self.robot_0_odom = msg

    def saveOdomRobot11(self, msg):                                                       # save Odd Goal's odometry
        self.robot_11_odom = msg

    def get_twist_to_waypoint(self, waypoint_odom):
        x = self.robot_7_odom.pose.pose.orientation.x
        y = self.robot_7_odom.pose.pose.orientation.y
        z = self.robot_7_odom.pose.pose.orientation.z
        w = self.robot_7_odom.pose.pose.orientation.w
        heading = euler_from_quaternion([x, y, z, w])[2]
        bearing = np.arctan2((waypoint_odom.pose.pose.position.y - self.robot_7_odom.pose.pose.position.y), (waypoint_odom.pose.pose.position.x - self.robot_7_odom.pose.pose.position.x))
        #print('Heading:', heading)
        #print('Bearing:', bearing)
        if heading - bearing < -np.pi:
            return -1
        elif heading - bearing < 0:
            return 1
        elif heading - bearing < np.pi:
            return -1
        else:
            return 1

    def calculateDistance(self, xA, yA, xB, yB):                                          
        dist = math.sqrt((xB - xA) ** 2 + (yB - yA) ** 2)
        return dist

    def updatePosition(self):
        self.x0 = self.robot_0_odom.pose.pose.position.x                                               
        self.y0 = self.robot_0_odom.pose.pose.position.y                                               
        self.x11 = self.robot_11_odom.pose.pose.position.x
        self.y11 = self.robot_11_odom.pose.pose.position.y
        self.x7 = self.robot_7_odom.pose.pose.position.x
        self.y7 = self.robot_7_odom.pose.pose.position.y
        self.chaser_position = [self.x0, self.y0]
        self.goal_position = [self.x11, self.y11]
        self.keeper_position = [self.x7, self.y7] 
        
    def process(self):
        rospy.logwarn("start")
        self.twist.linear.x = 1
        self.pub.publish(self.twist)
        time.sleep(1)
        #rospy.logwarn("initial movement done")

        while self.BlockedChaser is False:
            #rospy.logwarn("Entered first while loop")
            oddkeeper.updatePosition()
            dist_bw_chaser_goal = self.calculateDistance(self.chaser_position[0], self.chaser_position[1], self.goal_position[0], self.goal_position[1])
            #print("The distance between the Chaser and the Goal is %f", dist_bw_chaser_goal)
            #rospy.logwarn(dist_bw_chaser_goal)
            if (dist_bw_chaser_goal > 10):
                continue
            elif (dist_bw_chaser_goal <= 10):
                oddkeeper.updatePosition()
                dist_bw_keeper_chaser = self.calculateDistance(self.keeper_position[0], self.keeper_position[1], self.chaser_position[0], self.chaser_position[1])
                while (dist_bw_keeper_chaser > 0):                                       # while Odd Keeper hasn't reached Even Chaser
                    self.twist.linear.x = 2                                              # move towards Even Chaser
                    self.twist.angular.z = self.get_twist_to_waypoint(self.robot_0_odom)
                    self.pub.publish(self.twist) 
                    time.sleep(0.2) 
                    oddkeeper.updatePosition()
                    dist_bw_keeper_chaser = self.calculateDistance(self.keeper_position[0], self.keeper_position[1], self.chaser_position[0], self.chaser_position[1])
                    #print("The distance between the Keeper and Chaser is %f", dist_bw_keeper_chaser)
                    if (dist_bw_keeper_chaser == 0):                                     # when Odd Keeper reaches Even Chaser
                        #print("The Keeper has blocked the Chaser!")
                        break
                self.twist.linear.x = 0                                                  # Odd Keeper stops
                self.pub.publish(self.twist)
                self.BlockedChaser = True
                break
             


# here gamekeeper needs to reset to original position?


if __name__ == '__main__':
    rospy.init_node('oddkeeper_node', anonymous=True)
    oddkeeper = OddKeeper()
    oddkeeper.process()
    while not rospy.is_shutdown():
        rospy.spin()
    
