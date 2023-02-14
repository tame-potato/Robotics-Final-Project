# go to waypoint function calculates the twist required to reach a given position

import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import std_msgs
import time
import math
import numpy as np
from tf.transformations import *


# Must add a class specific variable (self.stall_counter) initialized to 0, and a  variable to hold the last odometry (self.last_odom) to use this function. The new values for this variables are returned so they can be updated everytime it is called.
# The new twist must be puclished after it is returned.
def check_collision(last_odom, my_odom, twist, stall_counter, pub):
        # check if not moving for some cycles, if not, then probably a collison, move away
        # print("stall counter:", self.stall_counter)
        last = last_odom.pose.pose.position
        new = my_odom.pose.pose.position

        if (last == new):
            stall_counter += 1         # not moving, add counter
            #print("I'm not moving:", self.stall_counter)
            if stall_counter >=3:      #not moving, collision, try to move away
                #print("Trying to get out of here ...:")
                change_direction = twist.linear.x * -1
                for i in range(15):
                    if i <=5:
                        #print("reversing ...")
                        twist.linear.x = change_direction   # Go inverse direction
                    else:
                        #print("forward ...")
                        #self.my_odom.pose.pose.orientation.w = 0
                        twist.angular.z = 0
                        twist.linear.x = 2
                    pub.publish(twist)
                    time.sleep(.2)

                stall_counter = 0
        else:                               # we are moving, we are good, do nothin
            stall_counter = 0
   
        last_odom.pose.pose.position = new # save last position for comparing next run
        
        return last_odom, stall_counter


def get_twist_to_waypoint(my_odom, waypoint_odom):

        x = my_odom.pose.pose.orientation.x
        y = my_odom.pose.pose.orientation.y
        z = my_odom.pose.pose.orientation.z
        w = my_odom.pose.pose.orientation.w

        heading = euler_from_quaternion([x, y, z, w])[2]

        bearing = np.arctan2((waypoint_odom.pose.pose.position.y - my_odom.pose.pose.position.y), (waypoint_odom.pose.pose.position.x - my_odom.pose.pose.position.x))

#        print('Heading:', heading)
#        print('Bearing:', bearing)

        if heading - bearing < -np.pi:
            return -1
        elif heading - bearing < 0:
            return 1
        elif heading - bearing < np.pi:
            return -1
        else:
            return 1

def check_distance(x1, y1, x2, y2):
        # calculate distance between pointA and pointB
        pointA = [x1, y1]          # define pointA position
        pointB = [x2, y2]          # define pointB position
        distance = math.sqrt((pointA[0]- pointB[0]) **2 + (pointA[1]- pointB[1]) **2)   
        return distance

def goto_location(targetrobot, my_odom, twist, pub):
        # move to specified robot
        twist.linear.x = 2
        pub.publish(twist)
        time.sleep(.1)

        for i in range(3):
            twist.linear.x = 2
            twist.angular.z = get_twist_to_waypoint(my_odom,targetrobot)     # move to target robot
            pub.publish(twist)
            time.sleep(0.1)

def got_the_ball(ball_odom, agent_odom):
        # check if the agent has the ball (in close range), what is close range?
        x1 = agent_odom.pose.pose.position.x  # agent x position                                             
        y1 = agent_odom.pose.pose.position.y  # agent y position                                               
        x2 = ball_odom.pose.pose.position.x  # ball x position
        y2 = ball_odom.pose.pose.position.y  # ball y position

        distance = check_distance(x1, y1, x2, y2)
        #print("distance to ball:", distance)

        if ((x1 + y1 != 0) and (x2 + y2 != 0)):      # on first launch, the robots position shows [0, 0], if not real, skeep
            if (distance <= 3):   # is this close enough without colliding?
                gotBall_flag = True       # ball is in my possesion
            else:
                gotBall_flag = False # if player is not close to ball then it is not in possesion

            return gotBall_flag

def check_objective(my_odom, objective_odom, last_odom, twist, stall_counter, pub):
        # check distance to my objective
        x1 = my_odom.pose.pose.position.x                                                      
        y1 = my_odom.pose.pose.position.y

        x2 = objective_odom.pose.pose.position.x   
        y2 = objective_odom.pose.pose.position.y

        distance = check_distance(x1, y1, x2, y2)

        #print("distance to goal:", distance)            # check distance between agent and its objective
        if distance <= 2:
            #print("Gooooooooaaaaaaaalllllllll")
            last_odom = my_odom
            stall_counter = 0
            return True, last_odom, stall_counter
        else:
            # not yet !!!
            last_odom, stall_counter = check_collision(last_odom, my_odom, twist, stall_counter, pub)
            return False, last_odom, stall_counter
