#!/usr/bin/env python

import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped
import math
import time
import sys
import pdb
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from random import randrange



x_pos_list = []
y_pos_list = []
x_pos = 0
y_pos = 0
flag_pos = 0 # to flag the first time the route_callback is called
x_goal_list = []
y_goal_list = []
x_goal = 0
y_goal = 0
flag_goal = 0 # to flag the first time the goal_callback is called


def route_callback(message, args):

    global flag_pos
    ID = args[0]
    global x_pos 
    global y_pos 
    x_pos = message.pose.pose.position.x
    y_pos = message.pose.pose.position.y 

    rospy.loginfo("ROUTE %s", flag_pos)

    flag_pos = 1

def goal_callback(message, args):

    global flag_goal
    ID = args[0]
    global x_goal 
    global y_goal
    x_goal = message.pose.position.x
    y_goal = message.pose.position.y

    rospy.loginfo("GOALLLLLL")

    flag_goal = 1

def animate(i, ID):

    global x_pos 
    global y_pos
    global flag_pos

    
    if ID == "0" :
        x_pos = x_pos + 30
    elif ID == "1":
        x_pos = x_pos - 30
    elif ID == "2":
        y_pos = y_pos + 30
    elif ID == "3":
        y_pos = y_pos - 30
    elif ID == "4":
        x_pos = x_pos + 31


    if flag_pos != 0:
        x_pos_list.append(x_pos)
        y_pos_list.append(y_pos) 
    if flag_goal != 0:
        x_goal_list.append(x_goal)
        y_goal_list.append(y_goal)

    plt.cla()
    #cl = ["b", "g", "r", "c", "m", "y", "k", "w"]

    #plt.plot(x_pos_list, y_pos_list, color = cl[randrange(0, 7)])
    plt.title("uav" + ID)
    plt.xlabel('x coordinate')
    plt.ylabel('y coordinate')
    plt.plot(x_pos_list, y_pos_list, label="uav" + ID + " route", color = 'darkorange')
    #plt.plot(x_goal_list, y_goal_list, label="goal_locations", marker= 'o')
    plt.scatter(x_goal_list, y_goal_list, label="goal_locations", color = 'firebrick')
    plt.legend(loc="upper right")


def trajectory_plot():

    rospy.init_node('trajectory_plot', anonymous=True)
    
    ID = sys.argv[1]
    uav = "uav" + ID
    uav_topic = uav+"/mavros/global_position/local"
    goal_topic = uav+"/mavros/setpoint_position/global"
    # import pdb;pdb.set_trace()
    rospy.Subscriber(uav_topic, Odometry, route_callback, (ID)) 
    rospy.Subscriber(goal_topic, PoseStamped, goal_callback, (ID))

    ani = FuncAnimation(plt.gcf(), animate, fargs=(ID,), interval=1000) # na dw mhpws mpainei sto callback

    plt.tight_layout()
    plt.show()
    
    # spin() -> keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    trajectory_plot()
    

