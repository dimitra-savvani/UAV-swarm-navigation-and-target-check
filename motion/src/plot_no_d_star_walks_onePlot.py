#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseStamped
import sys
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from random import randrange



x_pos_list_0 = []
y_pos_list_0 = []
x_pos_0 = 0
y_pos_0 = 0
flag_pos_0 = 0 # to flag the first time the route_callback is called
x_goal_list_0 = []
y_goal_list_0 = []
x_goal_0 = 0
y_goal_0 = 0
flag_goal_0 = 0 # to flag the first time the goal_callback is called
x_goal_points_0 = []
y_goal_points_0 = []

x_pos_list_1 = []
y_pos_list_1 = []
x_pos_1 = 0
y_pos_1 = 0
flag_pos_1 = 0 # to flag the first time the route_callback is called
x_goal_list_1 = []
y_goal_list_1 = []
x_goal_1 = 0
y_goal_1 = 0
flag_goal_1 = 0 # to flag the first time the goal_callback is called
x_goal_points_1 = []
y_goal_points_1 = []

x_pos_list_2 = []
y_pos_list_2 = []
x_pos_2 = 0
y_pos_2 = 0
flag_pos_2 = 0 # to flag the first time the route_callback is called
x_goal_list_2 = []
y_goal_list_2 = []
x_goal_2 = 0
y_goal_2 = 0
flag_goal_2 = 0 # to flag the first time the goal_callback is called
x_goal_points_2 = []
y_goal_points_2 = []

x_pos_list_3 = []
y_pos_list_3 = []
x_pos_3 = 0
y_pos_3 = 0
flag_pos_3 = 0 # to flag the first time the route_callback is called
x_goal_list_3 = []
y_goal_list_3 = []
x_goal_3 = 0
y_goal_3 = 0
flag_goal_3 = 0 # to flag the first time the goal_callback is called
x_goal_points_3 = []
y_goal_points_3 = []



def route_callback_0(message):

    global flag_pos_0
    global x_pos_0 
    global y_pos_0 
    x_pos_0 = message.pose.position.x
    y_pos_0 = message.pose.position.y 

    flag_pos_0 = 1

def route_callback_1(message):

    global flag_pos_1
    global x_pos_1 
    global y_pos_1 
    x_pos_1 = message.pose.position.x
    y_pos_1 = message.pose.position.y 

    flag_pos_1 = 1

def route_callback_2(message):

    global flag_pos_2
    global x_pos_2 
    global y_pos_2 
    x_pos_2 = message.pose.position.x
    y_pos_2 = message.pose.position.y 

    flag_pos_2 = 1

def route_callback_3(message):

    global flag_pos_3
    global x_pos_3 
    global y_pos_3 
    x_pos_3 = message.pose.position.x
    y_pos_3 = message.pose.position.y 

    flag_pos_3 = 1

def goal_callback_0(message):

    global flag_goal_0
    global x_goal_0 
    global y_goal_0
    x_goal_0 = message.pose.position.x
    y_goal_0 = message.pose.position.y

    global x_goal_points_0
    global y_goal_points_0
    if flag_goal_0 == 0: # first goal point
        x_goal_points_0.append(x_goal_0)
        y_goal_points_0.append(y_goal_0)
    flag_goal_0 = 1

def goal_callback_1(message):

    global flag_goal_1
    global x_goal_1 
    global y_goal_1
    x_goal_1 = message.pose.position.x
    y_goal_1 = message.pose.position.y

    global x_goal_points_1
    global y_goal_points_1
    if flag_goal_1 == 0: # first goal point
        x_goal_points_1.append(x_goal_1)
        y_goal_points_1.append(y_goal_1)
    flag_goal_1 = 1

def goal_callback_2(message):

    global flag_goal_2
    global x_goal_2 
    global y_goal_2
    x_goal_2 = message.pose.position.x
    y_goal_2 = message.pose.position.y

    global x_goal_points_2
    global y_goal_points_2
    if flag_goal_2 == 0: # first goal point
        x_goal_points_2.append(x_goal_2)
        y_goal_points_2.append(y_goal_2)
    flag_goal_2 = 1

def goal_callback_3(message):

    global flag_goal_3
    global x_goal_3 
    global y_goal_3
    x_goal_3 = message.pose.position.x
    y_goal_3 = message.pose.position.y

    global x_goal_points_3
    global y_goal_points_3
    if flag_goal_3 == 0: # first goal point
        x_goal_points_3.append(x_goal_3)
        y_goal_points_3.append(y_goal_3)
    flag_goal_3 = 1

def is_new_goal(x_old_value, y_old_value, x_new_value, y_new_value, x_goal_points, y_goal_points): # creating a list with the goal points
    
    x_new_goal = False
    y_new_goal = False

    if x_old_value != x_new_value:
        x_new_goal = True

    if y_old_value != y_new_value:
        y_new_goal = True

    if x_new_goal or y_new_goal:
        x_goal_points.append(x_new_value)
        y_goal_points.append(y_new_value)

    return x_goal_points, y_goal_points
    

def annotate_current_goal(x_goal_points, y_goal_points):

    i_goal =  len(x_goal_points)
    if i_goal == 1:
        plt.annotate(i_goal,xy=[x_goal_points[i_goal-1], y_goal_points[i_goal-1]], xytext=[x_goal_points[i_goal-1], y_goal_points[i_goal-1]+0.025])    
        # rospy.loginfo("current_goal %s, %s", x_goal_points[i_goal-1], y_goal_points[i_goal-1])
    if i_goal == 2:
        plt.annotate(i_goal-1,xy=[x_goal_points[i_goal-2], y_goal_points[i_goal-2]], xytext=[x_goal_points[i_goal-2], y_goal_points[i_goal-2]+0.025])
        plt.annotate(i_goal,xy=[x_goal_points[i_goal-1], y_goal_points[i_goal-1]], xytext=[x_goal_points[i_goal-1], y_goal_points[i_goal-1]+0.025])
        # rospy.loginfo("current_goal %s, %s", x_goal_points[i_goal-1], y_goal_points[i_goal-1])
    if i_goal == 3:
        plt.annotate(i_goal-2,xy=[x_goal_points[i_goal-3], y_goal_points[i_goal-3]], xytext=[x_goal_points[i_goal-3], y_goal_points[i_goal-3]+0.025])
        plt.annotate(i_goal-1,xy=[x_goal_points[i_goal-2], y_goal_points[i_goal-2]], xytext=[x_goal_points[i_goal-2], y_goal_points[i_goal-2]+0.025])
        plt.annotate(i_goal,xy=[x_goal_points[i_goal-1], y_goal_points[i_goal-1]], xytext=[x_goal_points[i_goal-1], y_goal_points[i_goal-1]+0.025])
        # rospy.loginfo("current_goal %s, %s", x_goal_points[i_goal-1], y_goal_points[i_goal-1])
    if i_goal == 4:
        plt.annotate(i_goal-3,xy=[x_goal_points[i_goal-4], y_goal_points[i_goal-4]], xytext=[x_goal_points[i_goal-4], y_goal_points[i_goal-4]+0.025])
        plt.annotate(i_goal-2,xy=[x_goal_points[i_goal-3], y_goal_points[i_goal-3]], xytext=[x_goal_points[i_goal-3], y_goal_points[i_goal-3]+0.025])
        plt.annotate(i_goal-1,xy=[x_goal_points[i_goal-2], y_goal_points[i_goal-2]], xytext=[x_goal_points[i_goal-2], y_goal_points[i_goal-2]+0.025])
        plt.annotate(i_goal,xy=[x_goal_points[i_goal-1], y_goal_points[i_goal-1]], xytext=[x_goal_points[i_goal-1], y_goal_points[i_goal-1]+0.025])
        # rospy.loginfo("current_goal %s, %s", x_goal_points[i_goal-1], y_goal_points[i_goal-1])
    if i_goal == 5:
        plt.annotate(i_goal-4,xy=[x_goal_points[i_goal-5], y_goal_points[i_goal-5]], xytext=[x_goal_points[i_goal-5], y_goal_points[i_goal-5]+0.025])
        plt.annotate(i_goal-3,xy=[x_goal_points[i_goal-4], y_goal_points[i_goal-4]], xytext=[x_goal_points[i_goal-4], y_goal_points[i_goal-4]+0.025])
        plt.annotate(i_goal-2,xy=[x_goal_points[i_goal-3], y_goal_points[i_goal-3]], xytext=[x_goal_points[i_goal-3], y_goal_points[i_goal-3]+0.025])
        plt.annotate(i_goal-1,xy=[x_goal_points[i_goal-2], y_goal_points[i_goal-2]], xytext=[x_goal_points[i_goal-2], y_goal_points[i_goal-2]+0.025])
        plt.annotate(i_goal,xy=[x_goal_points[i_goal-1], y_goal_points[i_goal-1]], xytext=[x_goal_points[i_goal-1], y_goal_points[i_goal-1]+0.025])
        # rospy.loginfo("current_goal %s, %s", x_goal_points[i_goal-1], y_goal_points[i_goal-1])
    if i_goal == 6:
        plt.annotate(i_goal-5,xy=[x_goal_points[i_goal-6], y_goal_points[i_goal-6]], xytext=[x_goal_points[i_goal-6], y_goal_points[i_goal-6]+0.025])
        plt.annotate(i_goal-4,xy=[x_goal_points[i_goal-5], y_goal_points[i_goal-5]], xytext=[x_goal_points[i_goal-5], y_goal_points[i_goal-5]+0.025])
        plt.annotate(i_goal-3,xy=[x_goal_points[i_goal-4], y_goal_points[i_goal-4]], xytext=[x_goal_points[i_goal-4], y_goal_points[i_goal-4]+0.025])
        plt.annotate(i_goal-2,xy=[x_goal_points[i_goal-3], y_goal_points[i_goal-3]], xytext=[x_goal_points[i_goal-3], y_goal_points[i_goal-3]+0.025])
        plt.annotate(i_goal-1,xy=[x_goal_points[i_goal-2], y_goal_points[i_goal-2]], xytext=[x_goal_points[i_goal-2], y_goal_points[i_goal-2]+0.025])
        plt.annotate(i_goal,xy=[x_goal_points[i_goal-1], y_goal_points[i_goal-1]], xytext=[x_goal_points[i_goal-1], y_goal_points[i_goal-1]+0.025])
        # rospy.loginfo("current_goal %s, %s", x_goal_points[i_goal-1], y_goal_points[i_goal-1])
    if i_goal == 7:
        plt.annotate(i_goal-6,xy=[x_goal_points[i_goal-7], y_goal_points[i_goal-7]], xytext=[x_goal_points[i_goal-7], y_goal_points[i_goal-7]+0.025])
        plt.annotate(i_goal-5,xy=[x_goal_points[i_goal-6], y_goal_points[i_goal-6]], xytext=[x_goal_points[i_goal-6], y_goal_points[i_goal-6]+0.025])
        plt.annotate(i_goal-4,xy=[x_goal_points[i_goal-5], y_goal_points[i_goal-5]], xytext=[x_goal_points[i_goal-5], y_goal_points[i_goal-5]+0.025])
        plt.annotate(i_goal-3,xy=[x_goal_points[i_goal-4], y_goal_points[i_goal-4]], xytext=[x_goal_points[i_goal-4], y_goal_points[i_goal-4]+0.025])
        plt.annotate(i_goal-2,xy=[x_goal_points[i_goal-3], y_goal_points[i_goal-3]], xytext=[x_goal_points[i_goal-3], y_goal_points[i_goal-3]+0.025])
        plt.annotate(i_goal-1,xy=[x_goal_points[i_goal-2], y_goal_points[i_goal-2]], xytext=[x_goal_points[i_goal-2], y_goal_points[i_goal-2]+0.025])
        plt.annotate(i_goal,xy=[x_goal_points[i_goal-1], y_goal_points[i_goal-1]], xytext=[x_goal_points[i_goal-1], y_goal_points[i_goal-1]+0.025])
        # rospy.loginfo("current_goal %s, %s", x_goal_points[i_goal-1], y_goal_points[i_goal-1])
    if i_goal == 8:
        plt.annotate(i_goal-7,xy=[x_goal_points[i_goal-8], y_goal_points[i_goal-8]], xytext=[x_goal_points[i_goal-8], y_goal_points[i_goal-8]+0.025])
        plt.annotate(i_goal-6,xy=[x_goal_points[i_goal-7], y_goal_points[i_goal-7]], xytext=[x_goal_points[i_goal-7], y_goal_points[i_goal-7]+0.025])
        plt.annotate(i_goal-5,xy=[x_goal_points[i_goal-6], y_goal_points[i_goal-6]], xytext=[x_goal_points[i_goal-6], y_goal_points[i_goal-6]+0.025])
        plt.annotate(i_goal-4,xy=[x_goal_points[i_goal-5], y_goal_points[i_goal-5]], xytext=[x_goal_points[i_goal-5], y_goal_points[i_goal-5]+0.025])
        plt.annotate(i_goal-3,xy=[x_goal_points[i_goal-4], y_goal_points[i_goal-4]], xytext=[x_goal_points[i_goal-4], y_goal_points[i_goal-4]+0.025])
        plt.annotate(i_goal-2,xy=[x_goal_points[i_goal-3], y_goal_points[i_goal-3]], xytext=[x_goal_points[i_goal-3], y_goal_points[i_goal-3]+0.025])
        plt.annotate(i_goal-1,xy=[x_goal_points[i_goal-2], y_goal_points[i_goal-2]], xytext=[x_goal_points[i_goal-2], y_goal_points[i_goal-2]+0.025])
        plt.annotate(i_goal,xy=[x_goal_points[i_goal-1], y_goal_points[i_goal-1]], xytext=[x_goal_points[i_goal-1], y_goal_points[i_goal-1]+0.025])
        # rospy.loginfo("current_goal %s, %s", x_goal_points[i_goal-1], y_goal_points[i_goal-1])
    if i_goal == 9:
        plt.annotate(i_goal-8,xy=[x_goal_points[i_goal-9], y_goal_points[i_goal-9]], xytext=[x_goal_points[i_goal-9], y_goal_points[i_goal-9]+0.025])
        plt.annotate(i_goal-7,xy=[x_goal_points[i_goal-8], y_goal_points[i_goal-8]], xytext=[x_goal_points[i_goal-8], y_goal_points[i_goal-8]+0.025])
        plt.annotate(i_goal-6,xy=[x_goal_points[i_goal-7], y_goal_points[i_goal-7]], xytext=[x_goal_points[i_goal-7], y_goal_points[i_goal-7]+0.025])
        plt.annotate(i_goal-5,xy=[x_goal_points[i_goal-6], y_goal_points[i_goal-6]], xytext=[x_goal_points[i_goal-6], y_goal_points[i_goal-6]+0.025])
        plt.annotate(i_goal-4,xy=[x_goal_points[i_goal-5], y_goal_points[i_goal-5]], xytext=[x_goal_points[i_goal-5], y_goal_points[i_goal-5]+0.025])
        plt.annotate(i_goal-3,xy=[x_goal_points[i_goal-4], y_goal_points[i_goal-4]], xytext=[x_goal_points[i_goal-4], y_goal_points[i_goal-4]+0.025])
        plt.annotate(i_goal-2,xy=[x_goal_points[i_goal-3], y_goal_points[i_goal-3]], xytext=[x_goal_points[i_goal-3], y_goal_points[i_goal-3]+0.025])
        plt.annotate(i_goal-1,xy=[x_goal_points[i_goal-2], y_goal_points[i_goal-2]], xytext=[x_goal_points[i_goal-2], y_goal_points[i_goal-2]+0.025])
        plt.annotate(i_goal,xy=[x_goal_points[i_goal-1], y_goal_points[i_goal-1]], xytext=[x_goal_points[i_goal-1], y_goal_points[i_goal-1]+0.025])
        # rospy.loginfo("current_goal %s, %s", x_goal_points[i_goal-1], y_goal_points[i_goal-1])
    if i_goal == 10:
        plt.annotate(i_goal-9,xy=[x_goal_points[i_goal-10], y_goal_points[i_goal-10]], xytext=[x_goal_points[i_goal-10], y_goal_points[i_goal-10]+0.025])
        plt.annotate(i_goal-8,xy=[x_goal_points[i_goal-9], y_goal_points[i_goal-9]], xytext=[x_goal_points[i_goal-9], y_goal_points[i_goal-9]+0.025])
        plt.annotate(i_goal-7,xy=[x_goal_points[i_goal-8], y_goal_points[i_goal-8]], xytext=[x_goal_points[i_goal-8], y_goal_points[i_goal-8]+0.025])
        plt.annotate(i_goal-6,xy=[x_goal_points[i_goal-7], y_goal_points[i_goal-7]], xytext=[x_goal_points[i_goal-7], y_goal_points[i_goal-7]+0.025])
        plt.annotate(i_goal-5,xy=[x_goal_points[i_goal-6], y_goal_points[i_goal-6]], xytext=[x_goal_points[i_goal-6], y_goal_points[i_goal-6]+0.025])
        plt.annotate(i_goal-4,xy=[x_goal_points[i_goal-5], y_goal_points[i_goal-5]], xytext=[x_goal_points[i_goal-5], y_goal_points[i_goal-5]+0.025])
        plt.annotate(i_goal-3,xy=[x_goal_points[i_goal-4], y_goal_points[i_goal-4]], xytext=[x_goal_points[i_goal-4], y_goal_points[i_goal-4]+0.025])
        plt.annotate(i_goal-2,xy=[x_goal_points[i_goal-3], y_goal_points[i_goal-3]], xytext=[x_goal_points[i_goal-3], y_goal_points[i_goal-3]+0.025])
        plt.annotate(i_goal-1,xy=[x_goal_points[i_goal-2], y_goal_points[i_goal-2]], xytext=[x_goal_points[i_goal-2], y_goal_points[i_goal-2]+0.025])
        plt.annotate(i_goal,xy=[x_goal_points[i_goal-1], y_goal_points[i_goal-1]], xytext=[x_goal_points[i_goal-1], y_goal_points[i_goal-1]+0.025])
        # rospy.loginfo("current_goal %s, %s", x_goal_points[i_goal-1], y_goal_points[i_goal-1])



def animate(i, ID):

    global x_pos_0
    global y_pos_0
    global flag_pos_0
    global i_goal_0

    global x_pos_1
    global y_pos_1
    global flag_pos_1
    global i_goal_1

    global x_pos_2
    global y_pos_2
    global flag_pos_2
    global i_goal_2

    global x_pos_3
    global y_pos_3
    global flag_pos_3
    global i_goal_3



    if flag_pos_0 != 0:
        x_pos_list_0.append(x_pos_0)
        y_pos_list_0.append(y_pos_0) 
    if flag_goal_0 != 0:
        x_goal_list_0.append(x_goal_0)
        y_goal_list_0.append(y_goal_0)

        old_x_0 = x_goal_list_0[len(x_goal_list_0) - 2]
        old_y_0 = y_goal_list_0[len(y_goal_list_0) - 2]
        new_x_0 = x_goal_list_0[len(x_goal_list_0) - 1]
        new_y_0 = y_goal_list_0[len(y_goal_list_0) - 1]
        global x_goal_points_0
        global y_goal_points_0
        x_goal_points_0, y_goal_points_0 = is_new_goal(old_x_0, old_y_0, new_x_0, new_y_0, x_goal_points_0, y_goal_points_0)

    if flag_pos_1 != 0:
        x_pos_list_1.append(x_pos_1)
        y_pos_list_1.append(y_pos_1) 
    if flag_goal_1 != 0:
        x_goal_list_1.append(x_goal_1)
        y_goal_list_1.append(y_goal_1)

        old_x_1 = x_goal_list_1[len(x_goal_list_1) - 2]
        old_y_1 = y_goal_list_1[len(y_goal_list_1) - 2]
        new_x_1 = x_goal_list_1[len(x_goal_list_1) - 1]
        new_y_1 = y_goal_list_1[len(y_goal_list_1) - 1]
        global x_goal_points_1
        global y_goal_points_1
        x_goal_points_1, y_goal_points_1 = is_new_goal(old_x_1, old_y_1, new_x_1, new_y_1, x_goal_points_1, y_goal_points_1) 

    if flag_pos_2 != 0:
        x_pos_list_2.append(x_pos_2)
        y_pos_list_2.append(y_pos_2) 
    if flag_goal_2 != 0:
        x_goal_list_2.append(x_goal_2)
        y_goal_list_2.append(y_goal_2)

        old_x_2 = x_goal_list_2[len(x_goal_list_2) - 2]
        old_y_2 = y_goal_list_2[len(y_goal_list_2) - 2]
        new_x_2 = x_goal_list_2[len(x_goal_list_2) - 1]
        new_y_2 = y_goal_list_2[len(y_goal_list_2) - 1]
        global x_goal_points_2
        global y_goal_points_2
        x_goal_points_2, y_goal_points_2 = is_new_goal(old_x_2, old_y_2, new_x_2, new_y_2, x_goal_points_2, y_goal_points_2) 

    if flag_pos_3 != 0:
        x_pos_list_3.append(x_pos_3)
        y_pos_list_3.append(y_pos_3) 
    if flag_goal_3 != 0:
        x_goal_list_3.append(x_goal_3)
        y_goal_list_3.append(y_goal_3)

        old_x_3 = x_goal_list_3[len(x_goal_list_3) - 2]
        old_y_3 = y_goal_list_3[len(y_goal_list_3) - 2]
        new_x_3 = x_goal_list_3[len(x_goal_list_3) - 1]
        new_y_3 = y_goal_list_3[len(y_goal_list_3) - 1]
        global x_goal_points_3
        global y_goal_points_3
        x_goal_points_3, y_goal_points_3 = is_new_goal(old_x_3, old_y_3, new_x_3, new_y_3, x_goal_points_3, y_goal_points_3)
            
    UAV_colors = {'0': 'r', '1': 'darkorange', '2': 'deepskyblue', '3': 'mediumorchid'}

    plt.cla()
    
    plt.title("All Iris" )
    plt.xlabel('x coordinate')
    plt.ylabel('y coordinate')

    Rout0 = plt.plot(x_pos_list_0, y_pos_list_0, label="Route for Iris_0", color = UAV_colors['0'])
    Rout1 = plt.plot(x_pos_list_1, y_pos_list_1, label="Route for Iris_1", color = UAV_colors['1'])
    Rout2 = plt.plot(x_pos_list_2, y_pos_list_2, label="Route for Iris_2", color = UAV_colors['2'])
    Rout3 = plt.plot(x_pos_list_3, y_pos_list_3, label="Route for Iris_3", color = UAV_colors['3'])

    plt.axis([-35, 35, -35, 35])

    Goal0 = plt.scatter(x_goal_list_0, y_goal_list_0, label="Goal_locations for Iris_0", color = 'darkred')
    Goal1 = plt.scatter(x_goal_list_1, y_goal_list_1, label="Goal_locations for Iris_1", color = 'chocolate')
    Goal2 = plt.scatter(x_goal_list_2, y_goal_list_2, label="Goal_locations for Iris_2", color = 'steelblue')
    Goal3 = plt.scatter(x_goal_list_3, y_goal_list_3, label="Goal_locations for Iris_3", color = 'rebeccapurple')
    
    Tall_obs = plt.scatter([-31, -31, -31],[-28, -10, 8], label = "Tall obstacles", color = 'g', marker='^')
    Short_obs = plt.scatter([27],[-28], label = "Short obstacles", color = 'aquamarine', marker='^')
    plt.grid()
    annotate_current_goal(x_goal_points_0, y_goal_points_0)
    annotate_current_goal(x_goal_points_1, y_goal_points_1)
    annotate_current_goal(x_goal_points_2, y_goal_points_2)
    annotate_current_goal(x_goal_points_3, y_goal_points_3)
    plt.legend(['Route for Iris_0', 'Route for Iris_1', 'Route for Iris_2', 'Route for Iris_3', 'Goal_locations for Iris_0' ,'Goal_locations for Iris_1', 'Goal_locations for Iris_2', 'Goal_locations for Iris_3', 'Tall obstacles', 'Short obstacles']) 
    

def trajectory_plot():

    rospy.init_node('trajectory_plot', anonymous=True)

    uav = []
    route_topic = []
    target_topic = []

    for ID in range(0,4):
        uav.append("uav" + str(ID))
        route_topic.append(uav[ID] + "/motion/position/global")
        target_topic.append(uav[ID] + "/motion/target_position/global")

    rospy.Subscriber("uav0/motion/position/global", PoseStamped, route_callback_0) 
    rospy.Subscriber(route_topic[1], PoseStamped, route_callback_1) 
    rospy.Subscriber(route_topic[2], PoseStamped, route_callback_2) 
    rospy.Subscriber(route_topic[3], PoseStamped, route_callback_3) 

    rospy.Subscriber(target_topic[0], PoseStamped, goal_callback_0)
    rospy.Subscriber(target_topic[1], PoseStamped, goal_callback_1)
    rospy.Subscriber(target_topic[2], PoseStamped, goal_callback_2)
    rospy.Subscriber(target_topic[3], PoseStamped, goal_callback_3)

    ani = FuncAnimation(plt.gcf(), animate, fargs=(ID,), interval=1000) 


    plt.show()
    
    # spin() -> keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    trajectory_plot()