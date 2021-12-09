#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseStamped
import sys
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
x_goal_points = []
y_goal_points = []


def route_callback(message):

    global flag_pos
    global x_pos 
    global y_pos 
    x_pos = message.pose.position.x
    y_pos = message.pose.position.y 

    flag_pos = 1

def goal_callback(message):

    global flag_goal
    global x_goal 
    global y_goal
    x_goal = message.pose.position.x
    y_goal = message.pose.position.y

    global x_goal_points
    global y_goal_points
    if flag_goal == 0: # first goal point
        x_goal_points.append(x_goal)
        y_goal_points.append(y_goal)
    flag_goal = 1

def is_new_goal(x_old_value, y_old_value, x_new_value, y_new_value,): # creating a list with the goal points
    
    global x_goal_points
    global y_goal_points
    
    x_new_goal = False
    y_new_goal = False

    if x_old_value != x_new_value:
        x_new_goal = True

    if y_old_value != y_new_value:
        y_new_goal = True

    if x_new_goal or y_new_goal:
        x_goal_points.append(x_new_value)
        y_goal_points.append(y_new_value)
    

def annotate_current_goal():
    global x_goal_points
    global y_goal_points
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



def animate(i):

    global x_pos 
    global y_pos
    global flag_pos
    global i_goal


    if flag_pos != 0:
        x_pos_list.append(x_pos)
        y_pos_list.append(y_pos) 
    if flag_goal != 0:
        x_goal_list.append(x_goal)
        y_goal_list.append(y_goal)

        old_x = x_goal_list[len(x_goal_list) - 2]
        old_y = y_goal_list[len(y_goal_list) - 2]
        new_x = x_goal_list[len(x_goal_list) - 1]
        new_y = y_goal_list[len(y_goal_list) - 1]
        is_new_goal(old_x, old_y, new_x, new_y) 
            

    plt.cla()
    
    plt.title("Iris")
    plt.xlabel('x coordinate')
    plt.ylabel('y coordinate')
    plt.plot(x_pos_list, y_pos_list, label="Route", color = 'gray')
    plt.axis([-35, 35, -35, 35])
    plt.scatter(x_goal_list, y_goal_list, label="Goal_locations", color = 'firebrick')
    plt.scatter([-31, -31, -31],[-28, -10, 8], label = "Tall obstacles", color = 'g', marker='^')
    plt.scatter(27,-28, label = "Short obstacles", color = 'aquamarine', marker='^')
    annotate_current_goal()
    plt.legend(loc="upper right")
    

def trajectory_plot():

    rospy.init_node('trajectory_plot', anonymous=True)
    

    route_topic = "/motion/position/global"     
    target_topic = "/motion/target_position/global" 
    rospy.Subscriber(route_topic, PoseStamped, route_callback) 
    rospy.Subscriber(target_topic, PoseStamped, goal_callback)

    ani = FuncAnimation(plt.gcf(), animate, interval=1000) 

    plt.tight_layout()
    plt.show()
    
    # spin() -> keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    trajectory_plot()