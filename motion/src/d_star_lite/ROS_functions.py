#!/usr/bin/env python3
import math

from utils import  ROS_to_Dstar_coordinates

import rospy
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Point
from motion.srv import new_point, new_pointResponse


""" ******************* """
""" FUNCTIONS """
""" ******************* """

def get_UAV_position(ID):

    ROS_position_topic = "uav" + str(ID) + "/motion/position/global"

    raw_ROS_current_point = rospy.wait_for_message(ROS_position_topic, PoseStamped, timeout=None) # Subscriber to `ROS_position_topic_for_UAV[i]` only for one time
    
    current_coordinate = [ 
        int(round(raw_ROS_current_point.pose.position.x)), # round coordinates to match the d* lite standards
        int(round(raw_ROS_current_point.pose.position.y)), # round coordinates to match the d* lite standards
        int(round(raw_ROS_current_point.pose.position.z))
    ]
    
    (current_coordinate[0], current_coordinate[1]) = ROS_to_Dstar_coordinates(current_coordinate[0], current_coordinate[1], 1)

    # print("got_current_point_from_initiator of UAV" + str(ID))

    return current_coordinate

def set_UAV_target(ID, target_point):
    set_UAV_target_topic = "uav" + str(ID) + "/motion/position/global/target"
    pub = rospy.Publisher(set_UAV_target_topic, PoseStamped, queue_size=1)

def calculate_on_patrol_population(swarmPopulation, sensed_overheat):

    breaker = False

    previous_pair = {}
    previous_pair["x"] = 1
    previous_pair["y"] = 1 

    if sensed_overheat:
        attempted_on_patrol_population = math.floor(swarmPopulation / 2)
    else:
        attempted_on_patrol_population = swarmPopulation

    # calculate population and x, y dividers for slicing the grid in appropriate patrol areas
    for y_divider in range(1, attempted_on_patrol_population):
        for x_divider in range(y_divider, y_divider+2):
            if x_divider*y_divider is attempted_on_patrol_population:
                breaker = True
                break
            elif x_divider*y_divider > attempted_on_patrol_population:
                diff1 = attempted_on_patrol_population - (previous_pair["x"]*previous_pair["y"])
                diff2 = x_divider*y_divider - attempted_on_patrol_population
                if diff1 > diff2 and attempted_on_patrol_population != swarmPopulation: # only when there are UAVs on detect mode
                    breaker = True
                    break
                else: 
                    x_divider = previous_pair["x"]
                    y_divider = previous_pair["y"]
                    breaker = True
                    break
            previous_pair["x"] = x_divider
        if breaker:
            break
        previous_pair["y"] = y_divider
    on_patrol_population = x_divider*y_divider
    return on_patrol_population, x_divider, y_divider


def split_grid_for_patrol(x_divider, y_divider, X_DIM, Y_DIM): 

    patrolHeight = rospy.get_param("/patrolHeight")

    # length and width of each UAVs patrol area
    subarea_length  = math.ceil(X_DIM / x_divider)
    subarea_width = math.ceil(Y_DIM / y_divider)

    patrol_centers = [] # patrol_centers of patrol areas
    for i in range(x_divider):
        center_x = math.floor(subarea_length/2) + i*subarea_length
        for j in range(y_divider):
            center_y = math.floor(subarea_width/2) + j*subarea_width
            patrol_centers.append([center_x, center_y, patrolHeight])
    return patrol_centers, subarea_length, subarea_width


def distance(p1, p2): # calculate distance in 3 dimensions
    vertical_distance = math.sqrt( ((p1[0]-p2[0])**2)+((p1[1]-p2[1])**2) )
    dist = math.sqrt( (vertical_distance**2)+((p1[2]-p2[2])**2) )
    return dist


def assign_coverage_area_to_UAVs(swarmPopulation, on_patrol_population, patrol_centers):

    assigned_areas = [-1]*on_patrol_population

    for c_iterator in range(on_patrol_population):
        min_dist = 100 # if world gets much bigger this min_dist has to change to a bigger number
        for ID in range(swarmPopulation):
            if rospy.get_param("/mode" + str(ID)) == "patrol" and ID not in assigned_areas:              
                if min_dist > distance(get_UAV_position(ID), patrol_centers[c_iterator]):
                    min_dist = distance(get_UAV_position(ID), patrol_centers[c_iterator])
                    assigned_areas[c_iterator] = ID
    print("assigned_areas are: ")
    print(assigned_areas)                
    return assigned_areas

def send_UAVs_to_overheated_point(ID):
    position_topic = "uav" + str(ID) + "/mavros/local_position/pose"
    # uav + "/mavros/local_position/pose"
    


def locate_obstacles():

    # Dictionary to store obstacle positions
    # the obstacle positions have to correspond to the .world file that is called
    # the .world that is called, is specified in the  ~/src/Firmware/launch/simulation.launch 
    obs = {} 
    obs['telephone_pole'] = Point()
    obs['telephone_pole'].x, obs['telephone_pole'].y = 3, 4

    (obs['telephone_pole'].x, obs['telephone_pole'].y) = ROS_to_Dstar_coordinates(obs['telephone_pole'].x, obs['telephone_pole'].y, 1)    

    return obs 