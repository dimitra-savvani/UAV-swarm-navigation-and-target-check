#!/usr/bin/env python3
import math

from utils import  ROS_to_Dstar_coordinates, stateNameToCoords

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
    
    (current_coordinate[0], current_coordinate[1]) = ROS_to_Dstar_coordinates(current_coordinate[0], current_coordinate[1])

    # print("got_current_point_from_initiator of UAV" + str(ID))

    return current_coordinate


def calculate_on_patrol_population(swarmPopulation, sensed_overheat):

    breaker = False

    previous_pair = {}
    previous_pair["x"] = 1
    previous_pair["y"] = 1 

    if sensed_overheat:
        attempted_on_patrol_population = int(math.floor(swarmPopulation / 2))
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


def assign_coverage_area_to_UAVs(swarmPopulation, on_patrol_population, patrol_centers, sensed_overheat, overheat_sensed_at):

    assigned_areas = [-1]*on_patrol_population

    on_detect_population = swarmPopulation-on_patrol_population
    on_detect_UAVS = [-1]*on_detect_population

    if sensed_overheat: # set UAVs that are closer to the overheat on detect mode(mode is actually set in the main program)

        overheat_pos = stateNameToCoords(overheat_sensed_at)
        overheatHeight = rospy.get_param("/overheatHeight")
        overheat_pos.append(overheatHeight)

        for d_iterator in range(on_detect_population):
            min_dist = 100
            for ID in range(swarmPopulation):
                if ID not in on_detect_UAVS:
                    if min_dist > distance(get_UAV_position(ID), overheat_pos):
                        min_dist = distance(get_UAV_position(ID), overheat_pos)
                        on_detect_UAVS[d_iterator] = ID

    for c_iterator in range(on_patrol_population):
        min_dist = 100 # if world gets much bigger this min_dist has to change to a bigger number
        for ID in range(swarmPopulation):
            if ID not in on_detect_UAVS and ID not in assigned_areas: # assign areas to patrol, for the UAVs that are not on detect mode           
                if min_dist > distance(get_UAV_position(ID), patrol_centers[c_iterator]): # select which drone patrols each area, based on which one is closer to the center of that area
                    min_dist = distance(get_UAV_position(ID), patrol_centers[c_iterator])
                    assigned_areas[c_iterator] = ID
    print("assigned_areas IDs are: ")
    print(assigned_areas)

    """ print("on_detect_UAVS are: ")
    print(on_detect_UAVS) """

    return assigned_areas


def locate_obstacles():

    # Dictionary to store obstacle positions
    # the obstacle positions have to correspond to the .world file that is called
    # the .world that is called, is specified in the  ~/src/Firmware/launch/simulation.launch 
    obs = {} 
    obs['telephone_pole'] = Point()
    obs['telephone_pole'].x, obs['telephone_pole'].y = 3, 4

    (obs['telephone_pole'].x, obs['telephone_pole'].y) = ROS_to_Dstar_coordinates(obs['telephone_pole'].x, obs['telephone_pole'].y)    

    return obs 