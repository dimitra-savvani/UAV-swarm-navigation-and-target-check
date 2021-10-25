#!/usr/bin/env python2
from utils import stateNameToCoords, ROS_to_Dstar_coordinates

import rospy
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Point
from motion.srv import new_point, new_pointResponse

""" ******************* """
""" SERVICE HANDLERS """
""" ******************* """

def target_point_handler(req, target_point):
    return new_pointResponse(req.ready, target_point)


""" ******************* """
""" FUNCTIONS """
""" ******************* """

def get_initial_position(ID):

    ROS_position_topic = "uav" + str(ID) + "/motion/position/global"

    raw_ROS_starting_point = rospy.wait_for_message(ROS_position_topic, PoseStamped, timeout=None) # Subscriber to `ROS_position_topic_for_UAV[i]` only for one time
    
    starting_coordinate = { 
        "ROS_x" : int(round(raw_ROS_starting_point.pose.position.x)), # round coordinates to match the d* lite standards
        "ROS_y" : int(round(raw_ROS_starting_point.pose.position.y)) # round coordinates to match the d* lite standards
    }
    
    (starting_coordinate["Dstar_x"], starting_coordinate["Dstar_y"]) = ROS_to_Dstar_coordinates(starting_coordinate["ROS_x"], starting_coordinate["ROS_y"], 1)

    print("got_starting_point_from_initiator of uav" + str(ID))

    return "x" + str(starting_coordinate["Dstar_x"]) + "y" + str(starting_coordinate["Dstar_y"])
    

def set_target_point(target_coords, ID):
    target_point = PoseStamped()
    (target_point) = ROS_to_Dstar_coordinates(target_coords[0], target_coords[1], -1)

    target_point_service = "uav" + str(ID) + "/motion/position/global/target"
    target_point_ser = rospy.Service(target_point_service, new_point, lambda msg: target_point_handler(msg, target_point))

def locate_obstacles():

    # Dictionary to store obstacle positions
    # the obstacle positions have to correspond to the .world file that is called
    # the .world that is called, is specified in the  ~/src/Firmware/launch/simulation.launch 
    obs = {} 
    obs['telephone_pole'] = Point()
    obs['telephone_pole'].x, obs['telephone_pole'].y = 3, 4

    (obs['telephone_pole'].x, obs['telephone_pole'].y) = ROS_to_Dstar_coordinates(obs['telephone_pole'].x, obs['telephone_pole'].y, 1)    

    return obs 