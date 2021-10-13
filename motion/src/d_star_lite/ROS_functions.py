#!/usr/bin/env python2

from utils import *

import rospy
from geometry_msgs.msg import PoseStamped
from motion.srv import new_point, new_pointResponse

""" ******************* """
""" SERVICE HANDLERS """
""" ******************* """

def target_point_handler(req, target_point):
    return new_pointResponse(req.value, target_point)


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

    print("got_starting_point_from_initiator", ID)

    return "x" + str(starting_coordinate["Dstar_x"]) + "y" + str(starting_coordinate["Dstar_y"])
    
def set_target_point(target_coords):
    target_point = PoseStamped()
    (target_point) = ROS_to_Dstar_coordinates(target_coords[0], target_coords[1], -1)

    target_point_service = "motion/position/global/target"
    target_point_ser = rospy.Service(target_point_service, new_point, lambda msg: target_point_handler(msg, target_point))