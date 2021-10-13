#!/usr/bin/env python2

from utils import *

import rospy
from geometry_msgs.msg import PoseStamped
from motion.srv import new_point, new_pointResponse
from motion.srv import flag, flagRequest

""" ******************* """
""" SERVICE HANDLERS """
""" ******************* """

def target_point_handler(req, target_point):
    return new_pointResponse(req.value, target_point)

""" def set_setpoint_handler0(req, new_setpoint):
    return new_pointResponse(req.value, new_setpoint)

def set_setpoint_handler1(req, new_setpoint):
    return new_pointResponse(req.value, new_setpoint)

def set_setpoint_handler2(req, new_setpoint):
    return new_pointResponse(req.value, new_setpoint)

def set_setpoint_handler3(req, new_setpoint):
    return new_pointResponse(req.value, new_setpoint) """


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

    print("got_starting_point_from_initiator" + str(ID))

    return "x" + str(starting_coordinate["Dstar_x"]) + "y" + str(starting_coordinate["Dstar_y"])
    
def set_target_point(target_coords):
    target_point = PoseStamped()
    (target_point) = ROS_to_Dstar_coordinates(target_coords[0], target_coords[1], -1)

    target_point_service = "motion/position/global/target"
    target_point_ser = rospy.Service(target_point_service, new_point, lambda msg: target_point_handler(msg, target_point))


def request_new_setpoint(expecting_setpoint):
    for ID in range(len(expecting_setpoint)):
        reached_setpoint_service = "uav" + str(ID) + "/motion/reached_setpoint"
        rospy.wait_for_service(reached_setpoint_service)
        try:
            reached_setpoint_handler = rospy.ServiceProxy(reached_setpoint_service, flag)
            req = flagRequest()
            # req.value = True
            res = reached_setpoint_handler(req)

            expecting_setpoint[ID] = res.value
        except rospy.ServiceException as e:
            print("Service call failed: %s"%e)
    return expecting_setpoint 

""" def reached_new_point(ID):
    reached_new_point_service = "uav" + str(ID) + "/motion/reached_new_point"

    rospy.wait_for_service(reached_new_point_service)
    try:
        reached_new_point_handler = rospy.ServiceProxy(reached_new_point_service, flag)
        req = flagRequest()
        req.value = True
        res = reached_new_point_handler(req)
    except rospy.ServiceException as e:
            print("Service call failed: %s"%e)
    return res.value """

""" def set_setpoint(pos_coords, ID):

    new_setpoint = PoseStamped()
    (new_setpoint) = ROS_to_Dstar_coordinates(pos_coords[ID][0], pos_coords[ID][1], -1)


    setpoint_service = "uav" + str(ID) +  "/motion/position/global/setpoint" 
    # setpoint_ser = rospy.Service(setpoint_service, new_point, lambda msg: set_setpoint_handler(msg, new_setpoint))
    if ID == 0:
        setpoint_ser = rospy.Service(setpoint_service, new_point, lambda msg: set_setpoint_handler0(msg, new_setpoint))
    elif ID == 1:
        setpoint_ser = rospy.Service(setpoint_service, new_point, lambda msg: set_setpoint_handler1(msg, new_setpoint))
    elif ID == 2:
        setpoint_ser = rospy.Service(setpoint_service, new_point, lambda msg: set_setpoint_handler2(msg, new_setpoint))
    elif ID == 3:
        setpoint_ser = rospy.Service(setpoint_service, new_point, lambda msg: set_setpoint_handler3(msg, new_setpoint)) """

""" class Navigator:

    def __init__(self, ID):
        self.ID = ID

    def set_setpoint(pos_coords, ID):

        new_setpoint = PoseStamped()
        (new_setpoint.pose.position.x, new_setpoint.pose.position.y) = ROS_to_Dstar_coordinates(pos_coords[ID][0], pos_coords[ID][1], -1)
        new_setpoint.pose.position.z = 2

        setpoint_service = "uav" + str(ID) +  "/motion/position/global/setpoint" 
        setpoint_ser = rospy.Service(setpoint_service, new_point, lambda msg: set_setpoint_handler(msg, new_setpoint))
        
        setpoint_ser = rospy.Service(setpoint_service, new_point, lambda msg: set_setpoint_handler0(msg, new_setpoint))
         """