#!/usr/bin/env python2

from utils import ROS_to_pygame_coordinates

import rospy
import sys
# from nav_msgs.msg import Odometry
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped
from motion.srv import next_step, next_stepResponse
from motion.srv import ros_to_Dstar, ros_to_DstarResponse
# from motion.msg import take_off
# from rospy.topics import Subscriber

""" ******************* """
""" CALLBACKS """
""" ******************* """

""" def flag_take_off_callback(self, take_off_flag):
    # rospy.loginfo("%s", take_off_flag)
    pass """

""" # rospy.Subscriber(pos_topic, Odometry, self.initial_pos_callback0)
def initial_pos_callback0(self, initial_pos0):
    self.initial_pos0 = []
    self.initial_pos0.append(initial_pos0.pose.pose.position.x)
    self.initial_pos0.append(initial_pos0.pose.pose.position.y)
    rospy.loginfo("lsala %s", self.initial_pos0) """


""" ******************* """
""" SERVER HANDLERS """
""" ******************* """

def starting_point_handler(req, pygame_starting_point):
    return ros_to_DstarResponse(pygame_starting_point)

def next_step_handler(req):
    req.requested_next_step.pose.position.x =  req.requested_next_step.pose.position.x + 3
    print(" Response ", next_stepResponse(req.requested_next_step) )
    print(" Request ", req.requested_next_step )
    return next_stepResponse(req.requested_next_step)

""" ******************* """
""" FUNCTION DECLARATIONS """
""" ******************* """


def get_starting_point_from_initiator(ID):
    print("get_starting_point_from_initiator")
    ROS_position_topic = "uav" + str(ID) + "/motion/position/global"
    
    raw_ROS_starting_point = rospy.wait_for_message(ROS_position_topic, PoseStamped, timeout=None) # Subscriber to `ROS_position_topic_for_UAV[i]` only for one time
    
    starting_coordinate = { 
        "ROS_x" : int(round(raw_ROS_starting_point.pose.position.x)), # round coordinates to match the d* lite standards
        "ROS_y" : int(round(raw_ROS_starting_point.pose.position.y)) # round coordinates to match the d* lite standards
    }
    # rospy.loginfo("uav%s starting point is %s, %s (from nav_node)", i, starting_coordinate["ROS_x"], starting_coordinate["ROS_y"])

    (starting_coordinate["pygame_x"], starting_coordinate["pygame_y"]) = ROS_to_pygame_coordinates(starting_coordinate["ROS_x"], starting_coordinate["ROS_y"], 1)

    pygame_starting_point = String()
    pygame_starting_point = "x" + str(starting_coordinate["pygame_x"]) + "y" + str(starting_coordinate["pygame_y"])
    
    return pygame_starting_point

def set_starting_point_to_pygame(ID, pygame_starting_point):
    print("set_starting_point_to_pygame")
    pygame_position_service = "uav" + str(ID) + "/motion/pygame/position"
    
    starting_point_ser = rospy.Service(pygame_position_service, ros_to_Dstar, lambda msg: starting_point_handler(msg, pygame_starting_point))   

def set_target(target_coords):
    target_topic = "motion/position/global/target"

    target_coordinate = {
        "pygame_x" : target_coords[0],
        "pygame_y" : target_coords[1]
    }
    (target_coordinate["ROS_x"], target_coordinate["ROS_y"]) = ROS_to_pygame_coordinates(target_coordinate["pygame_x"], target_coordinate["pygame_y"], -1)
    
    
    target_pub = rospy.Publisher(target_topic, PoseStamped, queue_size=10)
    

    ROS_target_coordinate = PoseStamped()
    ROS_target_coordinate.pose.position.x = target_coordinate["ROS_x"]
    ROS_target_coordinate.pose.position.y = target_coordinate["ROS_y"]
    ROS_target_coordinate.pose.position.z = 2 
    rospy.loginfo("global target point is: %s, %s, %s", target_coordinate["ROS_x"], target_coordinate["ROS_y"], ROS_target_coordinate.pose.position.z)
    target_pub.publish(ROS_target_coordinate)

def next_step_to_target(ID):
    
    
    next_step_service = "uav" + str(ID) + "/motion/next_step"
    print("call uav", str(ID))
    next_step_ser = rospy.Service(next_step_service, next_step, next_step_handler)
    rospy.spin()
    
    

""" def navigator(self):
    # uav = "uav" + ID
    take_off_topic = "motion/take_off_topic" + "uav0" 
    rospy.Subscriber(take_off_topic, take_off, self.flag_take_off_callback) 
    pass """



if __name__ == "__main__":
    ID = sys.argv[1]
    rospy.init_node('nav_node_' + ID, anonymous=False)

    """ SUBSCRIBERS """
    

    """ PUBLISHERS """
    

    set_starting_point_to_pygame(ID, get_starting_point_from_initiator(ID))
    rospy.spin()