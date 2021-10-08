#!/usr/bin/env python2

from utils import ROS_to_Dstar_coordinates

import rospy
import sys
# from nav_msgs.msg import Odometry
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped
from motion.srv import new_point, new_pointResponse, new_pointRequest
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

def next_step_handler(self, req):
    req.request_new_point.pose.position.x =  req.request_new_point.pose.position.x + 3
    print(" Response ", new_pointResponse(req.request_new_point) )
    print(" Request ", req.request_new_point )
    return new_pointResponse(req.request_new_point)

""" ******************* """
""" FUNCTION DECLARATIONS """
""" ******************* """


def get_starting_point_from_initiator(ID):

    ROS_position_topic = "uav" + str(ID) + "/motion/position/global"
    
    raw_ROS_starting_point = rospy.wait_for_message(ROS_position_topic, PoseStamped, timeout=None) # Subscriber to `ROS_position_topic_for_UAV[i]` only for one time
    
    starting_coordinate = { 
        "ROS_x" : int(round(raw_ROS_starting_point.pose.position.x)), # round coordinates to match the d* lite standards
        "ROS_y" : int(round(raw_ROS_starting_point.pose.position.y)) # round coordinates to match the d* lite standards
    }
    # rospy.loginfo("uav%s starting point is %s, %s (from nav_node)", i, starting_coordinate["ROS_x"], starting_coordinate["ROS_y"])

    (starting_coordinate["Dstar_x"], starting_coordinate["Dstar_y"]) = ROS_to_Dstar_coordinates(starting_coordinate["ROS_x"], starting_coordinate["ROS_y"], 1)

    Dstar_starting_point = String()
    Dstar_starting_point = "x" + str(starting_coordinate["Dstar_x"]) + "y" + str(starting_coordinate["Dstar_y"])
    print("got_starting_point_from_initiator", Dstar_starting_point)

    return Dstar_starting_point


def set_starting_point_to_Dstar(ID, Dstar_starting_point):
    print("set_starting_point_to_Dstar")
    Dstar_position_topic = "uav" + str(ID) + "/motion/Dstar/position"
    
    starting_point_pub = rospy.Publisher(Dstar_position_topic, String, queue_size=1)
    
    published = False
    rate = rospy.Rate(1) # 1Hz
    while not published: # Publish once
        connections = starting_point_pub.get_num_connections()
        if connections > 0:
            starting_point_pub.publish(Dstar_starting_point) 
            published = True
        else:
            rate.sleep  
 
def get_target_from_Dstar():
    target_point_service = "motion/position/global/target_srv"

    rospy.wait_for_service(target_point_service)
    target_point_cl = rospy.ServiceProxy(target_point_service, new_point)
    target_point_rq = new_pointRequest()
    target_point_rq.requested_new_point = True
    target_point_rspns = target_point_cl(target_point_rq)
    print(target_point_rspns)


def set_target_to_initiator(target_coords):
    target_topic = "motion/position/global/target"

    target_coordinate = {
        "Dstar_x" : target_coords[0],
        "Dstar_y" : target_coords[1]
    }
    (target_coordinate["ROS_x"], target_coordinate["ROS_y"]) = ROS_to_Dstar_coordinates(target_coordinate["Dstar_x"], target_coordinate["Dstar_y"], -1)
    
    
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
    next_step_Service = rospy.Service(next_step_service, new_point, next_step_handler)
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
    

    set_starting_point_to_Dstar(ID, get_starting_point_from_initiator(ID))
    # get_target_from_Dstar()