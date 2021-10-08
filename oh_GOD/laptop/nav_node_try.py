from utils import ROS_to_pygame_coordinates

import rospy
# from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped
from motion.srv import next_step, next_stepResponse
# from motion.msg import take_off
# from rospy.topics import Subscriber




""" ******************* """
""" SERVER """
""" ******************* """

def next_step_handler(req):
    req.requested_next_step.pose.position.x =  req.requested_next_step.pose.position.x + 3
    print(" Response ", next_stepResponse(req.requested_next_step) )
    print(" Request ", req.requested_next_step )
    return req.requested_next_step



def node_initiator():
    rospy.init_node('nav_node', anonymous=True)
    print("Hi I am nav_node initiator")
    


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
""" FUNCTION DECLARATIONS """
""" ******************* """


def starting_point_func(swarmPopulation):

    position_topic_for_UAV = [] # list to store position topics of UAVs 
    raw_ROS_starting_point_for_UAV = [] # list to store each uav' s initial position from topic_for_UAV[i]
    pygame_starting_point_for_UAV = [] # list for UAVs starting position for pygame graph
    for i in range(swarmPopulation):

        position_topic_for_UAV.append("uav" + str(i) + "/motion/position/global")
        
        raw_ROS_starting_point_for_UAV.append(rospy.wait_for_message(position_topic_for_UAV[i], PoseStamped, timeout=None)) # Subscriber to `position_topic_for_UAV[i]` only for one time
        
        starting_coordinate = { 
            "ROS_x" : int(round(raw_ROS_starting_point_for_UAV[i].pose.position.x)), # round coordinates to match the d* lite standards
            "ROS_y" : int(round(raw_ROS_starting_point_for_UAV[i].pose.position.y)) # round coordinates to match the d* lite standards
        }
        # rospy.loginfo("uav%s starting point is %s, %s (from nav_node)", i, starting_coordinate["ROS_x"], starting_coordinate["ROS_y"])

        (starting_coordinate["pygame_x"], starting_coordinate["pygame_y"]) = ROS_to_pygame_coordinates(starting_coordinate["ROS_x"], starting_coordinate["ROS_y"], 1)

        pygame_starting_point_for_UAV.append("x" + str(starting_coordinate["pygame_x"]) + "y" + str(starting_coordinate["pygame_y"]))
    
    return pygame_starting_point_for_UAV

def next_step_to_target(swarmPopulation):
    
    """ next_step_coordinate = {
        "pygame_x" : goal_coords[0],
        "pygame_y" : goal_coords[1]
    }
    (next_step_coordinate["ROS_x"], next_step_coordinate["ROS_y"]) = ROS_to_pygame_coordinates(next_step_coordinate["pygame_x"], next_step_coordinate["pygame_y"], -1)
    
    next_step_topic = "motion/position/global/target"
    next_step_pub = rospy.Publisher(next_step_topic, PoseStamped, queue_size=10)
    

    next_step_posestamped = PoseStamped()
    next_step_posestamped.pose.position.x = next_step_coordinate["ROS_x"]
    next_step_posestamped.pose.position.y = next_step_coordinate["ROS_y"]
    next_step_posestamped.pose.position.z = 2 
    rospy.loginfo("global next step from nav_node is %s, %s, %s", next_step_coordinate["ROS_x"], next_step_coordinate["ROS_y"], next_step_posestamped.pose.position.z)
     """
    next_step_service_for_UAV = []
    for i in range(swarmPopulation):
        next_step_service_for_UAV.append("uav" + str(i) + "/motion/next_step")
        print("call uav", str(i))
        next_step_service = rospy.Service(next_step_service_for_UAV[i], next_step, next_step_handler)
        rospy.spin()
    # next_step_pub.publish(next_step_posestamped)
    

""" def navigator(self):
    # uav = "uav" + ID
    take_off_topic = "motion/take_off_topic" + "uav0" 
    rospy.Subscriber(take_off_topic, take_off, self.flag_take_off_callback) 
    pass """
    
    