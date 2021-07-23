from utils import ROS_to_pygame_coordinates

import rospy
from nav_msgs.msg import Odometry
from motion.msg import take_off
from motion.msg import global_current_pos
from rospy.topics import Subscriber


class Navigator:

    def __init__(self):
        pass

    def flag_take_off_callback(self, take_off_flag):
        # rospy.loginfo("%s", take_off_flag)
        pass

    """ # rospy.Subscriber(pos_topic, Odometry, self.initial_pos_callback0)
    def initial_pos_callback0(self, initial_pos0):
        self.initial_pos0 = []
        self.initial_pos0.append(initial_pos0.pose.pose.position.x)
        self.initial_pos0.append(initial_pos0.pose.pose.position.y)
        rospy.loginfo("lsala %s", self.initial_pos0) """

    def initialize_UAVs_pos(self, Num_of_UAVs):

        uav_pos_topic = [] # list to store position topics of UAVs 
        uav_initial_pos = [] # list to store each uav' s initial position from global_current_pos.msg of uav_pos_topic[i]
        s_start = [] # list for UAVs starting position on pygame graph
        for i in range(Num_of_UAVs):

            uav_pos_topic.append("uav" + str(i) + "/motion/global_current_pos_topic")
         
            uav_initial_pos.append(rospy.wait_for_message(uav_pos_topic[i], global_current_pos, timeout=None)) # Subscriber to `uav_pos_topic[i]` only for one time
            rospy.loginfo("uav%s position is %s, %s from nav_node", i, int(round(uav_initial_pos[i].position.x)), int(round(uav_initial_pos[i].position.y)))

            R_x = int(round(uav_initial_pos[i].position.x))
            R_y = int(round(uav_initial_pos[i].position.y))

            (p_x, p_y) = ROS_to_pygame_coordinates(R_x, R_y, 1)

            s_start.append("x" + str(p_x) + "y" + str(p_y))
        
        return s_start

    def set_target_pos(self, goal_coords):
        (R_goal_x, R_goal_y) = ROS_to_pygame_coordinates(goal_coords[0], goal_coords[1], -1)
        
        goal_pos_topic = "goal/motion/global_current_pos_topic"
        goal_pos_pub = rospy.Publisher(goal_pos_topic, global_current_pos, queue_size=10)
        rospy.init_node('nav_node', anonymous=True)

        goal_pos = global_current_pos()
        goal_pos.position.x = R_goal_x
        goal_pos.position.y = R_goal_y
        goal_pos.position.z = 2 
        rospy.loginfo("global goal position from nav_node is %s, %s, %s", R_goal_x, R_goal_y, goal_pos.position.z)

        goal_pos_pub.publish(goal_pos)
       

    def navigator(self):
        # uav = "uav" + ID
        take_off_topic = "uav0" + "/motion/take_off_topic"
        rospy.Subscriber(take_off_topic, take_off, self.flag_take_off_callback) 
        pass
    
    