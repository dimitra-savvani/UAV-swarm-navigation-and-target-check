import rospy
from nav_msgs.msg import Odometry
from motion.msg import take_off

def initial_pos_callback(initial_pos_callback, args):
    pass

# def flag_take_off_callback(take_off_flag, args):
#     pass

def initialize_pos(ID):
    uav = "uav" + ID
    pos_topic = uav + "/mavros/global_position/local"
    rospy.Subscriber(pos_topic, Odometry, initial_pos_callback, (ID)) 
    return 1,1,1,1

def navigator():
    # uav = "uav" + ID
    # take_off_topic = uav + "/motion/take_off_topic"
    # rospy.Subscriber(take_off_topic, take_off, flag_take_off_callback, (ID)) 
    pass
