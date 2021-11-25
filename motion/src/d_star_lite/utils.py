from geometry_msgs.msg import PoseStamped
import rospy


displacement = 35

def stateNameToCoords(name):
    return [int(name.split('x')[1].split('y')[0]), int(name.split('x')[1].split('y')[1])]

def coordsToStateName(coords):
    return "x" + str(coords[0]) + "y" + str(coords[1])

def ROS_to_Dstar_coordinates(in_x, in_y):
    # direction controlls whether the coordinates are converted from ROS to pygame or from pygame to ROS, (should either be 1 or -1).
    out_x = in_x + displacement
    out_y = in_y + displacement

    return out_x, out_y

def Dstar_to_ROS_coordinates(in_x, in_y, ID):
    # direction controlls whether the coordinates are converted from ROS to pygame or from pygame to ROS, (should either be 1 or -1).
    out_x = in_x - displacement
    out_y = in_y - displacement

    position = PoseStamped()
    position.pose.position.x = out_x
    position.pose.position.y = out_y

    mode = rospy.get_param("/mode" + str(ID))
    if mode == "detect":
        overheatHeight = rospy.get_param("/overheatHeight") # param /overheatHeight declared in simulation.launch file of motion package
        position.pose.position.z = overheatHeight
    else:
        patrolHeight = rospy.get_param("/patrolHeight") # param /patrolHeight declared in simulation.launch file of motion package
        position.pose.position.z = patrolHeight
    return position

