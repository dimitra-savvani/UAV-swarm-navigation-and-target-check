from geometry_msgs.msg import PoseStamped
import rospy
flightHeight = rospy.get_param("/flightHeight") # param /flightHeight declared in simulation.launch file of motion package

x_displacement = 35
y_displacement = 28

def stateNameToCoords(name):
    return [int(name.split('x')[1].split('y')[0]), int(name.split('x')[1].split('y')[1])]

def ROS_to_Dstar_coordinates(in_x, in_y, direction):
    # direction controlls whether the coordinates are converted from ROS to pygame or from pygame to ROS, should either be 1 or -1.
    out_x = in_x + x_displacement*direction
    out_y = in_y + y_displacement*direction

    if direction == 1:
        return out_x, out_y
    else:
        position = PoseStamped()
        position.pose.position.x = out_x
        position.pose.position.y = out_y
        position.pose.position.z = flightHeight
        return position

