def stateNameToCoords(name):
    return [int(name.split('x')[1].split('y')[0]), int(name.split('x')[1].split('y')[1])]

def ROS_to_pygame_coordinates(in_x, in_y, direction):
    # direction controlls whether the coordinates are converted from ROS to pygame or from pygame to ROS, should either be 1 or -1.
    out_x = in_x + 25*direction
    out_y = in_y + 25*direction

    return out_x, out_y
