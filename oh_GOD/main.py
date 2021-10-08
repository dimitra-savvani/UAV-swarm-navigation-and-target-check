#!/usr/bin/env python2

import heapq
import pygame
import sys

import rospy
from rospy.client import spin
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped
from motion.srv import new_point, new_pointResponse

from graph import Node, Graph
from grid import GridWorld
from utils import *
from d_star_lite import initDStarLite, moveAndRescan



# Define some colors
BLACK = (0, 0, 0)
WHITE = (255, 255, 255)
GREEN = (0, 220, 0)
GRAY1 = (145, 145, 102)
GRAY2 = (77, 77, 51)
BLUE = (0, 0, 80)
RED = (255, 0, 0)
ORANGE= (255, 100, 100)
LIGHTBLUE= (60, 150, 255)
LIGHTPURPLE= (153, 30, 153)

colors = { 0: WHITE, 1: GREEN, -1: GRAY1, -2: GRAY2 }
uav_colors = { 0: RED, 1: ORANGE, 2: LIGHTBLUE, 3: LIGHTPURPLE }

# This sets the WIDTH and HEIGHT of each grid location
WIDTH = 15
HEIGHT = 15

# This sets the margin between each cell
MARGIN = 1

# Create a 2 dimensional array. A two dimensional
# array is simply a list of lists.
grid = []
for row in range(10):
    # Add an empty array that will hold each cell
    # in this row
    grid.append([])
    for column in range(10):
        grid[row].append(0)  # Append a cell

# Set row 1, cell 5 to one. (Remember rows and
# column numbers start at zero.)
grid[1][5] = 1

# Initialize pygame
pygame.init()

X_DIM = 50
Y_DIM = 50
VIEWING_RANGE = 3




# Set the HEIGHT and WIDTH of the screen
WINDOW_SIZE = [(WIDTH + MARGIN) * X_DIM + MARGIN,
               (HEIGHT + MARGIN) * Y_DIM + MARGIN]
screen = pygame.display.set_mode(WINDOW_SIZE, pygame.RESIZABLE)

# Set title of screen
pygame.display.set_caption("D* Lite Path Planning")

# Declare swarm population
swarmPopulation = int(sys.argv[1])

# Loop until the user clicks the close button or all Uavs reach destination.
done = []
for i in range(swarmPopulation +1):
    # All list cells become true when the corresponding UAV reaches destination, but the last. 
    # The last cell becomes True either when all drones reach destination or when the user clicks the close button
    done.append(False) 

# Used to manage how fast the screen updates
clock = pygame.time.Clock()


""" ******************* """
""" CALLBACKS """
""" ******************* """

""" # rospy.Subscriber(Dstar_position_topic, String, starting_point_cb, (i))
def starting_point_cb(starting_point, args):
    ID = args
    starting_point_for_UAV[ID] = starting_point.data
    print("starting_point_cb for uav", ID, starting_point.data) """

""" ******************* """
""" SERVICE HANDLERS """
""" ******************* """
def target_point_handler(req, send_target_point):
    return new_pointResponse(req.requested_new_point, send_target_point)


if __name__ == "__main__":

    starting_point_for_UAV = [] 
    graph_for_UAV = []
    k_m = []

    """ PUBLISHERS """

    rospy.init_node('main_node', anonymous=False)

    for i in range(swarmPopulation):
        graph_for_UAV.append(GridWorld(X_DIM, Y_DIM)) # create a graph for each drone seperately
        k_m.append(0) # initialize k+m for all UAVs to 0
        #starting_point_for_UAV.append(navigator[i].get_starting_point(i)) # read initial UAVs position
    
    
        """ Dstar_position_topic =  "uav" + str(i) + "/motion/Dstar/position"
        starting_point = rospy.wait_for_message(Dstar_position_topic, String, timeout=None) # Subscribe once 
        starting_point_for_UAV.append(starting_point.data)
        print("point1") """
    for i in range(swarmPopulation):
        ROS_position_topic = "uav" + str(i) + "/motion/position/global"
    
        raw_ROS_starting_point = rospy.wait_for_message(ROS_position_topic, PoseStamped, timeout=None) # Subscriber to `ROS_position_topic_for_UAV[i]` only for one time
        
        starting_coordinate = { 
            "ROS_x" : int(round(raw_ROS_starting_point.pose.position.x)), # round coordinates to match the d* lite standards
            "ROS_y" : int(round(raw_ROS_starting_point.pose.position.y)) # round coordinates to match the d* lite standards
        }
        # rospy.loginfo("uav%s starting point is %s, %s (from nav_node)", i, starting_coordinate["ROS_x"], starting_coordinate["ROS_y"])

        (starting_coordinate["Dstar_x"], starting_coordinate["Dstar_y"]) = ROS_to_Dstar_coordinates(starting_coordinate["ROS_x"], starting_coordinate["ROS_y"], 1)

        starting_point_for_UAV.append("x" + str(starting_coordinate["Dstar_x"]) + "y" + str(starting_coordinate["Dstar_y"]))
        print("got_starting_point_from_initiator", starting_point_for_UAV[i], i)

    """ pool = mp.Pool(mp.cpu_count())
    
    starting_point_for_UAV = pool.map(get_initial_position, [ID for ID in range(swarmPopulation)])

    pool.close() """

    target_point = 'x25y25'
    target_coords = stateNameToCoords(target_point)

    send_target_point = PoseStamped()
    (send_target_point.pose.position.x, send_target_point.pose.position.y) = ROS_to_Dstar_coordinates(target_coords[0], target_coords[0], -1)
    send_target_point.pose.position.z = 2

    target_point_service = "motion/position/global/target"
    target_point_ser = rospy.Service(target_point_service, new_point, lambda msg: target_point_handler(msg, send_target_point))
    #SERVICE VERSION

    """ target_topic = "motion/position/global/target"

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
    
    target_published = False
    rate = rospy.Rate(1) # 1Hz
    while not target_published: # Publish once
        connections = target_pub.get_num_connections()
        if connections > 0:
            target_pub.publish(ROS_target_coordinate)
            rospy.loginfo("global target point is: %s, %s, %s", target_coordinate["ROS_x"], target_coordinate["ROS_y"], ROS_target_coordinate.pose.position.z)
            target_published = True
        else:
            rate.sleep """

    queue = []
    current_position_for_UAV = []
    pos_coords = []
    print("point3")
    for i in range(swarmPopulation):
        graph_for_UAV[i].setStart(starting_point_for_UAV[i]) # set initial UAV position for each drone, to its graph
        
        graph_for_UAV[i].setGoal(target_point) # set the goal position for each drone, to its graph
        
        queue.append([]) # list of lists for queues of each UAV

        graph_for_UAV[i], queue[i], k_m[i] = initDStarLite(graph_for_UAV[i], queue[i], starting_point_for_UAV[i], target_point, k_m[i])

        current_position_for_UAV.append(starting_point_for_UAV[i])

        pos_coords.append(stateNameToCoords(current_position_for_UAV[i]))


    basicfont = pygame.font.SysFont('Comic Sans MS', 16)

    # -------- Main Program Loop -----------
    while not done[-1]:
        for event in pygame.event.get():  # User did something
            s_new = [None]*swarmPopulation 
            if event.type == pygame.QUIT:  # If user `cliked close
                done[-1] = True  # Flag that we are done so we exit this loop
            elif event.type == pygame.KEYDOWN and event.key == pygame.K_RIGHT:
                s_new[0], k_m[0] = moveAndRescan(graph_for_UAV[0], queue[0], current_position_for_UAV[0], VIEWING_RANGE, k_m[0])  
                if s_new[0] == 'goal':
                    print('Goal Reached! uav0')
                    done[0] = True
                else:
                    print('setting current_position_for_UAV[0] to ', s_new[0])
                    current_position_for_UAV[0] = s_new[0]
                    pos_coords[0] = stateNameToCoords(current_position_for_UAV[0])
                    # print('got pos coords: ', pos_coords)
            elif event.type == pygame.KEYDOWN and event.key == pygame.K_LEFT:
                s_new[1], k_m[1] = moveAndRescan(graph_for_UAV[1], queue[1], current_position_for_UAV[1], VIEWING_RANGE, k_m[1])
                if s_new[1] == 'goal':
                    print('Goal Reached! uav1')
                    done[1] = True
                else:
                    print('setting current_position_for_UAV[1] to ', s_new[1])
                    current_position_for_UAV[1] = s_new[1]
                    pos_coords[1] = stateNameToCoords(current_position_for_UAV[1])
                    # print('got pos coords: ', pos_coords)
            elif event.type == pygame.KEYDOWN and event.key == pygame.K_DOWN:
                s_new[2], k_m[2] = moveAndRescan(graph_for_UAV[2], queue[2], current_position_for_UAV[2], VIEWING_RANGE, k_m[2])
                if s_new[2] == 'goal':
                    print('Goal Reached! uav2')
                    done[2] = True
                else:
                    print('setting current_position_for_UAV[2] to ', s_new[2])
                    current_position_for_UAV[2] = s_new[2]
                    pos_coords[2] = stateNameToCoords(current_position_for_UAV[2])
                    # print('got pos coords: ', pos_coords)
            elif event.type == pygame.KEYDOWN and event.key == pygame.K_UP:
                s_new[3], k_m[3] = moveAndRescan(graph_for_UAV[3], queue[3], current_position_for_UAV[3], VIEWING_RANGE, k_m[3])
                if s_new[3] == 'goal':
                    print('Goal Reached! uav3')
                    done[3] = True
                else:
                    print('setting current_position_for_UAV[3] to ', s_new[3])
                    current_position_for_UAV[3] = s_new[3]
                    pos_coords[3] = stateNameToCoords(current_position_for_UAV[3])
                    # print('got pos coords: ', pos_coords)
            if done[0] and done[1] and done[2] and done[3]:
                    done[-1] = True

            elif event.type == pygame.MOUSEBUTTONDOWN:
                # User clicks the mouse. Get the position
                pos = pygame.mouse.get_pos()
                # Change the x/y screen coordinates to grid coordinates
                column = pos[0] // (WIDTH + MARGIN) 
                row = pos[1] // (HEIGHT + MARGIN) 
                # Set that location to one
                if(graph_for_UAV[0].cells[row][column] == 0):
                    graph_for_UAV[0].cells[row][column] = -1
                if(graph_for_UAV[1].cells[row][column] == 0):
                    graph_for_UAV[1].cells[row][column] = -1
                if(graph_for_UAV[2].cells[row][column] == 0):
                    graph_for_UAV[2].cells[row][column] = -1
                if(graph_for_UAV[3].cells[row][column] == 0):
                    graph_for_UAV[3].cells[row][column] = -1

        # Set the screen background
        screen.fill(BLACK)

        # Draw the grid
        for row in range(Y_DIM):
            for column in range(X_DIM):
                color = WHITE
                # if grid[row][column] == 1:
                #     color = GREEN 
                #create the grid
                pygame.draw.rect(screen, colors[0], [(MARGIN + WIDTH) * column + MARGIN, (MARGIN + HEIGHT) * row + MARGIN, WIDTH, HEIGHT])
                node_name = 'x' + str(column) + 'y' + str(row)
                if graph_for_UAV[0].cells[row][column] == -1: # if there is an obstacle (we could use any graph e.g graph_for_UAV[1], and it would be the same)
                    pygame.draw.rect(screen, colors[graph_for_UAV[0].cells[row][column]], [(MARGIN + WIDTH) * column + MARGIN - WIDTH / 2, (MARGIN + HEIGHT) * row + MARGIN - HEIGHT / 2, WIDTH, HEIGHT])
                    node_name = 'x' + str(column) + 'y' + str(row) 
                    # print('unseen obstacle for uav: ', i, 'at',  node_name, 'with value', graph_for_UAV[i].cells[row][column])
                for i in range(swarmPopulation):
                    if graph_for_UAV[i].cells[row][column] == -2 : # obstacles that uav[i] has seen
                        pygame.draw.rect(screen, colors[graph_for_UAV[i].cells[row][column]], [(MARGIN + WIDTH) * column + MARGIN - WIDTH / 2, (MARGIN + HEIGHT) * row + MARGIN - HEIGHT / 2, WIDTH, HEIGHT])
                        node_name = 'x' + str(column) + 'y' + str(row) 
                        # print('obstacle for uav: ', i, 'at',  node_name, 'with value', graph_for_UAV[i].cells[row][column])
                    
                #    if(graph_for_UAV[1].graph[node_name].g != float('inf')):
                #        # text = basicfont.render(
                #        # str(graph_for_UAV.graph[node_name].g), True, (0, 0, 200), (255,
                #        # 255, 255))
                #        text = basicfont.render(str(graph_for_UAV[1].graph[node_name].g), True, (0, 200, 0))
                #        textrect = text.get_rect()
                #        textrect.centerx = int(column * (WIDTH + MARGIN) + WIDTH / 2) + MARGIN
                #        textrect.centery = int(row * (HEIGHT + MARGIN) + HEIGHT / 2) + MARGIN
                #        screen.blit(text, textrect)

        # fill in goal cell with GREEN
        pygame.draw.rect(screen, GREEN, [(MARGIN + WIDTH) * target_coords[0] + MARGIN - WIDTH / 4, (MARGIN + HEIGHT) * target_coords[1] + MARGIN - HEIGHT / 4, WIDTH / 2, HEIGHT / 2])
        # print('drawing robot pos_coords: ', pos_coords)
        # draw moving robot, based on pos_coords
        
        robot_center = []
        for i in range(swarmPopulation):

            robot_center.append([int(pos_coords[i][0] * (WIDTH + MARGIN)) +  MARGIN, int(pos_coords[i][1] * (HEIGHT + MARGIN)) + MARGIN]) # drone appears at the upper left corner of a square
            
            # draw UAVs
            pygame.draw.circle(screen, uav_colors[i], robot_center[i], int(WIDTH / 3) - 2)
        
            # draw UAVs viewing range
            pygame.draw.rect(screen, BLUE, [robot_center[i][0] - VIEWING_RANGE * (WIDTH + MARGIN), robot_center[i][1] - VIEWING_RANGE * (HEIGHT + MARGIN), 2 * VIEWING_RANGE * (WIDTH + MARGIN), 2 * VIEWING_RANGE * (HEIGHT + MARGIN)], 2)
        
        # Limit to 60 frames per second
        clock.tick(20)

        # Go ahead and update the screen with what we've drawn.
        pygame.display.flip()

    # Be IDLE friendly. If you forget this line, the program will 'hang'
    # on exit.
    pygame.quit()