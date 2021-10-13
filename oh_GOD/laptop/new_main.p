#!/usr/bin/env python2

import heapq
import pygame
import sys

import rospy
from rospy.client import spin
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped
from motion.srv import flag, flagRequest

from graph import Node, Graph
from grid import GridWorld
from utils import *
from d_star_lite import initDStarLite, moveAndRescan
from ROS_functions import *



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

# ROS things
new_setpoint = PoseStamped()
forwarded_setpoint =  True # beacause the first setpoint is the take_off position

""" ******************* """
""" SERVICE HANDLERS """
""" ******************* """

def set_setpoint_handler(req):
    forwarded_setpoint = req.value
    return new_pointResponse(forwarded_setpoint, new_setpoint)


if __name__ == "__main__":

    starting_point_for_UAV = [] 
    graph_for_UAV = []
    k_m = []

    rospy.init_node('main_node', anonymous=False)

    
    for ID in range(swarmPopulation):
        graph_for_UAV.append(GridWorld(X_DIM, Y_DIM)) # create a graph for each drone seperately
        k_m.append(0) # initialize k+m for all UAVs to 0
        starting_point_for_UAV.append(get_initial_position(ID))
        setpoint_service = "uav" + str(ID) +  "/motion/position/global/setpoint"
        setpoint_ser = rospy.Service(setpoint_service, new_point, set_setpoint_handler)

    target_point = 'x25y25'
    target_coords = stateNameToCoords(target_point)
    set_target_point(target_coords)

    queue = []
    current_setpoint_for_UAV = []
    pos_coords = []

    for ID in range(swarmPopulation):
        graph_for_UAV[ID].setStart(starting_point_for_UAV[ID]) # set initial UAV position for each drone, to its graph
        
        graph_for_UAV[ID].setGoal(target_point) # set the goal position for each drone, to its graph
        
        queue.append([]) # list of lists for queues of each UAV

        graph_for_UAV[ID], queue[ID], k_m[ID] = initDStarLite(graph_for_UAV[ID], queue[ID], starting_point_for_UAV[ID], target_point, k_m[ID])

        current_setpoint_for_UAV.append(starting_point_for_UAV[ID])

        pos_coords.append(stateNameToCoords(current_setpoint_for_UAV[ID]))

    basicfont = pygame.font.SysFont('Comic Sans MS', 16)

    # -------- Main Program Loop -----------
    s_new = [None]*swarmPopulation
    expecting_setpoint_for_UAV = [False]*swarmPopulation
    while not done[-1]:
        # expecting_setpoint_for_UAV = [True, False, False, False] ####
        for event in pygame.event.get():  # User did something 
            if event.type == pygame.QUIT:  # If user `cliked close
                done[-1] = True  # Flag that we are done so we exit this loop
            else:
                expecting_setpoint_for_UAV = request_new_setpoint(expecting_setpoint_for_UAV) # check if uavs have reached setpoints
                for ID in range(swarmPopulation):
                    print("got in the for loop with ID: " + str(ID) + " with " + str(expecting_setpoint_for_UAV[ID]))
                    if expecting_setpoint_for_UAV[ID]:
                        print(str(expecting_setpoint_for_UAV[ID]) + " for uav" + str(ID)) 
                        forwarded_setpoint = False
                        s_new[ID], k_m[ID] = moveAndRescan(graph_for_UAV[ID], queue[ID], current_setpoint_for_UAV[ID], VIEWING_RANGE, k_m[ID])  
                        if s_new[ID] == 'goal':
                            print('Goal Reached! uav' + str(ID))
                            done[ID] = True
                        else:
                            print('setting current_setpoint_for_UAV' + str(ID) + " to " + s_new[ID])
                            current_setpoint_for_UAV[ID] = s_new[ID]
                            pos_coords[ID] = stateNameToCoords(current_setpoint_for_UAV[ID])
                            (new_setpoint) = ROS_to_Dstar_coordinates(pos_coords[ID][0], pos_coords[ID][1], -1)                       
                        while not forwarded_setpoint: 
                            print("trying to forward setpoint")
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