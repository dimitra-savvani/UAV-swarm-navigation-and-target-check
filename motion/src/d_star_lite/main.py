#!/usr/bin/env python2
# -*- coding: utf-8 -*-

import heapq
from pickle import TRUE
import pygame
from random import randint
import operator

from graph import Node, Graph
from grid import GridWorld
from utils import *
from d_star_lite import initDStarLite, moveAndRescan
from motion.srv import on_target, on_targetResponse
from motion.srv import new_point, new_pointResponse



# Define some colors
BLACK = (0, 0, 0)
WHITE = (255, 255, 255)
GREEN = (0, 220, 0)
LIGHTGREEN = (180, 225, 185)
GRAY1 = (145, 145, 102)
GRAY2 = (77, 77, 51)
BLUE = (0, 0, 80)
RED = (255, 0, 0)
ORANGE= (255, 100, 0)
LIGHTBLUE= (60, 150, 255)
LIGHTPURPLE= (153, 30, 153)

colors = { 0: WHITE, 1: GREEN, -1: GRAY1, -2: GRAY2 }
uav_colors = { 0: RED, 1: ORANGE, 2: LIGHTBLUE, 3: LIGHTPURPLE }

# This sets the WIDTH and HEIGHT of each grid location
WIDTH = 12
HEIGHT = 12

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

X_DIM = 70
Y_DIM = 70 # if you change dimension change displacement accordingly in utils.py
VIEWING_RANGE = rospy.get_param("/detectionDistance")


# Set the HEIGHT and WIDTH of the screen
WINDOW_SIZE = [(WIDTH + MARGIN) * X_DIM + MARGIN,
               (HEIGHT + MARGIN) * Y_DIM + MARGIN]
screen = pygame.display.set_mode(WINDOW_SIZE, pygame.RESIZABLE)

# Set title of screen
pygame.display.set_caption("D* Lite Path Planning")

# Declare swarm population
swarmPopulation = rospy.get_param("/swarmPopulation") # param /swarmPopulation declared in simulation.launch file of motion package

# Loop until the user clicks the close button.
done = False

# To signify that all Uavs reached target destination.
# All list cells become true when the corresponding UAV reaches destination, but the last. 
# The last cell becomes True when all drones reach destination  
made_it = [False]*(swarmPopulation +1)

extra_obs_coords = []

# To manage how fast the screen updates
clock = pygame.time.Clock()


""" ******************* """
""" FUNCTIONS """
""" ******************* """

def new_patrol_subtarget(id_param, center):
    # center[0] keeps the row value, center[1] keeps the column value
    
    graph_for_UAV[id_param] = GridWorld(X_DIM, Y_DIM)  
    static_obs_coords = graph_for_UAV[id_param].get_static_obs_coords()
    k_m[id_param] = 0
    starting_point_for_UAV[id_param] = current_position_for_UAV[id_param]

    new_target_x = randint(center[0] - math.floor(subarea_xdim/2), center[0] + math.floor(subarea_xdim/2))
    new_target_y = randint(center[1] - math.floor(subarea_ydim/2), center[1] + math.floor(subarea_ydim/2))
    
    while new_target_x not in range(X_DIM): # get new x coordinate for target if the given surpasses map dimensions
        new_target_x = randint(center[0] - math.floor(subarea_xdim/2), center[0] + math.floor(subarea_xdim/2))
    while new_target_y  not in range(Y_DIM): # get new y coordinate for target if the given surpasses map dimensions
        new_target_y = randint(center[1] - math.floor(subarea_ydim/2), center[1] + math.floor(subarea_ydim/2))
    while [new_target_x, new_target_y] in static_obs_coords: #if new target is an obstacle cell, corresponding to a static obstacle
        new_target_x = randint(center[0] - math.floor(subarea_xdim/2), center[0] + math.floor(subarea_xdim/2))
        new_target_y = randint(center[1] - math.floor(subarea_ydim/2), center[1] + math.floor(subarea_ydim/2))
    
    new_target = [new_target_x, new_target_y]
    target_point_for_UAV[id_param] = coordsToStateName(new_target)
    target_coords[id_param] = stateNameToCoords(target_point_for_UAV[id_param])
    target_point[id_param] = Dstar_to_ROS_coordinates(target_coords[id_param][0], target_coords[id_param][1], id_param)
    print("new patrol target for UAV" + str(ID))
    print(target_point[id_param].pose.position)
    graph_for_UAV[id_param].setStart(starting_point_for_UAV[id_param]) # set initial UAV position for UAV, to its graph
    graph_for_UAV[id_param].setGoal(target_point_for_UAV[id_param])
    queue[id_param] = []
    graph_for_UAV[id_param], queue[id_param], k_m[id_param] = initDStarLite(graph_for_UAV[id_param], queue[id_param], starting_point_for_UAV[id_param], target_point_for_UAV[id_param], k_m[id_param])
    pos_coords[id_param] = stateNameToCoords(current_position_for_UAV[id_param])

def target_overheated_point(id_param, overheat_sensed_at):
    global extra_obs_coords
    graph_for_UAV[id_param] = GridWorld(X_DIM, Y_DIM)
    static_obs_coords = graph_for_UAV[id_param].get_static_obs_coords()
    extra_obs_coords = graph_for_UAV[id_param].set_and_get_extra_obs()
    k_m[id_param] = 0
    starting_point_for_UAV[id_param] = current_position_for_UAV[id_param]

    target_point_for_UAV[id_param] = overheat_sensed_at
    target_coords[id_param] = stateNameToCoords(target_point_for_UAV[id_param])

    if target_coords[id_param] in static_obs_coords or target_coords[id_param] in extra_obs_coords:
        raise Exception("this location corresponds to an obstacle, try giving a different one")

    target_point[id_param] = Dstar_to_ROS_coordinates(target_coords[id_param][0], target_coords[id_param][1], id_param)

    graph_for_UAV[id_param].setStart(starting_point_for_UAV[id_param]) # set initial UAV position for UAV, to its graph
    graph_for_UAV[id_param].setGoal(target_point_for_UAV[id_param])
    queue[id_param] = []
    graph_for_UAV[id_param], queue[id_param], k_m[id_param] = initDStarLite(graph_for_UAV[id_param], queue[id_param], starting_point_for_UAV[id_param], target_point_for_UAV[id_param], k_m[id_param])
    pos_coords[id_param] = stateNameToCoords(current_position_for_UAV[id_param])

# ROS things

""" ******************* """
""" SERVICE HANDLERS """
""" ******************* """

class waypointing:

    def __init__(self, id):
        self.id = id

    def waypoint_handler(self, req):
        id = self.id
        reached_waypoint_for_UAV[id] = req.ready 
        
        # When the waypoint is ready, UAV accepts the new waypoint, 
        # and reached_waypoint_for_UAV[id] is set to False until UAV reaches the new waypoint
        if waypoint_ready[id]: 
            reached_waypoint_for_UAV[id] = False 
        return new_pointResponse(waypoint_ready[id], new_waypoint[id])


class reached_target_check:

    def __init__(self, id):
        self.id = id

    def on_target_handler(self, req):
        id  = self.id

        made_it[id] = req.arrival
        return on_targetResponse(req.arrival)


if __name__ == "__main__":

    starting_point_for_UAV = [] 
    graph_for_UAV = []
    k_m = []

    s_new = [None]*swarmPopulation
    reached_waypoint_for_UAV = [False]*swarmPopulation
    waypointer = []
    waypoint_ready = [False]*swarmPopulation
    new_waypoint = [PoseStamped()]*swarmPopulation

    can_accept_overheated_point = [False]*(swarmPopulation+1)
    sensed_overheat = False
    
    target_checker = []
    
    rospy.init_node('main_node', anonymous=False)
    rate = rospy.Rate(10)  # 1hz


    for ID in range(swarmPopulation):
        graph_for_UAV.append(GridWorld(X_DIM, Y_DIM)) # create a graph for each drone seperately
        k_m.append(0) # initialize k+m for all UAVs to 0
        starting_point_for_UAV.append(coordsToStateName(get_UAV_position(ID))) # get initial position from each drone
        waypointer.append(waypointing(ID)) # create separate objects for every UAV's waypoint handler
        waypoint_service = "uav" + str(ID) +  "/motion/position/global/waypoint"
        rospy.Service(waypoint_service, new_point, waypointer[ID].waypoint_handler)
    
        target_checker.append(reached_target_check(ID)) # create separate objects for every UAV's on_target handler
        reached_target_check_service = "uav" + str(ID) +  "/motion/on_target"
        rospy.Service(reached_target_check_service, on_target, target_checker[ID].on_target_handler)

    target_point_for_UAV = [] # 'x<x_coordinate>y<y_coordinate>'
        
    target_pub = []
    target_point = [PoseStamped()]*swarmPopulation 
    target_coords = []
    for ID in range(swarmPopulation):    
        target_point_for_UAV.append(starting_point_for_UAV[ID])
        target_coords.append(stateNameToCoords(target_point_for_UAV[ID])) 

        target_point[ID] = Dstar_to_ROS_coordinates(target_coords[ID][0], target_coords[ID][1], ID)
        target_point_topic = "uav" + str(ID) + "/motion/position/global/target"
        target_pub.append(rospy.Publisher(target_point_topic, PoseStamped, queue_size=10))

    queue = []
    current_position_for_UAV = []
    pos_coords = []
    two_last_waypoints = []
    for i in range(swarmPopulation):
        graph_for_UAV[i].setStart(starting_point_for_UAV[i]) # set initial UAV position for each drone, to its graph
  
        graph_for_UAV[i].setGoal(target_point_for_UAV[i]) # set the goal position for each drone, to its graph
        
        queue.append([]) # list of lists for queues of each UAV

        graph_for_UAV[i], queue[i], k_m[i] = initDStarLite(graph_for_UAV[i], queue[i], starting_point_for_UAV[i], target_point_for_UAV[i], k_m[i])
        
        current_position_for_UAV.append(starting_point_for_UAV[i])

        pos_coords.append(stateNameToCoords(current_position_for_UAV[i]))

        two_last_waypoints.append([pos_coords[i], pos_coords[i]])

    # assing patrol areas to swarm drones
    on_patrol_population, x_divider, y_divider = calculate_on_patrol_population(swarmPopulation, sensed_overheat)
    patrol_centers, subarea_xdim, subarea_ydim = split_grid_for_patrol(x_divider, y_divider, X_DIM, Y_DIM)
    UAVs_assigned_to_areas, on_detect_UAVS = assign_coverage_area_to_UAVs(swarmPopulation, on_patrol_population, patrol_centers, sensed_overheat, "no detected overheat yet")

    basicfont = pygame.font.SysFont('Comic Sans MS', 16)

    # -------- Main Program Loop -----------
    while not done:

        overheat_sensed_at = rospy.get_param("/overheat_sensed_at")
        # check if there is an overheat target
        if overheat_sensed_at is not "" and not sensed_overheat and min(can_accept_overheated_point[:-1]):
            sensed_overheat = True
            print("detected overheat")
            on_patrol_population, x_divider, y_divider = calculate_on_patrol_population(swarmPopulation, sensed_overheat)
            del(patrol_centers)
            patrol_centers, subarea_xdim, subarea_ydim = split_grid_for_patrol(x_divider, y_divider, X_DIM, Y_DIM)
            del(UAVs_assigned_to_areas)
            del(on_detect_UAVS)
            UAVs_assigned_to_areas, on_detect_UAVS = assign_coverage_area_to_UAVs(swarmPopulation, on_patrol_population, patrol_centers, sensed_overheat, overheat_sensed_at )
            on_detect_circle = {} 
            for ID in on_detect_UAVS:
                rospy.set_param("mode" + str(ID), "detect") # set apropriate UAV's mode to detect mode
                target_overheated_point(ID, overheat_sensed_at) # set overheated point as a terget for UAVs on detect mode
                on_detect_circle[ID] = False
                
                while True:
                    if target_pub[ID].get_num_connections() > 0:
                        target_pub[ID].publish(target_point[ID]) # publist the target to the UAVs on detect mode
                        break
                    else:
                        rate.sleep()
        # check if drones on detect mode should return to patrol mode
        if sensed_overheat and overheat_sensed_at is "":
            if min(on_detect_circle.items(), key=operator.itemgetter(1))[1]: # if every drone in detect mode is circling the overheated point
                for ID in on_detect_circle:
                    while (get_UAV_position(ID)[0:2] != two_last_waypoints[ID][0]): # if current position is the same as the last waypoint sent
                        print(get_UAV_position(ID)[0:2], two_last_waypoints[ID][0])
                sensed_overheat = False
                on_patrol_population, x_divider, y_divider = calculate_on_patrol_population(swarmPopulation, sensed_overheat)
                del(patrol_centers)
                patrol_centers, subarea_xdim, subarea_ydim = split_grid_for_patrol(x_divider, y_divider, X_DIM, Y_DIM)
                del(UAVs_assigned_to_areas)
                for ID in on_detect_UAVS:
                    # print("UAV" + str(ID) +" not anymore on detect mode")
                    rospy.set_param("mode" + str(ID), "patrol")
                del(on_detect_UAVS)
                UAVs_assigned_to_areas, on_detect_UAVS = assign_coverage_area_to_UAVs(swarmPopulation, on_patrol_population, patrol_centers, sensed_overheat, overheat_sensed_at )
                extra_obs_coords = []

        for ID in range(swarmPopulation):
            if reached_waypoint_for_UAV[ID] and not made_it[ID]:
                s_new[ID], k_m[ID] = moveAndRescan(graph_for_UAV[ID], queue[ID], current_position_for_UAV[ID], VIEWING_RANGE, k_m[ID])                
                # print('setting current_position_for_UAV' + str(ID) + ' to ' + s_new[ID])
                if s_new[ID] is not 'goal':
                    current_position_for_UAV[ID] = s_new[ID]
                # print("current position is: " + current_position_for_UAV[ID])
                pos_coords[ID] = stateNameToCoords(current_position_for_UAV[ID])
                
                two_last_waypoints[ID] = [pos_coords[ID], two_last_waypoints[ID][0]]
                graph_for_UAV = set_UAVs_as_obstacles(swarmPopulation, ID, graph_for_UAV, two_last_waypoints, X_DIM, Y_DIM, on_detect_UAVS)

                new_waypoint[ID] = Dstar_to_ROS_coordinates(pos_coords[ID][0], pos_coords[ID][1], ID)
                waypoint_ready[ID] = True
                while waypoint_ready[ID]:
                    waypoint_ready[ID] = reached_waypoint_for_UAV[ID] # set to false in the service handler when the coordinates are being sent

            if made_it[ID]: # when an UAV reaches target, it gets a new one
                if rospy.get_param("/mode" + str(ID)) == "patrol":
                    array_index = UAVs_assigned_to_areas.index(ID) # index on patrol_centers list that corresponds to this UAV's assigned area
                    new_patrol_subtarget(ID, patrol_centers[array_index])

                    while True:
                        if target_pub[ID].get_num_connections() > 0:
                            target_pub[ID].publish(target_point[ID])
                            print("sent target for UAV" + str(ID))
                            break
                        else:
                            rate.sleep()
                    made_it[ID] = False
                    can_accept_overheated_point[ID] = True
                elif rospy.get_param("/mode" + str(ID)) == "detect":
                    on_detect_circle[ID] = True
                    # when UAV circles around overheated point, clear its cells that are comsidered obstacles for other drones on detect mode
                    graph_for_UAV = clear_UAV_obs_cells (swarmPopulation, ID, graph_for_UAV, two_last_waypoints, X_DIM, Y_DIM) 
                    print(str(ID)+ "made it to overheated point")    

        if not made_it[-1] and min(made_it[:-1]): # min(made_it[:-1]) returns False if at least one of the UAVs has not reached destination
            made_it[-1] = True # set last cell of made_it list to true, only when all UAVs have reached destination 
            print("All UAVs are on target")
        
        for event in pygame.event.get():  # User did something
            if event.type == pygame.QUIT:  # If user clicked close
                done = True  # Flag that we are done so we exit this loop           
            elif event.type == pygame.MOUSEBUTTONDOWN:
                # User clicks the mouse. Get the position
                pos = pygame.mouse.get_pos()
                # Change the x/y screen coordinates to grid coordinates
                column = pos[0] // (WIDTH + MARGIN) 
                row = pos[1] // (HEIGHT + MARGIN)
                print(row, column) 
                # Set that location to one
                for ID in range(swarmPopulation):
                    if(graph_for_UAV[ID].cells[row][column] == 0):
                        graph_for_UAV[ID].cells[row][column] = -1

        # Set the screen background
        screen.fill(BLACK)

        # Draw the grid
        for row in range(Y_DIM):
            for column in range(X_DIM):
                
                color = WHITE
                # if grid[row][column] == 1:
                #      color = GREEN 
                # create the grid
                pygame.draw.rect(screen, colors[0], [(MARGIN + WIDTH) * column + MARGIN, (MARGIN + HEIGHT) * row + MARGIN, WIDTH, HEIGHT])
                node_name = 'x' + str(column) + 'y' + str(row)
                for ID in range(swarmPopulation):
                    if graph_for_UAV[ID].cells[row][column] == -1: # if there is an obstacle for any UAV (every UAV is considered as an obstacle to the others)
                        if [column, row] not in extra_obs_coords: # if this is an obstacle only on detect mode, color it light green
                            pygame.draw.rect(screen, colors[graph_for_UAV[ID].cells[row][column]], [(MARGIN + WIDTH) * column + MARGIN - WIDTH / 2, (MARGIN + HEIGHT) * row + MARGIN - HEIGHT / 2, WIDTH, HEIGHT])
                            node_name = 'x' + str(column) + 'y' + str(row)
                        else:
                            pygame.draw.rect(screen, LIGHTGREEN, [(MARGIN + WIDTH) * column + MARGIN - WIDTH / 2, (MARGIN + HEIGHT) * row + MARGIN - HEIGHT / 2, WIDTH, HEIGHT])
                            node_name = 'x' + str(column) + 'y' + str(row)
                        # print('unseen obstacle for uav: ', i, 'at',  node_name, 'with value', graph_for_UAV[i].cells[row][column])
                        break
                for ID in range(swarmPopulation):
                    if graph_for_UAV[ID].cells[row][column] == -2 : # obstacles that uav[i] has seen
                        pygame.draw.rect(screen, colors[graph_for_UAV[ID].cells[row][column]], [(MARGIN + WIDTH) * column + MARGIN - WIDTH / 2, (MARGIN + HEIGHT) * row + MARGIN - HEIGHT / 2, WIDTH, HEIGHT])
                        node_name = 'x' + str(column) + 'y' + str(row) 
                        break
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
        if overheat_sensed_at in target_point_for_UAV: # draw a green square to symbolize overheat point
            o_index = target_point_for_UAV.index(overheat_sensed_at)
            pygame.draw.rect(screen, GREEN, [(MARGIN + WIDTH) * target_coords[o_index][0] + MARGIN - WIDTH / 4, (MARGIN + HEIGHT) * target_coords[o_index][1] + MARGIN - HEIGHT / 4, WIDTH / 2, HEIGHT / 2])
        # print('drawing robot pos_coords: ', pos_coords)
        # draw moving robot, based on pos_coords
        
        robot_center = []
        for i in range(swarmPopulation):

            if target_point_for_UAV[i] != overheat_sensed_at: # if a UAV has different target than the overheated point, draw a square with the UAV's color to symbolize it
                pygame.draw.rect(screen, uav_colors[i], [(MARGIN + WIDTH) * target_coords[i][0] + MARGIN - WIDTH / 4, (MARGIN + HEIGHT) * target_coords[i][1] + MARGIN - HEIGHT / 4, WIDTH / 2, HEIGHT / 2])
            
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