#!/usr/bin/env python2

import heapq
import pygame

from nav_node import Navigator
from graph import Node, Graph
from grid import GridWorld
from utils import stateNameToCoords
from d_star_lite import initDStarLite, moveAndRescan

# Declare number of UAVs
Num_of_UAVs = 4


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

# Loop until the user clicks the close button or all Uavs reach destination.
done = []
for i in range(Num_of_UAVs +1):
    # All list cells become true when the corresponding UAV reaches destination, but the last. 
    # The last cell becomes True either when all drones reach destination or when the user clicks the close button
    done.append(False) 

# Used to manage how fast the screen updates
clock = pygame.time.Clock()

if __name__ == "__main__":
    graph = []
    k_m = []
    for i in range(Num_of_UAVs):
        graph.append(GridWorld(X_DIM, Y_DIM)) # create a graph for each drone seperately
        k_m.append(0) # initialize k+m for all UAVs to 0
    nav = Navigator()

    s_start = []
    (s_start) = nav.initialize_UAVs_pos(Num_of_UAVs) # read initial UAVs position
    # trrr = nav.navigator()
    s_goal = 'x25y25'
    goal_coords = stateNameToCoords(s_goal)
    nav.set_target_pos(goal_coords)

    queue = []
    s_current = []
    pos_coords = []
    for i in range(Num_of_UAVs):
        graph[i].setStart(s_start[i],) # set initial UAV position for each drone, to its graph
        
        graph[i].setGoal(s_goal) # set the goal position for each drone, to its graph
        
        queue.append([]) # list of lists for queues of each UAV

        graph[i], queue[i], k_m[i] = initDStarLite(graph[i], queue[i], s_start[i], s_goal, k_m[i])

        s_current.append(s_start[i])

        pos_coords.append(stateNameToCoords(s_current[i]))


    basicfont = pygame.font.SysFont('Comic Sans MS', 16)

    # -------- Main Program Loop -----------
    while not done[-1]:
        for event in pygame.event.get():  # User did something
            s_new = [None]*Num_of_UAVs 
            if event.type == pygame.QUIT:  # If user `cliked close
                done[-1] = True  # Flag that we are done so we exit this loop
            elif event.type == pygame.KEYDOWN and event.key == pygame.K_RIGHT:
                s_new[0], k_m[0] = moveAndRescan(graph[0], queue[0], s_current[0], VIEWING_RANGE, k_m[0])  
                if s_new[0] == 'goal':
                    print('Goal Reached! uav0')
                    done[0] = True
                else:
                    print('setting s_current[0] to ', s_new[0])
                    s_current[0] = s_new[0]
                    pos_coords[0] = stateNameToCoords(s_current[0])
                    # print('got pos coords: ', pos_coords)
            elif event.type == pygame.KEYDOWN and event.key == pygame.K_LEFT:
                s_new[1], k_m[1] = moveAndRescan(graph[1], queue[1], s_current[1], VIEWING_RANGE, k_m[1])
                if s_new[1] == 'goal':
                    print('Goal Reached! uav1')
                    done[1] = True
                else:
                    print('setting s_current[1] to ', s_new[1])
                    s_current[1] = s_new[1]
                    pos_coords[1] = stateNameToCoords(s_current[1])
                    # print('got pos coords: ', pos_coords)
            elif event.type == pygame.KEYDOWN and event.key == pygame.K_UP:
                s_new[2], k_m[2] = moveAndRescan(graph[2], queue[2], s_current[2], VIEWING_RANGE, k_m[2])
                if s_new[2] == 'goal':
                    print('Goal Reached! uav2')
                    done[2] = True
                else:
                    print('setting s_current[2] to ', s_new[2])
                    s_current[2] = s_new[2]
                    pos_coords[2] = stateNameToCoords(s_current[2])
                    # print('got pos coords: ', pos_coords)
            elif event.type == pygame.KEYDOWN and event.key == pygame.K_DOWN:
                s_new[3], k_m[3] = moveAndRescan(graph[3], queue[3], s_current[3], VIEWING_RANGE, k_m[3])
                if s_new[3] == 'goal':
                    print('Goal Reached! uav3')
                    done[3] = True
                else:
                    print('setting s_current[3] to ', s_new[3])
                    s_current[3] = s_new[3]
                    pos_coords[3] = stateNameToCoords(s_current[3])
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
                if(graph[0].cells[row][column] == 0):
                    graph[0].cells[row][column] = -1
                if(graph[1].cells[row][column] == 0):
                    graph[1].cells[row][column] = -1
                if(graph[2].cells[row][column] == 0):
                    graph[2].cells[row][column] = -1
                if(graph[3].cells[row][column] == 0):
                    graph[3].cells[row][column] = -1

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
                if graph[0].cells[row][column] == -1: # if there is an obstacle (we could use any graph e.g graph[1], and it would be the same)
                    pygame.draw.rect(screen, colors[graph[0].cells[row][column]], [(MARGIN + WIDTH) * column + MARGIN - WIDTH / 2, (MARGIN + HEIGHT) * row + MARGIN - HEIGHT / 2, WIDTH, HEIGHT])
                    node_name = 'x' + str(column) + 'y' + str(row) 
                    # print('unseen obstacle for uav: ', i, 'at',  node_name, 'with value', graph[i].cells[row][column])
                for i in range(Num_of_UAVs):
                    if graph[i].cells[row][column] == -2 : # obstacles that uav[i] has seen
                        pygame.draw.rect(screen, colors[graph[i].cells[row][column]], [(MARGIN + WIDTH) * column + MARGIN - WIDTH / 2, (MARGIN + HEIGHT) * row + MARGIN - HEIGHT / 2, WIDTH, HEIGHT])
                        node_name = 'x' + str(column) + 'y' + str(row) 
                        # print('obstacle for uav: ', i, 'at',  node_name, 'with value', graph[i].cells[row][column])
                    
                #    if(graph[1].graph[node_name].g != float('inf')):
                #        # text = basicfont.render(
                #        # str(graph.graph[node_name].g), True, (0, 0, 200), (255,
                #        # 255, 255))
                #        text = basicfont.render(str(graph[1].graph[node_name].g), True, (0, 200, 0))
                #        textrect = text.get_rect()
                #        textrect.centerx = int(column * (WIDTH + MARGIN) + WIDTH / 2) + MARGIN
                #        textrect.centery = int(row * (HEIGHT + MARGIN) + HEIGHT / 2) + MARGIN
                #        screen.blit(text, textrect)

        # fill in goal cell with GREEN
        pygame.draw.rect(screen, GREEN, [(MARGIN + WIDTH) * goal_coords[0] + MARGIN - WIDTH / 4, (MARGIN + HEIGHT) * goal_coords[1] + MARGIN - HEIGHT / 4, WIDTH / 2, HEIGHT / 2])
        # print('drawing robot pos_coords: ', pos_coords)
        # draw moving robot, based on pos_coords
        
        robot_center = []
        for i in range(Num_of_UAVs):

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