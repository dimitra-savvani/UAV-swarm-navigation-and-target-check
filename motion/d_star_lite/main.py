#!/usr/bin/env python2

import heapq
import pygame
import sys

from nav_node import initialize_pos, navigator
from graph import Node, Graph
from grid import GridWorld
from utils import stateNameToCoords
from d_star_lite import initDStarLite, moveAndRescan

# Callbacks





# Define some colors
BLACK = (0, 0, 0)
WHITE = (255, 255, 255)
GREEN = (0, 220, 0)
RED = (255, 0, 0)
ORANGE= (255, 100, 100)
LIGHTBLUE= (60, 150, 255)
LIGHTPURPLE= (153, 30, 153)
GRAY1 = (145, 145, 102)
GRAY2 = (77, 77, 51)
BLUE = (0, 0, 80)

colors = { 0: WHITE, 1: GREEN, -1: GRAY1, -2: GRAY2 }

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

# Loop until the user clicks the close button.
done = False
done0 = False
done1 = False
done2 = False
done3 = False

# Used to manage how fast the screen updates
clock = pygame.time.Clock()

if __name__ == "__main__":
    graph0 = GridWorld(X_DIM, Y_DIM)
    graph1 = GridWorld(X_DIM, Y_DIM)
    graph2 = GridWorld(X_DIM, Y_DIM)
    graph3 = GridWorld(X_DIM, Y_DIM)
    ID = sys.argv[1] #drone ID
    (s0_start, s1_start, s2_start, s3_start) = initialize_pos(ID)
    s0_start = 'x40y25'
    s1_start = 'x10y25'
    s2_start = 'x25y10'
    s3_start = 'x25y40'
    s_goal = 'x25y25'
    goal_coords = stateNameToCoords(s_goal)

    graph0.setStart(s0_start,)
    graph1.setStart(s1_start,)
    graph2.setStart(s2_start,)
    graph3.setStart(s3_start,)
    
    graph0.setGoal(s_goal)
    graph1.setGoal(s_goal)
    graph2.setGoal(s_goal)
    graph3.setGoal(s_goal)
    k_m0 = 0
    k_m1 = 0
    k_m2 = 0
    k_m3 = 0
    s0_last = s0_start
    s1_last = s1_start
    s2_last = s2_start
    s3_last = s3_start
    queue0 = []
    queue1 = []
    queue2 = []
    queue3 = []

    graph0, queue0, k_m0 = initDStarLite(graph0, queue0, s0_start, s_goal, k_m0)
    graph1, queue1, k_m1 = initDStarLite(graph1, queue1, s1_start, s_goal, k_m1)
    graph2, queue2, k_m2 = initDStarLite(graph2, queue2, s2_start, s_goal, k_m2)
    graph3, queue3, k_m3 = initDStarLite(graph3, queue3, s3_start, s_goal, k_m3)

    s0_current = s0_start
    s1_current = s1_start
    s2_current = s2_start
    s3_current = s3_start

    pos0_coords = stateNameToCoords(s0_current)
    pos1_coords = stateNameToCoords(s1_current)
    pos2_coords = stateNameToCoords(s2_current)
    pos3_coords = stateNameToCoords(s3_current)

    basicfont = pygame.font.SysFont('Comic Sans MS', 16)

    # -------- Main Program Loop -----------
    while not done:
        for event in pygame.event.get():  # User did something
            if event.type == pygame.QUIT:  # If user `cliked close
                done = True  # Flag that we are done so we exit this loop
            elif event.type == pygame.KEYDOWN and event.key == pygame.K_RIGHT:
                s0_new, k_m0 = moveAndRescan(graph0, queue0, s0_current, VIEWING_RANGE, k_m0)  
                if s0_new == 'goal':
                    print('Goal Reached! uav0')
                    done0 = True
                else:
                    print('setting s0_current to ', s0_new)
                    s0_current = s0_new
                    pos0_coords = stateNameToCoords(s0_current)
                    # print('got pos coords: ', pos_coords)
            elif event.type == pygame.KEYDOWN and event.key == pygame.K_LEFT:
                s1_new, k_m1 = moveAndRescan(graph1, queue1, s1_current, VIEWING_RANGE, k_m1)
                if s1_new == 'goal':
                    print('Goal Reached! uav1')
                    done1 = True
                else:
                    print('setting s1_current to ', s1_new)
                    s1_current = s1_new
                    pos1_coords = stateNameToCoords(s1_current)
                    # print('got pos coords: ', pos_coords)
            elif event.type == pygame.KEYDOWN and event.key == pygame.K_UP:
                s2_new, k_m2 = moveAndRescan(graph2, queue2, s2_current, VIEWING_RANGE, k_m2)
                if s2_new == 'goal':
                    print('Goal Reached! uav2')
                    done2 = True
                else:
                    print('setting s2_current to ', s2_new)
                    s2_current = s2_new
                    pos2_coords = stateNameToCoords(s2_current)
                    # print('got pos coords: ', pos_coords)
            elif event.type == pygame.KEYDOWN and event.key == pygame.K_DOWN:
                s3_new, k_m3 = moveAndRescan(graph3, queue3, s3_current, VIEWING_RANGE, k_m3)
                if s3_new == 'goal':
                    print('Goal Reached! uav3')
                    done3 = True
                else:
                    print('setting s3_current to ', s3_new)
                    s3_current = s3_new
                    pos3_coords = stateNameToCoords(s3_current)
                    # print('got pos coords: ', pos_coords)
            if done0 and done1 and done2 and done3:
                    done = True

            elif event.type == pygame.MOUSEBUTTONDOWN:
                # User clicks the mouse. Get the position
                pos = pygame.mouse.get_pos()
                # Change the x/y screen coordinates to grid coordinates
                column = pos[0] // (WIDTH + MARGIN) 
                row = pos[1] // (HEIGHT + MARGIN) 
                # Set that location to one
                if(graph0.cells[row][column] == 0):
                    graph0.cells[row][column] = -1
                if(graph1.cells[row][column] == 0):
                    graph1.cells[row][column] = -1
                if(graph2.cells[row][column] == 0):
                    graph2.cells[row][column] = -1
                if(graph3.cells[row][column] == 0):
                    graph3.cells[row][column] = -1

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
                if graph0.cells[row][column] == -1: # if there is an obstacle (we could use any graph e.g graph1, and it would be the same)
                    pygame.draw.rect(screen, colors[graph0.cells[row][column]], [(MARGIN + WIDTH) * column + MARGIN - WIDTH / 2, (MARGIN + HEIGHT) * row + MARGIN - HEIGHT / 2, WIDTH, HEIGHT])
                    node_name = 'x' + str(column) + 'y' + str(row) 
                    # print('obstacle0 at ', node_name, 'with value', graph0.cells[row][column])
                if graph0.cells[row][column] == -2 : # obstacles that uav0 has seen
                    pygame.draw.rect(screen, colors[graph0.cells[row][column]], [(MARGIN + WIDTH) * column + MARGIN - WIDTH / 2, (MARGIN + HEIGHT) * row + MARGIN - HEIGHT / 2, WIDTH, HEIGHT])
                    node_name = 'x' + str(column) + 'y' + str(row) 
                    print('obstacle0 at ', node_name, 'with value', graph0.cells[row][column])
                if graph1.cells[row][column] == -2 : # obstacles that uav1 has seen
                    pygame.draw.rect(screen, colors[graph1.cells[row][column]], [(MARGIN + WIDTH) * column + MARGIN - WIDTH / 2, (MARGIN + HEIGHT) * row + MARGIN - HEIGHT / 2, WIDTH, HEIGHT])
                    node_name = 'x' + str(column) + 'y' + str(row)
                    print('obstacle1 at ', node_name, 'with value', graph1.cells[row][column])
                if graph2.cells[row][column] == -2 : # obstacles that uav2 has seen
                    pygame.draw.rect(screen, colors[graph2.cells[row][column]], [(MARGIN + WIDTH) * column + MARGIN - WIDTH / 2, (MARGIN + HEIGHT) * row + MARGIN - HEIGHT / 2, WIDTH, HEIGHT])
                    node_name = 'x' + str(column) + 'y' + str(row)
                    print('obstacle2 at ', node_name, 'with value', graph2.cells[row][column])
                if  graph3.cells[row][column] == -2 : # obstacles that uav3 has seen
                    pygame.draw.rect(screen, colors[graph3.cells[row][column]], [(MARGIN + WIDTH) * column + MARGIN - WIDTH / 2, (MARGIN + HEIGHT) * row + MARGIN - HEIGHT / 2, WIDTH, HEIGHT])
                    node_name = 'x' + str(column) + 'y' + str(row)
                    print('obstacle3 at ', node_name, 'with value', graph3.cells[row][column])
                #    if(graph1.graph[node_name].g != float('inf')):
                #        # text = basicfont.render(
                #        # str(graph.graph[node_name].g), True, (0, 0, 200), (255,
                #        # 255, 255))
                #        text = basicfont.render(str(graph1.graph[node_name].g), True, (0, 200, 0))
                #        textrect = text.get_rect()
                #        textrect.centerx = int(column * (WIDTH + MARGIN) + WIDTH / 2) + MARGIN
                #        textrect.centery = int(row * (HEIGHT + MARGIN) + HEIGHT / 2) + MARGIN
                #        screen.blit(text, textrect)

        # fill in goal cell with GREEN
        pygame.draw.rect(screen, GREEN, [(MARGIN + WIDTH) * goal_coords[0] + MARGIN - WIDTH / 4, (MARGIN + HEIGHT) * goal_coords[1] + MARGIN - HEIGHT / 4, WIDTH / 2, HEIGHT / 2])
        # print('drawing robot pos_coords: ', pos_coords)
        # draw moving robot, based on pos_coords
        
        robot0_center = [int(pos0_coords[0] * (WIDTH + MARGIN)) +  MARGIN, int(pos0_coords[1] * (HEIGHT + MARGIN)) + MARGIN] # drone appears at the upper left corner of a square
        robot1_center = [int(pos1_coords[0] * (WIDTH + MARGIN)) +  MARGIN, int(pos1_coords[1] * (HEIGHT + MARGIN)) + MARGIN] # drone appears at the upper left corner of a square
        robot2_center = [int(pos2_coords[0] * (WIDTH + MARGIN)) +  MARGIN, int(pos2_coords[1] * (HEIGHT + MARGIN)) + MARGIN] # drone appears at the upper left corner of a square
        robot3_center = [int(pos3_coords[0] * (WIDTH + MARGIN)) +  MARGIN, int(pos3_coords[1] * (HEIGHT + MARGIN)) + MARGIN] # drone appears at the upper left corner of a square

        pygame.draw.circle(screen, RED, robot0_center, int(WIDTH / 3) - 2)
        pygame.draw.circle(screen, ORANGE, robot1_center, int(WIDTH / 3) - 2)
        pygame.draw.circle(screen, LIGHTBLUE, robot2_center, int(WIDTH / 3) - 2)
        pygame.draw.circle(screen, LIGHTPURPLE, robot3_center, int(WIDTH / 3) - 2)
        

        # draw robot viewing range
        pygame.draw.rect(screen, BLUE, [robot0_center[0] - VIEWING_RANGE * (WIDTH + MARGIN), robot0_center[1] - VIEWING_RANGE * (HEIGHT + MARGIN), 2 * VIEWING_RANGE * (WIDTH + MARGIN), 2 * VIEWING_RANGE * (HEIGHT + MARGIN)], 2)
        pygame.draw.rect(screen, BLUE, [robot1_center[0] - VIEWING_RANGE * (WIDTH + MARGIN), robot1_center[1] - VIEWING_RANGE * (HEIGHT + MARGIN), 2 * VIEWING_RANGE * (WIDTH + MARGIN), 2 * VIEWING_RANGE * (HEIGHT + MARGIN)], 2)
        pygame.draw.rect(screen, BLUE, [robot2_center[0] - VIEWING_RANGE * (WIDTH + MARGIN), robot2_center[1] - VIEWING_RANGE * (HEIGHT + MARGIN), 2 * VIEWING_RANGE * (WIDTH + MARGIN), 2 * VIEWING_RANGE * (HEIGHT + MARGIN)], 2)
        pygame.draw.rect(screen, BLUE, [robot3_center[0] - VIEWING_RANGE * (WIDTH + MARGIN), robot3_center[1] - VIEWING_RANGE * (HEIGHT + MARGIN), 2 * VIEWING_RANGE * (WIDTH + MARGIN), 2 * VIEWING_RANGE * (HEIGHT + MARGIN)], 2)

        # Limit to 60 frames per second
        clock.tick(20)

        # Go ahead and update the screen with what we've drawn.
        pygame.display.flip()

    # Be IDLE friendly. If you forget this line, the program will 'hang'
    # on exit.
    pygame.quit()
