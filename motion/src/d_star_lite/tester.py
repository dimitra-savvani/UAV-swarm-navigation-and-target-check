""" from random import randint
import math


center = [10.0, 20]
subarea_length = 20
subarea_width = 40.3

new_target_x = randint(center[0] - math.floor(subarea_length/2), center[0] + math.floor(subarea_length/2))
new_target_y = randint(center[1] - math.floor(subarea_width/2), center[1] + math.floor(subarea_width/2))

new_target = [new_target_x, new_target_y]
print(new_target) """

""" def stateNameToCoords(name):
    return [int(name.split('x')[1].split('y')[0]), int(name.split('x')[1].split('y')[1])]
print(stateNameToCoords("x20y31")) """

while True:
    print("once")
    break