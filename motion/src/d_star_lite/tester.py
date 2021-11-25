""" pos_coords = []
two_last_waypoints = []

for i in range(4):
    pos_coords.append([i,i+2])
    two_last_waypoints.append([pos_coords[i], pos_coords[i]])

print(two_last_waypoints[2][0])

for ID in range(4):
    pos_coords[ID] = [ID,ID+1]
    two_last_waypoints[ID] = [pos_coords[ID], two_last_waypoints[ID][0]] ####

print(two_last_waypoints) """

a = [[1,2],3]