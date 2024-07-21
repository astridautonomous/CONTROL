from maneuvers import durak_trajectory, park_trajectory, serit_degistirme
import lanelet2
import numpy as np

filename = "/home/ege/carla-ros-bridge/catkin_ws/erdals2g.osm" # ADD YOUR ".OSM" FILE
origin = lanelet2.io.Origin(0.0,0.0)
map = lanelet2.io.load(filename, origin)

maneuver_points = []
x_coord = []
y_coord = []

for point in map.pointLayer:
    local_x = point.attributes['local_x']
    x_coord.append(float(local_x))
    local_y = point.attributes['local_y']
    y_coord.append(float(local_y))
    points = [x_coord, y_coord]
    maneuver_points = np.array(points).T
print(maneuver_points)

refposedeneme = 'A* nav_ws'
kararmetre = 0


def manevra(kararmetre):

    if kararmetre == 0:
        refpose = park_trajectory(maneuver_points[4], maneuver_points[3], maneuver_points[2], maneuver_points[1], maneuver_points[0])
        print(refpose)

    elif kararmetre == 1:
        refpose = durak_trajectory(maneuver_points[8], maneuver_points[7], maneuver_points[6], maneuver_points[5], maneuver_points[4], maneuver_points[3], maneuver_points[2], maneuver_points[1], maneuver_points[0])

    elif kararmetre == 2:
        refpose = serit_degistirme(maneuver_points[4], maneuver_points[3], maneuver_points[2], maneuver_points[1], maneuver_points[0])
        
    else:
        refpose = refposedeneme

    return refpose
