#!/usr/bin/env python3

import rospy
import numpy as np
import lanelet2
import sys
from transforms3d.euler import quat2euler
from carla_msgs.msg import CarlaEgoVehicleControl, CarlaEgoVehicleStatus
from nav_msgs.msg import Odometry
from traj import TrajectoryProcess
from maneuvers import durak_trajectory, park_trajectory, serit_degistirme

# sys.path.append("/home/ege/astrid/NAVIGATION/nav_ws/src")
# from navigation.msg import Path
from st_pid.msg import Path

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

kararmetre = None # 0: Park, 1: Durak, 2: Serit Degistirme , else: refpose

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


def mapToImg(coord):
    return [coord[1]+190,coord[0]+162]
def imgToMap(coord):
    return [coord[1]-162,coord[0]-190]

current_velocity=0

def velocity_callback(data):
    global current_velocity
    current_velocity = data.velocity

def refpose_callback(data):
    pat = []
    global refposedeneme
    if len(data.path_) > 0:
        for i in range(0,len(data.path_)-2,2):
            pat.append([data.path_[i],data.path_[i+1]])
        refposedeneme = np.array(pat)
    # print("ref pose",refposedeneme)

def pose_callback(data):
    global current_pose
    global yaw  #rad
    global vehicle
    # print(data)
    _, _, yaw = quat2euler(
        [data.pose.pose.orientation.w,
        data.pose.pose.orientation.x,
        data.pose.pose.orientation.y,
        data.pose.pose.orientation.z])
    current_pose = np.array([data.pose.pose.position.x, data.pose.pose.position.y])
    # print("data.pose.pose.position.x",data.pose.pose.position.x)
    # print("data.pose.pose.orientation.x",data.pose.pose.orientation.x)
    msg = CarlaEgoVehicleControl()
    print("current pose",current_pose)
    steercmd = vehicle.process_poses_stanley(current_pose, yaw, current_velocity)

    v_error = 1.38 - current_velocity ## m/s
    brakecmd= vehicle.pidbrake(v_error,0.1)
    throttlecmd = vehicle.pidthrottle(v_error,0.1)
    
    msg.brake = brakecmd
    msg.throttle = throttlecmd
    msg.steer = steercmd
    print("steercmd",msg.steer)
    print("min_distance_idx",vehicle.min_distance_index)
    pub.publish(msg)

if __name__ == "__main__":
    rospy.init_node("astrid_control")
    rate = rospy.Rate(1)
    refposedeneme = []
    onceki_kararmetre = None

    while not rospy.is_shutdown():
        sub3 = rospy.Subscriber("/path_planner", Path, callback=refpose_callback)
        rate.sleep()
        if len(refposedeneme) >= 0:
            break
    refposedeneme = manevra(kararmetre)
    vehicle = TrajectoryProcess(refposedeneme)
    
    while(rospy.is_shutdown != True):
        sub1 = rospy.Subscriber("/carla/ego_vehicle/odometry", Odometry, callback=pose_callback)
        sub2 = rospy.Subscriber("/carla/ego_vehicle/vehicle_status", CarlaEgoVehicleStatus, callback=velocity_callback)
        rospy.loginfo("Node has been started!")
        print(CarlaEgoVehicleControl)
        pub = rospy.Publisher("/carla/ego_vehicle/vehicle_control_cmd", CarlaEgoVehicleControl, queue_size=10)

        if kararmetre != onceki_kararmetre:
            refpose = manevra(kararmetre)
            vehicle = TrajectoryProcess(refpose)
            onceki_kararmetre = kararmetre
        rate.sleep()