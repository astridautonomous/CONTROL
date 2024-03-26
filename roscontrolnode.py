#!/home/ege/anaconda3/envs/deneme/bin python3

import rospy
import numpy as np
from transforms3d.euler import quat2euler
from carla_msgs.msg import CarlaEgoVehicleControl, CarlaEgoVehicleStatus
from nav_msgs.msg import Odometry
from traj import TrajectoryProcess
import numpy as np

# Create numpy array from [0,0] to [100,100] with step size 1
refposedeneme = np.array(np.meshgrid(np.arange(101), np.arange(101))).T.reshape(-1, 2)

def pose_callback(data):
    global current_pose
    global yaw
    _, _, yaw = quat2euler(
        [data.pose.pose.orientation.w,
        data.pose.pose.orientation.x,
        data.pose.pose.orientation.y,
        data.pose.pose.orientation.z])
    current_pose = np.array([data.pose.pose.position.x, data.pose.pose.position.y])
    msg = CarlaEgoVehicleControl()
    steer = TrajectoryProcess(refposedeneme).process_poses_stanley(current_pose, yaw, current_velocity)

    msg.steer = steer
    msg.throttle = 1
    pub.publish(msg)

def velocity_callback(data):
    global current_velocity
    current_velocity = data.velocity



if __name__ == "__main__":
    rospy.init_node("astrid_control")
    sub1 = rospy.Subscriber("/carla/ego_vehicle/odometry", Odometry, callback=pose_callback)
    sub2 = rospy.Subscriber("/carla/ego_vehicle/vehicle_status", CarlaEgoVehicleStatus, callback=velocity_callback)
    rospy.loginfo("Node has been started!")
    pub = rospy.Publisher("/carla/ego_vehicle/", CarlaEgoVehicleControl)

    rospy.spin()