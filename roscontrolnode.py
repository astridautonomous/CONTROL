#!/home/ege/anaconda3/envs/rosbridge-carla/bin python3

import rospy
import numpy as np
from transforms3d.euler import quat2euler
from carla_msgs.msg import CarlaEgoVehicleControl, CarlaEgoVehicleStatus
from nav_msgs.msg import Odometry
from traj import TrajectoryProcess
import numpy as np

#example trajectory
start_point = np.array([-74, -143])
end_point = np.array([-80, -70])

refposedeneme = np.linspace(start_point, end_point, 300)
current_velocity=0

def velocity_callback(data):
    global current_velocity
    current_velocity = data.velocity


def pose_callback(data):
    global current_pose
    global yaw  #rad
    _, _, yaw = quat2euler(
        [data.pose.pose.orientation.w,
        data.pose.pose.orientation.x,
        data.pose.pose.orientation.y,
        data.pose.pose.orientation.z])
    current_pose = np.array([data.pose.pose.position.x, data.pose.pose.position.y])
    msg = CarlaEgoVehicleControl()
    steercmd = TrajectoryProcess(refposedeneme).process_poses_stanley(current_pose, yaw, current_velocity)

    v_error = 2.77 - current_velocity ## m/s
    brakecmd=TrajectoryProcess(refposedeneme).pidbrake(v_error,0.1)
    throttlecmd = TrajectoryProcess(refposedeneme).pidthrottle(v_error,0.1)
    
    msg.brake = brakecmd
    msg.throttle = throttlecmd
    msg.steer = steercmd
    pub.publish(msg)


if __name__ == "__main__":
    rospy.init_node("astrid_control")
    sub1 = rospy.Subscriber("/carla/ego_vehicle/odometry", Odometry, callback=pose_callback)
    sub2 = rospy.Subscriber("/carla/ego_vehicle/vehicle_status", CarlaEgoVehicleStatus, callback=velocity_callback)
    rospy.loginfo("Node has been started!")
    pub = rospy.Publisher("/carla/ego_vehicle/vehicle_control_cmd", CarlaEgoVehicleControl, queue_size=10)

    rospy.spin()
