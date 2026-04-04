import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry, Path
import math
from carla_msgs.msg import CarlaEgoVehicleControl
from std_msgs.msg import Float32

class PurePursuitController(Node):
    def __init__(self):
        super().__init__('pure_pursuit_controller')

        # Params
        self.L = 4.0                              # Carla ego vehicle wheelbase (m)
        self.lookahead_distance = 2.0             #### Calibrative Variable
        self.final_waypoint_brake_threshold = 2.0 #### Calibrative Variable
        self.target_throttle = 0.3                # Throttle (0.0 - 1.0)

        # Subscribers
        self.odom_sub = self.create_subscription(Odometry, '/carla/hero/odometry', self.odom_callback, 10)
        self.path_sub = self.create_subscription(Path, '/astrid/navigation/global_path', self.path_callback, 10)

        # Publishers
        self.steer_pub = self.create_publisher(Float32, '/astrid/control/steer_cmd', 10)
        self.control_pub = self.create_publisher(CarlaEgoVehicleControl,'/carla/hero/vehicle_control_cmd', 10)

        # Internal state
        self.current_pose = None
        self.current_orientation = None
        self.waypoints = []
        self.goal_reached = False

    def odom_callback(self, msg):
        self.current_pose = msg.pose.pose
        self.current_orientation = msg.pose.pose.orientation
        if self.current_pose and self.waypoints and not self.goal_reached:
            self.compute_and_publish_control()

    def path_callback(self, msg):
        self.waypoints = [pose.pose.position for pose in msg.poses]
        self.goal_reached = False
        self.get_logger().info(f'{len(self.waypoints)} waypoint alındı.')

    def get_yaw_from_quaternion(self):
        w = self.current_orientation.w
        x = self.current_orientation.x
        y = self.current_orientation.y
        z = self.current_orientation.z
        t3 = 2.0 * (w * z + x * y)
        t4 = 1.0 - 2.0 * (y * y + z * z)
        return math.atan2(t3, t4)

    def compute_and_publish_control(self):
        if not self.waypoints:
            return

        x = self.current_pose.position.x
        y = self.current_pose.position.y

        # En yakın waypoint
        closest_idx = min(
            range(len(self.waypoints)),
            key=lambda i: math.hypot(self.waypoints[i].x - x, self.waypoints[i].y - y)
        )

        # Lookahead waypoint
        target_idx = closest_idx
        for i in range(closest_idx, len(self.waypoints)):
            wp = self.waypoints[i]
            dist = math.hypot(wp.x - x, wp.y - y)
            if dist >= self.lookahead_distance:
                target_idx = i
                break

        if target_idx == closest_idx:
            target_idx = len(self.waypoints) - 1
            self.get_logger().info(
                "Lookahead distance aşılamadı, son waypoint hedef olarak seçildi.")

        xt = self.waypoints[target_idx].x
        yt = self.waypoints[target_idx].y
        distance_to_target = math.hypot(xt - x, yt - y)

        # Yaw
        yaw = self.get_yaw_from_quaternion()

        # Alpha
        dx = xt - x
        dy = yt - y
        alpha = math.atan2(dy, dx) - yaw
        alpha = math.atan2(math.sin(alpha), math.cos(alpha))  # Normalize [-pi, pi]

        # Pure Pursuit direksiyon açısı
        delta = math.atan2(2.0 * self.L * math.sin(alpha), self.lookahead_distance)

        # ← İLK KODDAN ALINAN ÇALIŞAN STEERİNG MANTIĞI
        steer_cmd = max(min(delta, 1.0), -1.0)
        if alpha > 0:
            steer_cmd = -steer_cmd
        else:
            steer_cmd = abs(steer_cmd)

        self.get_logger().info(
            f"\n- Pozisyon: ({x:.2f}, {y:.2f})"
            f"\n- En Yakın Waypoint [{closest_idx}]: "
            f"({self.waypoints[closest_idx].x:.2f}, {self.waypoints[closest_idx].y:.2f})"
            f"\n- Target [{target_idx}]: ({xt:.2f}, {yt:.2f})"
            f"\n- Target Mesafesi: {distance_to_target:.2f} m"
            f"\n- Kalan Waypoint: {len(self.waypoints) - target_idx}"
        )
        self.get_logger().warn(
            f"Yaw: {math.degrees(yaw):.2f}° | "
            f"Alpha: {math.degrees(alpha):.2f}° | "
            f"Delta: {math.degrees(delta):.2f}° | "
            f"Steer: {steer_cmd:.3f}"
        )

        # Float32 steer yayınla
        steer_msg = Float32()
        steer_msg.data = steer_cmd
        self.steer_pub.publish(steer_msg)

        # CarlaEgoVehicleControl yayınla
        control_msg = CarlaEgoVehicleControl()
        final_waypoint = self.waypoints[-1]
        dist_to_final = math.hypot(final_waypoint.x - x, final_waypoint.y - y)

        if dist_to_final < self.final_waypoint_brake_threshold:
            self.goal_reached = True
            control_msg.throttle = 0.0
            control_msg.brake = 1.0
            control_msg.steer = 0.0
            self.get_logger().info(
                f"GOAL REACHED (Mesafe: {dist_to_final:.2f}m). ARAÇ DURDURULUYOR.")
        else:
            control_msg.throttle = self.target_throttle
            control_msg.brake = 0.0
            control_msg.steer = steer_cmd

        control_msg.hand_brake = False
        control_msg.reverse = False
        control_msg.manual_gear_shift = False
        self.control_pub.publish(control_msg)


def main(args=None):
    rclpy.init(args=args)
    node = PurePursuitController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
