import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry, Path
import math
# from carla_msgs.msg import CarlaEgoVehicleControl  # BrakeManager üstlendi
from std_msgs.msg import Int16
from visualization_msgs.msg import Marker, MarkerArray  # YENİ EKLENTİ
from geometry_msgs.msg import Point  # YENİ EKLENTİ

class PurePursuitController(Node):
    def __init__(self):
        super().__init__('pure_pursuit_controller')
        # Params
        self.L = 1.7                              # Carla ego vehicle wheelbase (m)
        self.lookahead_distance = 2.85     #### Calibrative Variable
        # self.final_waypoint_brake_threshold = 2.0  # BrakeManager üstlendi
        # self.target_throttle = 0.3                 # BrakeManager üstlendi

        # Subscribers
        self.odom_sub = self.create_subscription(Odometry, '/astrid/odometry_local', self.odom_callback, 10)
        self.path_sub = self.create_subscription(Path, '/astrid/navigation/fusion_path', self.path_callback, 10)

        # Publishers
        self.steer_pub = self.create_publisher(Int16, '/astrid/control/steer_cmd', 10)
        # self.control_pub = self.create_publisher(CarlaEgoVehicleControl, '/carla/hero/vehicle_control_cmd', 10)  # BrakeManager üstlendi
        
        # YENİ: Hedef waypoint için marker publisher
        self.target_marker_pub = self.create_publisher(Marker, '/astrid/control/target_waypoint_marker', 10)

        # Internal state
        self.current_pose = None
        self.current_orientation = None
        self.waypoints = []
        # self.goal_reached = False  # BrakeManager üstlendi

    def odom_callback(self, msg):
        self.current_pose = msg.pose.pose
        self.current_orientation = msg.pose.pose.orientation
        if self.current_pose and self.waypoints:  # goal_reached kontrolü BrakeManager'da
            self.compute_and_publish_control()

    def path_callback(self, msg):
        self.waypoints = [pose.pose.position for pose in msg.poses]
        # self.goal_reached = False  # BrakeManager üstlendi
        #self.get_logger().info(f'{len(self.waypoints)} waypoint alındı.')

    def get_yaw_from_quaternion(self):
        w = self.current_orientation.w
        x = self.current_orientation.x
        y = self.current_orientation.y
        z = self.current_orientation.z
        t3 = 2.0 * (w * z + x * y)
        t4 = 1.0 - 2.0 * (y * y + z * z)
        return math.atan2(t3, t4)

    # YENİ: Hedef waypoint'i görselleştirmek için yardımcı fonksiyon
    def publish_target_marker(self, target_idx, x, y):
        marker = Marker()
        marker.header.frame_id = "map"  # veya "odom" - kullandığınız frame'e göre ayarlayın
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "target_waypoint"
        marker.id = 0
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        
        # Pozisyon
        marker.pose.position.x = x
        marker.pose.position.y = y
        marker.pose.position.z = 0.0
        
        # Boyut
        marker.scale.x = 0.5  # Yarıçap
        marker.scale.y = 0.5
        marker.scale.z = 0.5
        
        # Renk (kırmızı)
        marker.color.r = 1.0
        marker.color.g = 0.0
        marker.color.b = 0.0
        marker.color.a = 1.0
        
        # Ömür (sürekli görünmesi için 0)
        marker.lifetime.sec = 0
        
        self.target_marker_pub.publish(marker)

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

        # YENİ: Hedef waypoint'i yayınla
        self.publish_target_marker(target_idx, xt, yt)

        # Yaw
        yaw = self.get_yaw_from_quaternion()# + math.radians(170.0) #+math.radians(110)

        # Alpha
        dx = xt - x
        dy = yt - y
        ref_yaw = math.atan2(dy, dx)
        alpha =ref_yaw - yaw
        alpha = math.atan2(math.sin(alpha), math.cos(alpha))  # Normalize [-pi, pi]

        # Pure Pursuit direksiyon açısı
        delta = math.atan2(2.0 * self.L * math.sin(alpha), self.lookahead_distance)

        steer_cmd = math.degrees(delta)
        if math.degrees(delta) < -25.5:
            steer_cmd = -25.5
        elif math.degrees(delta) > 25.5:
            steer_cmd = 25.5

        if alpha > 0:
            steer_cmd = -steer_cmd 
        else:
            steer_cmd = abs(steer_cmd)

        self.get_logger().info (
            f"\n- Position: ({x:.2f}, {y:.2f})"
            f"\n- CLosest Waypoint [{closest_idx}]: ({self.waypoints[closest_idx].x:.2f}, {self.waypoints[closest_idx].y:.2f}) "
            f"\n- Target [{target_idx}]: ({xt:.2f}, {yt:.2f})"
            #f"\n- Target Mesafesi: {distance_to_target:.2f} m"
            #f"\n- Kalan Waypoint: {len(self.waypoints) - target_idx}"
            
        )   
        self.get_logger().warn(
            f"\n- Yaw: {math.degrees(yaw):.2f}°| "
            f"\n- Ref Yaw {math.degrees(ref_yaw):.2f}°|"
            f"\n- Alpha: {math.degrees(alpha):.2f}° | "
            f"\n- Delta: {math.degrees(delta):.2f}° | "
            #f"Steer: {steer_cmd:.3f}"
        )

        # Sadece steer yayınla — throttle/brake kararı BrakeManager'a ait
        steer_msg = Int16()
        steer_msg.data =int(steer_cmd) * 10
        self.steer_pub.publish(steer_msg)

        #── Aşağıdaki blok BrakeManager'a devredildi ──────────────────────
        #control_msg = CarlaEgoVehicleControl()
        #final_waypoint = self.waypoints[-1]
        #dist_to_final = math.hypot(final_waypoint.x - x, final_waypoint.y - y)
        #if dist_to_final < self.final_waypoint_brake_threshold:
        #    self.goal_reached = True
        #    control_msg.throttle = 0.0
        #     control_msg.brake = 1.0
        #    control_msg.steer = 0.0
        #    self.get_logger().info(
        #        f"GOAL REACHED (Mesafe: {dist_to_final:.2f}m). ARAÇ DURDURULUYOR.")
        #else:
        #    control_msg.throttle = self.target_throttle
        #    control_msg.brake = 0.0
        #    control_msg.steer = steer_cmd
        #control_msg.hand_brake = False
        #control_msg.reverse = False
        #control_msg.manual_gear_shift = False
        #self.control_pub.publish(control_msg)
    

def main(args=None):
    rclpy.init(args=args)
    node = PurePursuitController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
