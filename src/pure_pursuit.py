import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry, Path
import math
from std_msgs.msg import Int16


class PurePursuitController(Node):
    def __init__(self):
        super().__init__('pure_pursuit_controller') 
        
        # Params
        self.L = 1.7 # Wheelbase
        self.lookahead_distance = 3.289
      
        self.final_waypoint_brake_threshold = 0.5 #### Calibrative Variable
        
        # Subscribers
        self.odom_sub = self.create_subscription(Odometry, '/clap/ros/odometry', self.odom_callback, 10)
        self.path_sub = self.create_subscription(Path, '/astrid/navigation/fusion_path', self.path_callback, 10)
        
        # Publishers
        self.steer_pub = self.create_publisher(Int16, '/astrid/control/steer_cmd', 10) 
        self.brake_pub = self.create_publisher(Int16, '/astrid/control/brake_cmd', 10) 
        

        # Internal state
        self.current_pose = None
        self.current_orientation = None
        self.waypoints = []
        self.current_waypoint_index = 0

    def odom_callback(self, msg):
        self.current_pose = msg.pose.pose
        self.current_orientation = msg.pose.pose.orientation
        self.current_orientation.x = msg.pose.pose.orientation.x
        self.current_orientation.y = msg.pose.pose.orientation.y
        self.current_orientation.z = msg.pose.pose.orientation.z
        self.current_orientation.w = msg.pose.pose.orientation.w

        if self.current_pose and self.waypoints:
            self.compute_and_publish_control()

    def path_callback(self, msg):
        self.waypoints = [pose.pose.position for pose in msg.poses]
        self.current_waypoint_index = 0
        #self.get_logger().info(f'{len(self.waypoints)} waypoint alındı.')

    def get_yaw_from_quaternion(self, w_o, x_o, y_o, z_o,yaw=None):
        w_o = self.current_orientation.w
        x_o = self.current_orientation.x
        y_o = self.current_orientation.y
        z_o = self.current_orientation.z
        #self.get_logger().info(f'Quaternion: ({w_o}, {x_o}, {y_o}, {z_o})')
        t3 = +2.0 * (w_o * z_o + x_o * y_o)
        t4 = +1.0 - 2.0 * (y_o * y_o + z_o * z_o)
        yaw = math.atan2(t3, t4)
        return yaw

    def compute_and_publish_control(self):
        if not self.waypoints:
           return

        x = self.current_pose.position.x
        y = self.current_pose.position.y

        closest_dist = float('inf')
        closest_idx = 0
        for i in range(len(self.waypoints)):
            wp = self.waypoints[i]
            dist = math.sqrt((wp.x - x)**2 + (wp.y - y)**2)
            if dist < closest_dist:
                closest_dist = dist
                closest_idx = i

        target_idx = closest_idx
        for i in range(closest_idx, len(self.waypoints)):
            wp = self.waypoints[i]
            dist = math.sqrt((wp.x - x)**2 + (wp.y - y)**2)
            if dist >= self.lookahead_distance:
                target_idx = i
                break

        if target_idx == closest_idx:
            target_idx = len(self.waypoints) - 1
            self.get_logger().info("Lookahead distance aşılamadı, son waypoint hedef olarak seçildi.") 
            
        xt = self.waypoints[target_idx].x
        yt = self.waypoints[target_idx].y
        distance_to_target = math.sqrt((xt - x)**2 + (yt - y)**2)

        
        self.get_logger().info(
        f"\n- Mevcut Pozisyon: ({x:.2f}, {y:.2f})"
        f"\n- En Yakın Waypoint [Index {closest_idx}]: ({self.waypoints[closest_idx].x:.2f}, {self.waypoints[closest_idx].y:.2f})"
        f"\n- Seçilen Target [Index {target_idx}]: ({xt:.2f}, {yt:.2f})"
        f"\n- Target Mesafesi: {distance_to_target:.2f} m (Lookahead: {self.lookahead_distance:.2f} m)"
        f"\n- Kalan Waypoint Sayısı: {len(self.waypoints) - target_idx}"
        )
    
        w_o = self.current_orientation.w
        x_o = self.current_orientation.x
        y_o = self.current_orientation.y
        z_o = self.current_orientation.z
        self.get_logger().info(f'Quaternion: ({w_o}, {x_o}, {y_o}, {z_o})') 
        t3 = +2.0 * (w_o * z_o + x_o * y_o)
        t4 = +1.0 - 2.0 * (y_o * y_o + z_o * z_o)
        yaw = -math.atan2(t3, t4)-math.radians(110)
        if yaw < 0:
            yaw = yaw + math.radians(360)

        #if yaw<0:
        #    yaw=yaw+math.radians(360)
        #yaw = math.radians(360)-yaw
        yaw = math.atan2(math.sin(yaw), math.cos(yaw)) # Normalize angle to [-pi, pi]

        dx = xt - x
        dy = yt - y
        #alpha = alpha_map - ALPHA_VEHGİCLE 
        alpha = math.atan2(dx, dy) - yaw 
        alpha = math.atan2(math.sin(alpha), math.cos(alpha)) - math.radians(29.5) # Normalize angle to [-pi, pi]

        #alpha = math.atan2(math.sin(alpha), math.cos(alpha)) # Tekrar normalize etme yorum satırına alındı.

        delta = math.atan2(2.0 * self.L * math.sin(alpha), self.lookahead_distance)
        steer_cmd = math.degrees(delta)#max(min(delta, 25.5), -25.5)
        if math.degrees(delta) < -20.0:
            steer_cmd = -20.0
        elif math.degrees(delta) > 20.0:
            steer_cmd = 20.0
        #if -179 > yaw > -180:
        #    steer_cmd = -steer_cmd
        #if 165 < yaw < 180:
        #    steer_cmd = -steer_cmd
        if alpha > 0:
            steer_cmd = -steer_cmd
        else:
            steer_cmd = abs(steer_cmd)

        self.get_logger().warn(
        f"Lookahead: {self.lookahead_distance:.2f} m | "
        f"Yaw: {math.degrees(yaw):.2f}° | "
        f"Alpha: {math.degrees(alpha):.2f}° | "
        f"Delta: {math.degrees(delta):.2f}° | "
        f"Steer: {steer_cmd:.2f} | "
        )

        # Sadece steering komutunu Int16 olarak yayınla
        steer_msg = Int16()
        steer = int(10 * steer_cmd)
        steer_msg.data = steer
        self.steer_pub.publish(steer_msg)
        
        brake_value = 2
        
        # Yolun son noktasına olan mesafeyi kontrol et
        if self.waypoints:
            final_waypoint = self.waypoints[-1]
            dist_to_final = math.sqrt((final_waypoint.x - x)**2 + (final_waypoint.y - y)**2)
            
            if dist_to_final < self.final_waypoint_brake_threshold:
                brake_value = 1 # Aracı durdur
                self.get_logger().info(f"GOAL WAYPOINTE YAKLAŞILDI (Mesafe: {dist_to_final:.2f}m). FREN UYGULANIYOR.")
        
        brake_msg = Int16()
        brake_msg.data = brake_value
        self.brake_pub.publish(brake_msg)
        
def main(args=None):
    rclpy.init(args=args)
    node = PurePursuitController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
