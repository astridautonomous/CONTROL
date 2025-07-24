import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from carla_msgs.msg import CarlaEgoVehicleControl
from std_msgs.msg import Int8, String, Float32
from geometry_msgs.msg import PoseArray
from shapely.geometry import Point as ShapelyPoint, Polygon
import lanelet2
import lanelet2.io
import lanelet2.traffic_rules
import lanelet2.projection
import ast

class BrakeManager:
    def __init__(self, node: Node, threshold: float, brake_duration: float, wait_duration: float):
        self.node = node
        self.threshold = threshold
        self.brake_duration = brake_duration  # Frenleme süresi 
        self.wait_duration = wait_duration    # Fren sonrası bekleme süresi

        # İstasyonlar için durum değişkenleri
        self.current_stop_index = 0
        self.has_braked = False
        self.is_waiting_after_brake = False  # Fren sonrası bekleme durumunu tutar
        self.stop_start_time = None
        self.wait_start_time = None  # Bekleme süresinin başlangıç zamanı

        # Genel durum değişkenleri
        self.dynamic_mode = 0
        self.traffic_light_state = []
        self.traffic_light_status = 0  # 0: yeşil, 1: kırmızı
        self.is_stopped_at_traffic_light = False
        self.stop_start_time_traffic_light = None
        self.traffic_light_recently_released = False
        self.traffic_light_release_time = None
        self.traffic_light_release_delay = 2.0  # seconds to wait after red light turns green
        self.current_steer_cmd = 0.0
        self.is_stopped_for_dynamic = False
        self.park_point = None
        self.has_braked_at_park = False

        # Publishers
        self.control_pub = self.node.create_publisher(CarlaEgoVehicleControl, '/carla/hero/vehicle_control_cmd', 10)
        self.throttle_pub = self.node.create_publisher(Int8, '/astrid/control/throttle_cmd', 10)
        self.brake_pub = self.node.create_publisher(Int8, '/astrid/control/brake_cmd', 10)

        # Subscribers
        self.node.create_subscription(Int8, '/astrid/slam/dynamic_mode', self.dynamic_mode_callback, 10)
        self.node.create_subscription(String, '/astrid/perception/traffic_sign', self.traffic_light_callback, 10)
        self.node.create_subscription(PoseArray, '/astrid/navigation/goals', self.goal_points_callback, 10)
        self.node.create_subscription(Float32, '/astrid/control/steer_cmd', self.steering_cmd_callback, 10)
        self.node.create_subscription(Int8, '/astrid/perception/traffic_light_status', self.traffic_light_status_callback, 10)

        filename = "/home/tezcan/Downloads/simulation_fulltrack.osm"
        origin = lanelet2.io.Origin(0.0, 0.0)
        projector = lanelet2.projection.LocalCartesianProjector(origin)
        traffic_rules = lanelet2.traffic_rules.create(
            lanelet2.traffic_rules.Locations.Germany,
            lanelet2.traffic_rules.Participants.Vehicle
        )
        self.map = lanelet2.io.load(filename, projector)

        self.traffic_light_areas = []
        for area in self.map.areaLayer:
            if 'subtype' in area.attributes and area.attributes['subtype'] == 'traffic_light_area':
                coords = []
                for linestring in area.outerBound:
                    for point in linestring:
                        if 'local_x' in point.attributes and 'local_y' in point.attributes:
                            coords.append((float(point.attributes['local_x']), float(point.attributes['local_y'])))
                if len(coords) >= 3:
                    self.traffic_light_areas.append(Polygon(coords))
                    self.node.get_logger().info("traffic_light_area poligonu eklendi.")
                else:
                    self.node.get_logger().warn("Yetersiz nokta ile traffic_light_area oluşturulamadı.")
        self.node.get_logger().info(f"{len(self.traffic_light_areas)} adet traffic_light_area bulundu.")

        # Park ve durak listeleri ayrık
        self.brake_points = []
        stations = {"station_maneuver1": [], "station_maneuver2": [], "station_maneuver3": [], "park5": []}
        for line in self.map.lineStringLayer:
            if "type" in line.attributes:
                t = line.attributes["type"]
                if t in stations:
                    for point in line:
                        if 'local_x' in point.attributes and 'local_y' in point.attributes:
                            stations[t].append([float(point.attributes['local_x']), float(point.attributes['local_y'])])
        
        # İstasyon noktalarını `brake_points` listesine ekle
        for key in ["station_maneuver1", "station_maneuver2", "station_maneuver3"]:
            pts = stations[key]
            if len(pts) > 3:
                pt = pts[3]
                self.brake_points.append(pt)
                self.node.get_logger().info(f"İstasyon noktası eklendi ({key}): {pt}")
            else:
                self.node.get_logger().warn(f"{key} için yeterli nokta bulunamadı.")

        # Park noktasını `park_point` değişkenine ata
        park_pts = stations["park5"]
        if park_pts:
            self.park_point = park_pts[-1]  # Son noktayı al
            self.node.get_logger().info(f"Park noktası ayarlandı: {self.park_point}")
        else:
            self.node.get_logger().warn(f"park5 için nokta bulunamadı.")


###################################################### CALLBACKS #########################################################
    def traffic_light_callback(self, msg: String):
        try:
            data = ast.literal_eval(msg.data)
            self.traffic_light_state = data if isinstance(data, list) else []
        except:
            self.traffic_light_state = []
        # self.node.get_logger().info(f"Trafik ışığı algılama: {self.traffic_light_state}")

    def traffic_light_status_callback(self, msg: Int8):
        if self.traffic_light_status == 1 and msg.data == 0:
            self.traffic_light_recently_released = True
            self.traffic_light_release_time = self.node.get_clock().now().nanoseconds * 1e-9
            self.node.get_logger().info("Trafik ışığı kırmızıdan yeşile döndü. Kısa bir süre beklenecek.")
        self.traffic_light_status = msg.data
        # self.node.get_logger().info(f"Trafik ışığı durumu: {self.traffic_light_status}")

    def steering_cmd_callback(self, msg: Float32):
        self.current_steer_cmd = msg.data

    def goal_points_callback(self, msg: PoseArray):
        if msg.poses:
            # Gelen hedeflerin sonuncusunu al
            last_pose = msg.poses[-1]
            self.park_point = [last_pose.position.x, last_pose.position.y]
            
            # Yeni park noktasını logla ve fren bayrağını sıfırla
            self.node.get_logger().info(f"Dinamik olarak yeni park hedefi alındı: {self.park_point}")
            
            # Eğer yeni bir hedef geldiyse, önceki park freni durumunu sıfırla
            self.has_braked_at_park = False

    def dynamic_mode_callback(self, msg: Int8):
        self.dynamic_mode = msg.data
        self.node.get_logger().info(f"Dinamik mod: {self.dynamic_mode}")
        if self.dynamic_mode == 0 and self.is_stopped_for_dynamic:
            self.node.get_logger().warn("Engel kalktı. Gaz veriliyor.")
            self.send_control(throttle=0.33, brake=0.0)
            self.is_stopped_for_dynamic = False

############################################################################################################################
    def inside_traffic_light_area(self, current_x: float, current_y: float) -> bool:
        point = ShapelyPoint(current_x, current_y)
        return any(poly.contains(point) for poly in self.traffic_light_areas)

    def manage_brake(self, current_x: float, current_y: float) -> bool:
        brake_applied = False
        current_time = self.node.get_clock().now().nanoseconds * 1e-9
        
        # Trafik Işığı Kontrolü
        if self.traffic_light_status == 1:  # Kırmızı ışık
            self.node.get_logger().warn("Trafik ışığı kırmızı. Fren yapılıyor.")
            self.send_control(throttle=0.0, brake=1.0)
            self.is_stopped_at_traffic_light = True
            self.traffic_light_recently_released = False
            return True
        elif self.traffic_light_status == 0:  # Yeşil ışık
            if self.traffic_light_recently_released:
                elapsed_since_release = current_time - self.traffic_light_release_time
                if elapsed_since_release < self.traffic_light_release_delay:
                    self.node.get_logger().info(f"Trafik ışığı yeşil ama {self.traffic_light_release_delay - elapsed_since_release:.2f} saniye daha bekleniyor.")
                    self.send_control(throttle=0.0, brake=1.0)
                    self.is_stopped_at_traffic_light = True
                    return True
                else:
                    self.node.get_logger().info("Trafik ışığı beklemesi tamamlandı. İleriye gidiliyor.")
                    self.traffic_light_recently_released = False
                    self.is_stopped_at_traffic_light = False
            else:
                self.is_stopped_at_traffic_light = False
        
        # Dinamik Engel Kontrolü
        if self.dynamic_mode == 1:
            self.node.get_logger().warn("Dinamik engel algılandı. Fren yapılıyor.")
            self.send_control(throttle=0.0, brake=1.0)
            self.is_stopped_for_dynamic = True
            return True

        # Park Freni Kontrolü
        if self.park_point and not self.has_braked_at_park:
            px, py = self.park_point
            if abs(current_x - px) < self.threshold and abs(current_y - py) < self.threshold:
                self.node.get_logger().warn("!!! GÖREV TAMAMLANDI: Park noktasına ulaşıldı. Kalıcı olarak fren yapılıyor. !!!")
                self.send_control(throttle=0.0, brake=1.0)
                self.has_braked_at_park = True # Park freni yapıldığını işaretle ve bir daha bırakma
                return True # Fren komutu gönderildi, bu döngü için işlem bitti.

        # Sıralı İstasyon Durakları
        if not brake_applied and not self.is_stopped_at_traffic_light and not self.is_stopped_for_dynamic:
            if self.current_stop_index < len(self.brake_points):
                tx, ty = self.brake_points[self.current_stop_index]
                
                # Hedef noktasına yaklaşıldıysa ve henüz fren yapılmadıysa
                if (abs(current_x - tx) < self.threshold and abs(current_y - ty) < self.threshold and not self.has_braked):
                    self.node.get_logger().warn(f"{self.current_stop_index + 1}. durak noktasına ulaşıldı. {self.brake_duration} saniye fren yapılıyor.")
                    self.send_control(throttle=0.0, brake=1.0)
                    self.has_braked = True
                    self.stop_start_time = current_time
                    self.is_waiting_after_brake = False
                    brake_applied = True
                
                # Eğer frenleme aşamasındaysa
                if self.has_braked and not self.is_waiting_after_brake:
                    elapsed_brake_time = current_time - self.stop_start_time
                    if elapsed_brake_time < self.brake_duration:
                        self.send_control(throttle=0.0, brake=1.0) # Freni sürdür
                        brake_applied = True
                    else:
                        self.node.get_logger().warn(f"{self.brake_duration}s fren tamamlandı. {self.wait_duration}s bekleme moduna geçiliyor.")
                        self.send_control(throttle=0.0, brake=0.0)
                        self.is_waiting_after_brake = True
                        self.wait_start_time = current_time
                        brake_applied = True 
                
                # Eğer bekleme aşamasındaysa
                if self.is_waiting_after_brake:
                    elapsed_wait_time = current_time - self.wait_start_time
                    if elapsed_wait_time < self.wait_duration:
                        self.send_control(throttle=0.0, brake=0.0) # Boşta beklemeye devam et
                        brake_applied = True
                    else:
                        self.node.get_logger().warn(f"{self.wait_duration}s bekleme tamamlandı. Sonraki durağa devam ediliyor.")
                        self.has_braked = False
                        self.is_waiting_after_brake = False
                        self.stop_start_time = None
                        self.wait_start_time = None
                        self.current_stop_index += 1
                        # Burada gaz komutu gönderilmiyor, bir sonraki döngüye bırakılıyor.

        # Eğer hiçbir frenleme veya bekleme koşulu aktif değilse, gaz ver
        if not brake_applied and not self.is_stopped_at_traffic_light and not self.is_stopped_for_dynamic and not self.has_braked_at_park:
            self.send_control(throttle=0.33, brake=0.0)

        return brake_applied

    def send_control(self, throttle: float, brake: float):
        msg = CarlaEgoVehicleControl()
        msg.throttle = throttle
        msg.brake = brake
        msg.steer = self.current_steer_cmd
        self.control_pub.publish(msg)

        t_msg = Int8()
        b_msg = Int8()
        t_msg.data = 75 if throttle > 0 else 0
        b_msg.data = 1 if brake > 0 else 0
        self.throttle_pub.publish(t_msg)
        self.brake_pub.publish(b_msg)


class BrakeNode(Node):
    def __init__(self):
        super().__init__('brake_node')
    
        self.brake_manager = BrakeManager(self, threshold=1.0, brake_duration=3.0, wait_duration=5.0) 
        self.create_subscription(Odometry, '/carla/hero/odometry', self.odom_callback, 10)

    def odom_callback(self, msg: Odometry):
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        self.brake_manager.manage_brake(x, y)


def main(args=None):
    rclpy.init(args=args)
    node = BrakeNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()