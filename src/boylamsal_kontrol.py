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

THROTTLE_VALUE = 0.33
THROTTLE_INT   = 75

# ── Fren süresi sabitleri (saniye) ───────────────────────────────────────────
TL_BRAKE_DURATION      = 3.0   # Trafik ışığı kırmızıda fren süresi
DYNAMIC_BRAKE_DURATION = 3.0   # Dinamik engelde fren süresi


class BrakeManager:
    def __init__(self, node: Node, dist_threshold: float,
                 brake_duration: float, wait_duration: float):

        self.node              = node
        self.dist_threshold    = dist_threshold
        self.dist_threshold_sq = dist_threshold * dist_threshold
        self.brake_duration    = brake_duration
        self.wait_duration     = wait_duration

        self.current_stop_index      = 0
        self.has_braked              = False
        self.is_waiting_after_brake  = False
        self.stop_start_time         = None
        self.wait_start_time         = None

        # ── Dinamik engel ────────────────────────────────────────────────────
        self.dynamic_mode             = 0
        self.is_stopped_for_dynamic   = False
        self.dynamic_brake_start_time = None   # ← YENİ: fren başlangıç zamanı

        # ── Trafik ışığı ─────────────────────────────────────────────────────
        self.traffic_light_status            = 0
        self.is_stopped_at_traffic_light     = False
        self.traffic_light_brake_start_time  = None   # ← YENİ
        self.traffic_light_recently_released = False
        self.traffic_light_release_time      = None
        self.traffic_light_release_delay     = 2.0

        self.current_steer_cmd  = 0.0
        self.park_point         = None
        self.has_braked_at_park = False

        # ── Publishers ───────────────────────────────────────────────────────
        self.control_pub  = self.node.create_publisher(
            CarlaEgoVehicleControl, '/carla/hero/vehicle_control_cmd', 10)
        self.throttle_pub = self.node.create_publisher(
            Int8, '/astrid/control/throttle_cmd', 10)
        self.brake_pub    = self.node.create_publisher(
            Int8, '/astrid/control/brake_cmd', 10)

        # ── Subscribers ──────────────────────────────────────────────────────
        self.node.create_subscription(
            Int8, '/astrid/slam/dynamic_mode',
            self.dynamic_mode_callback, 10)
        self.node.create_subscription(
            String, '/astrid/perception/traffic_sign',
            self.traffic_light_callback, 10)
        self.node.create_subscription(
            PoseArray, '/astrid/navigation/goals',
            self.goal_points_callback, 10)
        self.node.create_subscription(
            Float32, '/astrid/control/steer_cmd',
            self.steering_cmd_callback, 10)

        # ── Map Loading ──────────────────────────────────────────────────────
        filename  = "/home/talha/testler/test_yanal/CarlaRobotaxiTrack.osm"
        origin    = lanelet2.io.Origin(0.0, 0.0)
        projector = lanelet2.projection.LocalCartesianProjector(origin)
        lanelet2.traffic_rules.create(
            lanelet2.traffic_rules.Locations.Germany,
            lanelet2.traffic_rules.Participants.Vehicle,
        )
        self.map = lanelet2.io.load(filename, projector)

        # ── Traffic Light Areas ──────────────────────────────────────────────
        self.traffic_light_areas = []
        for area in self.map.areaLayer:
            if 'subtype' not in area.attributes or \
               area.attributes['subtype'] != 'traffic_light_area':
                continue
            coords = [
                (float(pt.attributes['local_x']), float(pt.attributes['local_y']))
                for ls in area.outerBound
                    for pt in ls
                        if 'local_x' in pt.attributes and 'local_y' in pt.attributes
            ]
            if len(coords) >= 3:
                self.traffic_light_areas.append(Polygon(coords))
                self.node.get_logger().info("traffic_light_area poligonu eklendi.")
            else:
                self.node.get_logger().warn(
                    "Yetersiz nokta ile traffic_light_area oluşturulamadı.")
        self.node.get_logger().info(
            f"{len(self.traffic_light_areas)} adet traffic_light_area bulundu.")

        # ── Brake / Park Points ──────────────────────────────────────────────
        self.brake_points = []
        stations = {
            "durak1": [],
            "durak2": [],
            "durak3": [],
            "park6":             [],
        }
        for line in self.map.lineStringLayer:
            if 'type' not in line.attributes:
                continue
            t = line.attributes['type']
            if t in stations:
                for point in line:
                    if 'local_x' in point.attributes and 'local_y' in point.attributes:
                        stations[t].append(
                            [float(point.attributes['local_x']),
                             float(point.attributes['local_y'])]
                        )

        for key in ["durak1", "durak2", "durak3"]:
            pts = stations[key]
            if len(pts) > 3:
                pt = pts[3]
                self.brake_points.append(pt)
                self.node.get_logger().info(
                    f"İstasyon noktası eklendi ({key}): {pt}")
            else:
                self.node.get_logger().warn(
                    f"{key} için yeterli nokta bulunamadı.")

        park_pts = stations["park6"]
        if park_pts:
            self.park_point = park_pts[-1]
            self.node.get_logger().info(
                f"Park noktası ayarlandı: {self.park_point}")
        else:
            self.node.get_logger().warn("park6 için nokta bulunamadı.")

    # ════════════════════════════════════════════════════════════════════════
    # CALLBACKS
    # ════════════════════════════════════════════════════════════════════════

    def traffic_light_callback(self, msg: String) -> None:
        raw = msg.data.strip()
        if raw == 'yesil_isik':
            if self.traffic_light_status == 1:
                self.traffic_light_recently_released = True
                self.traffic_light_release_time = (
                    self.node.get_clock().now().nanoseconds * 1e-9
                )
                self.node.get_logger().info(
                    f"Trafik ışığı yeşile döndü. "
                    f"{self.traffic_light_release_delay}s bekleme başlatıldı."
                )
                self._apply_brake()
            else:
                self._apply_throttle()
            self.traffic_light_status           = 0
            # Kırmızı fren timer'ını sıfırla — artık yeşil
            self.traffic_light_brake_start_time = None
            self.node.get_logger().info("traffic_light_status = 0 (yeşil)")

        elif raw == 'kirmizi_isik':
            # Yeni kırmızı sinyali → timer'ı sadece ilk gelişte başlat
            if self.traffic_light_status != 1:
                self.traffic_light_brake_start_time = (
                    self.node.get_clock().now().nanoseconds * 1e-9
                )
                self.node.get_logger().warn(
                    f"Kırmızı ışık — {TL_BRAKE_DURATION}s fren başlatıldı.")
            self.traffic_light_status            = 1
            self.traffic_light_recently_released = False
            self._apply_brake()
        else:
            self.node.get_logger().warn(
                f"Tanınmayan trafik ışığı mesajı: '{raw}'")

    def steering_cmd_callback(self, msg: Float32) -> None:
        self.current_steer_cmd = msg.data

    def goal_points_callback(self, msg: PoseArray) -> None:
        if msg.poses:
            last_pose = msg.poses[-1]
            self.park_point = [last_pose.position.x, last_pose.position.y]
            self.node.get_logger().info(
                f"Yeni park hedefi: {self.park_point}")
            self.has_braked_at_park = False

    def dynamic_mode_callback(self, msg: Int8) -> None:
        prev = self.dynamic_mode
        self.dynamic_mode = msg.data
        self.node.get_logger().info(f"Dinamik mod: {self.dynamic_mode}")

        if self.dynamic_mode == 1 and prev != 1:
            # Yeni engel → timer başlat
            self.dynamic_brake_start_time = (
                self.node.get_clock().now().nanoseconds * 1e-9
            )
            self.node.get_logger().warn(
                f"Dinamik engel algılandı — {DYNAMIC_BRAKE_DURATION}s fren başlatıldı.")

        if self.dynamic_mode == 0 and self.is_stopped_for_dynamic:
            self.node.get_logger().warn("Engel kalktı. Gaz veriliyor.")
            self._apply_throttle()
            self.is_stopped_for_dynamic   = False
            self.dynamic_brake_start_time = None

    # ════════════════════════════════════════════════════════════════════════
    # LOW-LEVEL SEND HELPERS
    # ════════════════════════════════════════════════════════════════════════

    def _send_control(self, throttle: float, brake: float) -> None:
        ctrl          = CarlaEgoVehicleControl()
        ctrl.throttle = float(throttle)
        ctrl.brake    = float(brake)
        ctrl.steer    = self.current_steer_cmd
        self.control_pub.publish(ctrl)

        t_msg      = Int8()
        b_msg      = Int8()
        t_msg.data = THROTTLE_INT if throttle > 0.0 else 0
        b_msg.data = 1            if brake    > 0.0 else 0
        self.throttle_pub.publish(t_msg)
        self.brake_pub.publish(b_msg)

    def _apply_brake(self)    -> None: self._send_control(throttle=0.0,           brake=1.0)
    def _apply_throttle(self) -> None: self._send_control(throttle=THROTTLE_VALUE, brake=0.0)
    def _apply_coast(self)    -> None: self._send_control(throttle=0.0,           brake=0.0)

    # ════════════════════════════════════════════════════════════════════════
    # HELPERS
    # ════════════════════════════════════════════════════════════════════════

    def _within_threshold(self, cx, cy, tx, ty) -> bool:
        dx, dy = cx - tx, cy - ty
        return (dx * dx + dy * dy) < self.dist_threshold_sq

    def inside_traffic_light_area(self, x: float, y: float) -> bool:
        point = ShapelyPoint(x, y)
        return any(poly.contains(point) for poly in self.traffic_light_areas)

    # ════════════════════════════════════════════════════════════════════════
    # PRIORITY CHECKS
    # ════════════════════════════════════════════════════════════════════════

    def _check_traffic_light(self, now: float) -> bool:
        """
        Öncelik 1: Kırmızı ışıkta sabit TL_BRAKE_DURATION saniye fren bas,
        sonra coast'a geç. Yeşile geçince normal akışa dön.
        """
        if self.traffic_light_status == 1:
            elapsed = now - (self.traffic_light_brake_start_time or now)

            if elapsed < TL_BRAKE_DURATION:
                # Fren fazı
                self.is_stopped_at_traffic_light = True
                self._apply_brake()
                self.node.get_logger().warn(
                    f"Trafik ışığı kırmızı — fren {elapsed:.1f}/{TL_BRAKE_DURATION}s")
            else:
                # Süre doldu → coast, bir daha basma
                if self.is_stopped_at_traffic_light:
                    self.node.get_logger().info(
                        "Trafik ışığı fren süresi doldu — coast moduna geçildi.")
                    self.is_stopped_at_traffic_light = False
                self._apply_coast()
            return True

        # Yeşile geçiş sonrası kısa bekleme
        if self.traffic_light_recently_released:
            remaining = self.traffic_light_release_delay - (
                now - self.traffic_light_release_time)
            if remaining > 0:
                self.node.get_logger().info(
                    f"Yeşil ama {remaining:.2f}s bekleniyor.")
                self._apply_brake()
                return True
            self.node.get_logger().info("Bekleme tamamlandı — ileriye gidiliyor.")
            self.traffic_light_recently_released = False

        self.is_stopped_at_traffic_light = False
        return False

    def _check_dynamic(self, now: float) -> bool:
        """
        Öncelik 2: Dinamik engelde sabit DYNAMIC_BRAKE_DURATION saniye fren bas,
        sonra coast'a geç.
        """
        if self.dynamic_mode == 1:
            elapsed = now - (self.dynamic_brake_start_time or now)

            if elapsed < DYNAMIC_BRAKE_DURATION:
                self.is_stopped_for_dynamic = True
                self._apply_brake()
                self.node.get_logger().warn(
                    f"Dinamik engel — fren {elapsed:.1f}/{DYNAMIC_BRAKE_DURATION}s")
            else:
                if self.is_stopped_for_dynamic:
                    self.node.get_logger().info(
                        "Dinamik fren süresi doldu — coast moduna geçildi.")
                    self.is_stopped_for_dynamic = False
                self._apply_coast()
            return True

        self.is_stopped_for_dynamic = False
        return False

    def _check_park(self, cx: float, cy: float) -> bool:
        """Öncelik 3: Park noktasına ulaşıldıysa kalıcı dur."""
        if not self.park_point or self.has_braked_at_park:
            return False
        px, py = self.park_point
        if not self._within_threshold(cx, cy, px, py):
            return False
        self.node.get_logger().warn(
            "!!! GÖREV TAMAMLANDI — Park noktasına ulaşıldı. Kalıcı fren. !!!")
        self.has_braked_at_park = True
        self._apply_brake()
        return True

    def _check_station(self, cx: float, cy: float, now: float) -> bool:
        """Öncelik 4: İstasyon durak noktalarında fren + bekleme."""
        if self.current_stop_index >= len(self.brake_points):
            return False

        tx, ty = self.brake_points[self.current_stop_index]

        if not self.has_braked:
            if not self._within_threshold(cx, cy, tx, ty):
                return False
            self.node.get_logger().warn(
                f"{self.current_stop_index + 1}. durağa ulaşıldı — "
                f"{self.brake_duration}s fren.")
            self.has_braked          = True
            self.stop_start_time     = now
            self.is_waiting_after_brake = False
            self._apply_brake()
            return True

        if not self.is_waiting_after_brake:
            if now - self.stop_start_time < self.brake_duration:
                self._apply_brake()
                return True
            self.node.get_logger().warn(
                f"Fren tamamlandı — {self.wait_duration}s bekleme başlıyor.")
            self.is_waiting_after_brake = True
            self.wait_start_time        = now
            self._apply_coast()
            return True

        if now - self.wait_start_time < self.wait_duration:
            self._apply_coast()
            return True

        self.node.get_logger().warn("Bekleme tamamlandı — sonraki durağa devam.")
        self.has_braked             = False
        self.is_waiting_after_brake = False
        self.stop_start_time        = None
        self.wait_start_time        = None
        self.current_stop_index    += 1
        return False

    # ════════════════════════════════════════════════════════════════════════
    # MAIN ORCHESTRATOR
    # ════════════════════════════════════════════════════════════════════════

    def manage_brake(self, current_x: float, current_y: float) -> bool:
        now = self.node.get_clock().now().nanoseconds * 1e-9

        if self._check_traffic_light(now):             return True  # Öncelik 1
        if self._check_dynamic(now):                   return True  # Öncelik 2
        if self._check_park(current_x, current_y):    return True  # Öncelik 3
        if self._check_station(current_x, current_y, now): return True  # Öncelik 4

        self._apply_throttle()
        return False


# ════════════════════════════════════════════════════════════════════════════
# ROS2 NODE
# ════════════════════════════════════════════════════════════════════════════

class BrakeNode(Node):
    def __init__(self):
        super().__init__('brake_node')
        self.brake_manager = BrakeManager(
            self, dist_threshold=1.0, brake_duration=3.0, wait_duration=5.0
        )
        self.create_subscription(
            Odometry, '/carla/hero/odometry', self.odom_callback, 10
        )

    def odom_callback(self, msg: Odometry) -> None:
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