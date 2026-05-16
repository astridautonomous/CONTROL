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
import json
import math
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy


class BrakeManager:
    def __init__(self, node: Node, dist_threshold: float = 1.5,
                 brake_duration: float = 3.0, wait_duration: float = 5.0,
                 park_dist_threshold: float = 1.0,
                 station_dist_threshold: float = 2.0,
                 gorev_dist_threshold: float = 0.75,
                 gorev_brake_duration: float = 3.0,
                 gorev_wait_duration: float = 5.0,
                 throttle_value: float = 0.4,
                 throttle_int: int = 75,
                 tl_brake_duration: float = 3.0,
                 dynamic_brake_duration: float = 3.0,
                 map_file: str = "",
                 geojson_file: str = ""):
        self.node                  = node
        self.dist_threshold        = dist_threshold
        self.throttle_value        = throttle_value
        self.throttle_int          = throttle_int
        self.tl_brake_duration     = tl_brake_duration
        self.dynamic_brake_duration = dynamic_brake_duration
        self.map_file              = map_file
        self.geojson_file          = geojson_file

        # ── Ayrı mesafe eşikleri ─────────────────────────────────────────────
        self.park_dist_threshold_sq    = park_dist_threshold    * park_dist_threshold
        self.station_dist_threshold_sq = station_dist_threshold * station_dist_threshold

        self.brake_duration = brake_duration
        self.wait_duration  = wait_duration

        # ── Dinamik engel ────────────────────────────────────────────────────
        self.dynamic_mode             = 0
        self.is_stopped_for_dynamic   = False
        self.dynamic_brake_start_time = None

        # ── Trafik ışığı ─────────────────────────────────────────────────────
        self.traffic_light_status            = 0
        self.is_stopped_at_traffic_light     = False
        self.traffic_light_brake_start_time  = None
        self.traffic_light_recently_released = False
        self.traffic_light_release_time      = None
        self.traffic_light_release_delay     = 2.0

        self.current_steer_cmd  = 0.0
        self.park_point         = None
        self.has_braked_at_park = False
        self.park_reached       = False

        # ── Her durak için bağımsız state ────────────────────────────────────
        self.station_states = []

        # ── GeoJSON Görev Noktaları ──────────────────────────────────────────
        self.gorev_dist_threshold_sq = gorev_dist_threshold * gorev_dist_threshold
        self.gorev_brake_duration    = gorev_brake_duration
        self.gorev_wait_duration     = gorev_wait_duration
        self.gorev_points            = []
        self.gorev_states            = []

        self.transient_qos_perception = QoSProfile(
            depth=5,
            durability=QoSDurabilityPolicy.VOLATILE,
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
        )

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
            self.traffic_light_callback, self.transient_qos_perception)
        self.node.create_subscription(
            Float32, '/astrid/control/steer_cmd',
            self.steering_cmd_callback, 10)

        # ── Map Loading ──────────────────────────────────────────────────────
        self._load_gorev_points("/home/talha/Kodlar/ornek_1.geojson")
        filename  = "/home/talha/Kodlar/carla_test.osm"
        filename  = self.map_file
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

        # ── Brake / Park Points (Lanelet2) ───────────────────────────────────
        self.brake_points = []
        stations = {
            "durak1": [],
            "durak2": [],
            "durak3": [],
            "park5":  [],
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

        # Her durak için bağımsız state oluştur
        self.station_states = [
            {
                "has_braked":      False,
                "is_waiting":      False,
                "stop_start_time": None,
                "wait_start_time": None,
                "done":            False,
                "active":          False,
            }
            for _ in self.brake_points
        ]

        park_pts = stations["park5"]
        if park_pts:
            self.park_point = park_pts[-1]
            self.node.get_logger().info(
                f"Park noktası ayarlandı: {self.park_point}")
            self.has_braked_at_park = False
        else:
            self.node.get_logger().warn("park2 için nokta bulunamadı.")

        # ── GeoJSON'dan Görev Noktalarını Yükle ─────────────────────────────
        self._load_gorev_points(self.geojson_file)

    # ════════════════════════════════════════════════════════════════════════
    # GEOJSON LOADER
    # ════════════════════════════════════════════════════════════════════════
    def _load_gorev_points(self, geojson_path: str) -> None:
        """GeoJSON dosyasından 'gorev_' ile başlayan noktaları sıralı yükler."""
        try:
            with open(geojson_path, "r", encoding="utf-8") as f:
                data = json.load(f)
        except Exception as e:
            self.node.get_logger().error(
                f"GeoJSON okunamadı: {geojson_path} — {e}")
            return

        gorev_features = sorted(
            [
                feat for feat in data.get("features", [])
                if feat.get("properties", {}).get("name", "").startswith("gorev_")
            ],
            key=lambda f: f["properties"]["name"],
        )

        for feat in gorev_features:
            props = feat["properties"]
            name  = props.get("name", "?")
            lx    = props.get("local_x")
            ly    = props.get("local_y")
            if lx is None or ly is None:
                self.node.get_logger().warn(
                    f"'{name}' için local_x/local_y eksik, atlandı.")
                continue
            self.gorev_points.append((float(lx), float(ly), name))
            self.gorev_states.append({
                "has_braked":      False,
                "is_waiting":      False,
                "stop_start_time": None,
                "wait_start_time": None,
                "done":            False,
            })

        # ── Koordinat Özet Logu ──────────────────────────────────────────────
        self.node.get_logger().info(
            f"Toplam {len(self.gorev_points)} görev noktası yüklendi.")

        if self.gorev_points:
            header = (
                f"\n{'─' * 60}\n"
                f"  {'#':>3}  {'İsim':<22}  {'X (local)':>12}  {'Y (local)':>12}\n"
                f"{'─' * 60}"
            )
            rows = "\n".join(
                f"  {idx:>3}. {name:<22}  {lx:>12.4f}  {ly:>12.4f}"
                for idx, (lx, ly, name) in enumerate(self.gorev_points, start=1)
            )
            footer = f"{'─' * 60}"
            self.node.get_logger().info(
                f"[GeoJSON Görev Noktaları]{header}\n{rows}\n{footer}"
            )

    # ════════════════════════════════════════════════════════════════════════
    # CALLBACKS
    # ════════════════════════════════════════════════════════════════════════
    def traffic_light_callback(self, msg: String) -> None:
        msg_list = json.loads(msg.data)
        raw = msg_list[0][0]
        if raw == 'yesil-isik':
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
            self.traffic_light_brake_start_time = None
            self.node.get_logger().info("traffic_light_status = 0 (yeşil)")
        elif raw == 'kirmizi-isik':
            if self.traffic_light_status != 1:
                self.traffic_light_brake_start_time = (
                    self.node.get_clock().now().nanoseconds * 1e-9
                )
                self.node.get_logger().warn(
                    f"Kırmızı ışık — {self.tl_brake_duration}s fren başlatıldı.")
            self.traffic_light_status            = 1
            self.traffic_light_recently_released = False
            self._apply_brake()
        else:
            self.node.get_logger().warn(
                f"Tanınmayan trafik ışığı mesajı: '{raw}'")

    def steering_cmd_callback(self, msg: Float32) -> None:
        self.current_steer_cmd = msg.data

    def dynamic_mode_callback(self, msg: Int8) -> None:
        prev = self.dynamic_mode
        self.dynamic_mode = msg.data
        self.node.get_logger().info(f"Dinamik mod: {self.dynamic_mode}")
        if self.dynamic_mode == 1 and prev != 1:
            self.dynamic_brake_start_time = (
                self.node.get_clock().now().nanoseconds * 1e-9
            )
            self.node.get_logger().warn(
                f"Dinamik engel algılandı — {self.dynamic_brake_duration}s fren başlatıldı.")
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
        t_msg.data = self.throttle_int if throttle > 0.0 else 0
        b_msg.data = 1            if brake    > 0.0 else 0
        self.throttle_pub.publish(t_msg)
        self.brake_pub.publish(b_msg)

    def _apply_brake(self)    -> None: self._send_control(throttle=0.0,           brake=1.0)
    def _apply_throttle(self) -> None: self._send_control(throttle=self.throttle_value, brake=0.0)
    def _apply_coast(self)    -> None: self._send_control(throttle=0.0,           brake=0.0)

    # ════════════════════════════════════════════════════════════════════════
    # HELPERS
    # ════════════════════════════════════════════════════════════════════════
    def _within_park_threshold(self, cx, cy, tx, ty) -> bool:
        dx, dy = cx - tx, cy - ty
        return (dx * dx + dy * dy) < self.park_dist_threshold_sq

    def _within_station_threshold(self, cx, cy, tx, ty) -> bool:
        dx, dy = cx - tx, cy - ty
        return (dx * dx + dy * dy) < self.station_dist_threshold_sq

    def _within_gorev_threshold(self, cx, cy, tx, ty) -> bool:
        dx, dy = cx - tx, cy - ty
        return (dx * dx + dy * dy) < self.gorev_dist_threshold_sq

    def inside_traffic_light_area(self, x: float, y: float) -> bool:
        point = ShapelyPoint(x, y)
        return any(poly.contains(point) for poly in self.traffic_light_areas)

    # ════════════════════════════════════════════════════════════════════════
    # PRIORITY CHECKS
    # ════════════════════════════════════════════════════════════════════════
    def _check_traffic_light(self, now: float) -> bool:
        if self.traffic_light_status == 1:
            elapsed = now - (self.traffic_light_brake_start_time or now)
            if elapsed < self.tl_brake_duration:
                self.is_stopped_at_traffic_light = True
                self._apply_brake()
                self.node.get_logger().warn(
                    f"Trafik ışığı kırmızı — fren {elapsed:.1f}/{self.tl_brake_duration}s")
            else:
                if self.is_stopped_at_traffic_light:
                    self.node.get_logger().info(
                        "Trafik ışığı fren süresi doldu — coast moduna geçildi.")
                    self.is_stopped_at_traffic_light = False
                self._apply_coast()
            return True
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
        if self.dynamic_mode == 1:
            elapsed = now - (self.dynamic_brake_start_time or now)
            if elapsed < self.dynamic_brake_duration:
                self.is_stopped_for_dynamic = True
                self._apply_brake()
                self.node.get_logger().warn(
                    f"Dinamik engel — fren {elapsed:.1f}/{self.dynamic_brake_duration}s")
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
        if self.park_reached:
            self._apply_brake()
            return True
        if not self.park_point or self.has_braked_at_park:
            return False
        px, py = self.park_point
        dist_sq = (cx - px) ** 2 + (cy - py) ** 2
        self.node.get_logger().info(
            f"[PARK DEBUG] araç=({cx:.2f},{cy:.2f}) "
            f"hedef=({px:.2f},{py:.2f}) "
            f"mesafe={math.sqrt(dist_sq):.2f}m eşik=0.5m"
        )
        if not self._within_park_threshold(cx, cy, px, py):
            return False
        self.node.get_logger().warn(
            "!!! GÖREV TAMAMLANDI — Park noktasına ulaşıldı. Kalıcı fren. !!!")
        self.has_braked_at_park = True
        self.park_reached       = True
        self._apply_brake()
        return True

    def _check_station(self, cx: float, cy: float, now: float) -> bool:
        """Öncelik 4: Tüm durak noktalarını bağımsız olarak kontrol et."""
        for i, (point, state) in enumerate(
                zip(self.brake_points, self.station_states)):
            if state["done"]:
                continue
            tx, ty = point
            # ── Henüz fren başlamadı → yakınlık kontrolü ────────────────────
            if not state["has_braked"]:
                if not self._within_station_threshold(cx, cy, tx, ty):
                    continue
                self.node.get_logger().warn(
                    f"{i + 1}. durağa ulaşıldı — {self.brake_duration}s fren.")
                state["has_braked"]      = True
                state["active"]          = True
                state["stop_start_time"] = now
                state["is_waiting"]      = False
                self._apply_brake()
                return True
            # ── Fren aşaması ────────────────────────────────────────────────
            if not state["is_waiting"]:
                if now - state["stop_start_time"] < self.brake_duration:
                    self._apply_brake()
                    return True
                self.node.get_logger().warn(
                    f"{i + 1}. durak freni tamamlandı — "
                    f"{self.wait_duration}s bekleme başlıyor.")
                state["is_waiting"]      = True
                state["wait_start_time"] = now
                self._apply_coast()
                return True
            # ── Bekleme aşaması ─────────────────────────────────────────────
            if now - state["wait_start_time"] < self.wait_duration:
                self._apply_coast()
                return True
            # ── Durak tamamlandı ─────────────────────────────────────────────
            self.node.get_logger().warn(
                f"{i + 1}. durak bekleme tamamlandı — devam ediliyor.")
            state["done"]   = True
            state["active"] = False
            return False
        return False

    def _check_gorev(self, cx: float, cy: float, now: float) -> bool:
        """Öncelik 5: GeoJSON'dan yüklenen görev noktalarını sırayla kontrol et."""
        for i, ((tx, ty, name), state) in enumerate(
                zip(self.gorev_points, self.gorev_states)):
            if state["done"]:
                continue
            # ── Henüz frenlenmedi → yakınlık kontrolü ───────────────────────
            if not state["has_braked"]:
                dist_sq = (cx - tx) ** 2 + (cy - ty) ** 2
                self.node.get_logger().info(
                    f"[GÖREV DEBUG] araç=({cx:.2f},{cy:.2f}) "
                    f"hedef={name}({tx:.2f},{ty:.2f}) "
                    f"mesafe={math.sqrt(dist_sq):.2f}m "
                    f"eşik={math.sqrt(self.gorev_dist_threshold_sq):.1f}m"
                )
                if not self._within_gorev_threshold(cx, cy, tx, ty):
                    continue
                self.node.get_logger().warn(
                    f"Görev noktasına ulaşıldı → {name} "
                    f"({tx:.2f}, {ty:.2f}) — {self.gorev_brake_duration}s fren.")
                state["has_braked"]      = True
                state["stop_start_time"] = now
                state["is_waiting"]      = False
                self._apply_brake()
                return True
            # ── Fren aşaması ─────────────────────────────────────────────────
            if not state["is_waiting"]:
                if now - state["stop_start_time"] < self.gorev_brake_duration:
                    self._apply_brake()
                    return True
                self.node.get_logger().warn(
                    f"{name} freni tamamlandı — "
                    f"{self.gorev_wait_duration}s bekleme başlıyor.")
                state["is_waiting"]      = True
                state["wait_start_time"] = now
                self._apply_coast()
                return True
            # ── Bekleme aşaması ───────────────────────────────────────────────
            if now - state["wait_start_time"] < self.gorev_wait_duration:
                self._apply_coast()
                return True
            # ── Görev noktası tamamlandı ──────────────────────────────────────
            self.node.get_logger().warn(
                f"{name} bekleme tamamlandı — sonraki hedefe devam ediliyor.")
            state["done"] = True
            return False
        return False

    # ════════════════════════════════════════════════════════════════════════
    # MAIN ORCHESTRATOR
    # ════════════════════════════════════════════════════════════════════════
    def manage_brake(self, current_x: float, current_y: float) -> bool:
        now = self.node.get_clock().now().nanoseconds * 1e-9
        if self._check_gorev(current_x, current_y, now):   return True
        if self._check_traffic_light(now):                  return True  # Öncelik 1
        if self._check_dynamic(now):                        return True  # Öncelik 2
        if self._check_park(current_x, current_y):         return True  # Öncelik 3
        if self._check_station(current_x, current_y, now): return True  # Öncelik 4
        self._apply_throttle()
        return False

# ════════════════════════════════════════════════════════════════════════════
# ROS2 NODE
# ════════════════════════════════════════════════════════════════════════════
class BrakeNode(Node):
    def __init__(self):
        super().__init__('brake_node')

        # ── Parametre bildirimleri ────────────────────────────────────────────
        self.declare_parameter('throttle_value',        0.4)
        self.declare_parameter('throttle_int',          75)
        self.declare_parameter('tl_brake_duration',     3.0)
        self.declare_parameter('dynamic_brake_duration',3.0)
        self.declare_parameter('brake_duration',        3.0)
        self.declare_parameter('wait_duration',         5.0)
        self.declare_parameter('dist_threshold',        1.5)
        self.declare_parameter('park_dist_threshold',   1.0)
        self.declare_parameter('station_dist_threshold',2.0)
        self.declare_parameter('gorev_dist_threshold',  0.75)
        self.declare_parameter('gorev_brake_duration',  3.0)
        self.declare_parameter('gorev_wait_duration',   5.0)
        self.declare_parameter('map_file',              '')
        self.declare_parameter('geojson_file',          '')

        self.brake_manager = BrakeManager(
            self,
            dist_threshold          = self.get_parameter('dist_threshold').value,
            brake_duration          = self.get_parameter('brake_duration').value,
            wait_duration           = self.get_parameter('wait_duration').value,
            park_dist_threshold     = self.get_parameter('park_dist_threshold').value,
            station_dist_threshold  = self.get_parameter('station_dist_threshold').value,
            gorev_dist_threshold    = self.get_parameter('gorev_dist_threshold').value,
            gorev_brake_duration    = self.get_parameter('gorev_brake_duration').value,
            gorev_wait_duration     = self.get_parameter('gorev_wait_duration').value,
            throttle_value          = self.get_parameter('throttle_value').value,
            throttle_int            = self.get_parameter('throttle_int').value,
            tl_brake_duration       = self.get_parameter('tl_brake_duration').value,
            dynamic_brake_duration  = self.get_parameter('dynamic_brake_duration').value,
            map_file                = self.get_parameter('map_file').value,
            geojson_file            = self.get_parameter('geojson_file').value,
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
