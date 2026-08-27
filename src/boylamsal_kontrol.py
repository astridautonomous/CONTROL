import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from std_msgs.msg import Int8, Int16, String, Float32
from geometry_msgs.msg import PoseArray
from shapely.geometry import Point as ShapelyPoint, Polygon
import lanelet2
import lanelet2.io
import lanelet2.traffic_rules
import lanelet2.projection
import json
import math
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy

DEFAULT_MAP_FILE     = "/home/talha/Desktop/Robotaksi_final/3.gun_kodları/map/QF_map.osm"
DEFAULT_GEOJSON_FILE = "/home/talha/Desktop/Robotaksi_final/hakem_kodları/geojson/ana_map.geojson"

DEFAULT_PARAMS = {
    'brake_duration':         5.0,
    'traffic_light_red_timeout': 20.0,
    'park_dist_threshold':    1.0,
    'park_wait_duration':     5.0,
    'station_dist_threshold': 1.5,
    'station_wait_duration':  5.0,
    'gorev_dist_threshold':   1.5,
    'gorev_wait_duration':    5.0,
    'throttle_int':           40,
    'map_file':               DEFAULT_MAP_FILE,
    'geojson_file':           DEFAULT_GEOJSON_FILE,
}


class BrakeManager:
    def __init__(self, node: Node,
                 brake_duration: float = DEFAULT_PARAMS['brake_duration'],
                 traffic_light_red_timeout: float = DEFAULT_PARAMS['traffic_light_red_timeout'],
                 park_dist_threshold: float = DEFAULT_PARAMS['park_dist_threshold'],
                 park_wait_duration: float = DEFAULT_PARAMS['park_wait_duration'],
                 station_dist_threshold: float = DEFAULT_PARAMS['station_dist_threshold'],
                 station_wait_duration: float = DEFAULT_PARAMS['station_wait_duration'],
                 gorev_dist_threshold: float = DEFAULT_PARAMS['gorev_dist_threshold'],
                 gorev_wait_duration: float = DEFAULT_PARAMS['gorev_wait_duration'],
                 throttle_int: int = DEFAULT_PARAMS['throttle_int'],
                 map_file: str = DEFAULT_PARAMS['map_file'],
                 geojson_file: str = DEFAULT_PARAMS['geojson_file']):
        self.node                  = node
        self.throttle_int          = throttle_int
        self.map_file              = map_file
        self.geojson_file          = geojson_file
        # ── Ayrı mesafe eşikleri ─────────────────────────────────────────────
        self.park_dist_threshold_sq    = park_dist_threshold    * park_dist_threshold
        self.station_dist_threshold_sq = station_dist_threshold * station_dist_threshold
        self.brake_duration = brake_duration
        self.park_wait_duration    = park_wait_duration
        self.station_wait_duration = station_wait_duration
        # ── Trafik ışığı kırmızıda azami bekleme (timeout) süresi ────────────
        # Kırmızı ışık algılandıktan sonra bu süre dolana kadar yeşil
        # görülmezse, araç otomatik olarak gaza geçer (sensör/algılama
        # sorunlarında sonsuza kadar kilitli kalmamak için).
        self.traffic_light_red_timeout = traffic_light_red_timeout
        # ── Dinamik engel ────────────────────────────────────────────────────
        self.dynamic_mode             = 0
        self.is_stopped_for_dynamic   = False
        self.dynamic_brake_start_time = None
        self.dynamic_one_count        = 0
        self.dynamic_zero_count       = 0
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
        self.park_is_waiting      = False
        self.park_wait_start_time = None
        self.park_stop_time       = None
        self.park_finished        = False
        # ── Her durak için bağımsız state ────────────────────────────────────
        self.station_states = []
        # ── GeoJSON Görev Noktaları ──────────────────────────────────────────
        self.gorev_dist_threshold_sq = gorev_dist_threshold * gorev_dist_threshold
        self.gorev_wait_duration     = gorev_wait_duration
        self.gorev_points            = []
        self.gorev_states            = []
        self.transient_qos_perception = QoSProfile(
            depth=5,
            durability=QoSDurabilityPolicy.VOLATILE,
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
        )
        # ── Publishers ───────────────────────────────────────────────────────
        self.throttle_pub = self.node.create_publisher(
            Int16, '/astrid/control/throttle_cmd', 10)
        self.brake_pub    = self.node.create_publisher(
            Int16, '/astrid/control/brake_cmd', 10)
        # ── Subscribers ──────────────────────────────────────────────────────
        self.node.create_subscription(
            Int8, '/astrid/slam/dynamic_mode',
            self.dynamic_mode_callback, 10)
        self.node.create_subscription(
            String, '/astrid/perception/traffic_sign',
            self.traffic_light_callback, self.transient_qos_perception)
        # ── OSM (Lanelet2) haritasını yükle ───────────────────────────────────
        origin    = lanelet2.io.Origin(0.0, 0.0)
        projector = lanelet2.projection.LocalCartesianProjector(origin)
        lanelet2.traffic_rules.create(
            lanelet2.traffic_rules.Locations.Germany,
            lanelet2.traffic_rules.Participants.Vehicle,
        )
        self.map = lanelet2.io.load(self.map_file, projector)
        # ── Brake / Park Points (Lanelet2) ───────────────────────────────────
        self.brake_points = []
        stations = {
            "durak1": [],
            "durak2": [],
            "durak3": [],
            "park3":  [],
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
        park_pts = stations["park3"]
        if park_pts:
            self.park_point = park_pts[-1]
            self.node.get_logger().info(
                f"Park noktası ayarlandı: {self.park_point}")
            self.has_braked_at_park = False
        else:
            self.node.get_logger().warn("park3 için nokta bulunamadı.")
        # ── GeoJSON'dan Görev Noktalarını Yükle (TEK ÇAĞRI) ─────────────────
        self._load_gorev_points(self.geojson_file)
    # ════════════════════════════════════════════════════════════════════════
    # GEOJSON LOADER
    # ════════════════════════════════════════════════════════════════════════
    def _load_gorev_points(self, geojson_path: str) -> None:
        """GeoJSON dosyasından 'gorev_' ile başlayan noktaları sıralı yükler."""
        if self.gorev_points:
            self.node.get_logger().warn(
                "Görev noktaları zaten yüklü — tekrar yükleme atlandı "
                f"({geojson_path})."
            )
            return
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
            self.traffic_light_brake_start_time = None
            self.node.get_logger().info("traffic_light_status = 0 (yeşil)")
        elif raw == 'kirmizi_isik':
            if self.traffic_light_status != 1:
                self.traffic_light_brake_start_time = (
                    self.node.get_clock().now().nanoseconds * 1e-9
                )
                self.node.get_logger().warn(
                    f"Kırmızı ışık — {self.brake_duration}s fren başlatıldı.")
            self.traffic_light_status            = 1
            self.traffic_light_recently_released = False
            self._apply_brake()
        else:
            self.node.get_logger().warn(
                f"Tanınmayan trafik ışığı mesajı: '{raw}'")
    def steering_cmd_callback(self, msg: Float32) -> None:
        self.current_steer_cmd = msg.data
    def dynamic_mode_callback(self, msg: Int8) -> None:
        raw = msg.data

        # ── 5 art arda aynı değer gelmeden state değişmesin ──────────────────
        if raw == 1:
            self.dynamic_one_count  += 1
            self.dynamic_zero_count  = 0
        elif raw == 0:
            self.dynamic_zero_count += 1
            self.dynamic_one_count   = 0
        else:
            self.dynamic_one_count  = 0
            self.dynamic_zero_count = 0

        self.node.get_logger().info(
            f"Dinamik engel sayaç → 1: {self.dynamic_one_count}/5  "
            f"0: {self.dynamic_zero_count}/5  (gelen: {raw})")

        prev = self.dynamic_mode

        if raw == 1 and self.dynamic_one_count >= 5 and self.dynamic_mode != 1:
            self.dynamic_mode = 1
        elif raw == 0 and self.dynamic_zero_count >= 5 and self.dynamic_mode != 0:
            self.dynamic_mode = 0
        else:
            return

        self.node.get_logger().info(f"Dinamik mod: {self.dynamic_mode}")
        if self.dynamic_mode == 1 and prev != 1:
            self.dynamic_brake_start_time = (
                self.node.get_clock().now().nanoseconds * 1e-9
            )
            self.node.get_logger().warn(
                f"Dinamik engel algılandı — {self.brake_duration}s fren başlatıldı.")
        if self.dynamic_mode == 0 and self.is_stopped_for_dynamic:
            self.node.get_logger().warn("Engel kalktı. Gaz veriliyor.")
            self._apply_throttle()
            self.is_stopped_for_dynamic   = False
            self.dynamic_brake_start_time = None
    # ════════════════════════════════════════════════════════════════════════
    # LOW-LEVEL SEND HELPERS
    # ════════════════════════════════════════════════════════════════════════
    def _send_control(self, throttle: float, brake: float) -> None:
        t_msg      = Int16()
        b_msg      = Int16()
        t_msg.data = self.throttle_int if throttle > 0.0 else 0
        b_msg.data = int(brake)  # 0 = fren yok, 1 = coast/throttle, 2 = sert fren
        self.throttle_pub.publish(t_msg)
        self.brake_pub.publish(b_msg)
    def _apply_brake(self)    -> None: self._send_control(throttle=0.0,                brake=1.0)
    def _apply_throttle(self) -> None: self._send_control(throttle=self.throttle_int, brake=2.0)
    def _apply_coast(self)    -> None: self._send_control(throttle=0.0,                brake=2.0)
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
            # ── 1) Sert fren aşaması ─────────────────────────────────────────
            if elapsed < self.brake_duration:
                self.is_stopped_at_traffic_light = True
                self._apply_brake()
                self.node.get_logger().warn(
                    f"Trafik ışığı kırmızı — fren {elapsed:.1f}/{self.brake_duration}s")
                return True
            # ── 2) Fren bitti, yeşili bekliyoruz (coast) ─────────────────────
            if self.is_stopped_at_traffic_light:
                self.node.get_logger().info(
                    "Trafik ışığı fren süresi doldu — coast moduna geçildi.")
                self.is_stopped_at_traffic_light = False
            # ── 3) Timeout kontrolü: yeşil hâlâ görülmediyse ──────────────────
            if elapsed >= self.traffic_light_red_timeout:
                self.node.get_logger().warn(
                    f"Kırmızı ışıkta {self.traffic_light_red_timeout}s "
                    f"boyunca yeşil algılanmadı — otomatik gaz veriliyor."
                )
                # Trafik ışığı state'ini tamamen sıfırla ki bu fonksiyon
                # bir daha "meşgul" (True) dönüp diğer öncelikleri (dinamik
                # engel, park, durak, görev) bloke etmesin.
                self.traffic_light_status            = 0
                self.traffic_light_brake_start_time  = None
                self.traffic_light_recently_released = False
                self.traffic_light_release_time      = None
                self._apply_throttle()
                # False dönerek kontrolü orkestratöre bırakıyoruz; böylece
                # aynı döngüde başka bir fren noktası (park/durak/görev/
                # dinamik engel) varsa o öncelik devreye girip gerekirse
                # tekrar fren uygulayabilir.
                return False
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
            if elapsed < self.brake_duration:
                self.is_stopped_for_dynamic = True
                self._apply_brake()
                self.node.get_logger().warn(
                    f"Dinamik engel — fren {elapsed:.1f}/{self.brake_duration}s")
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
        now = self.node.get_clock().now().nanoseconds * 1e-9
        if self.park_finished:
            self._apply_coast()
            return True
        if self.park_reached:
            if not self.park_is_waiting:
                if now - self.park_stop_time < self.brake_duration:
                    self._apply_brake()
                    return True
                self.node.get_logger().warn(
                    f"Park freni tamamlandı — {self.park_wait_duration}s bekleme başlıyor.")
                self.park_is_waiting      = True
                self.park_wait_start_time = now
                self._apply_coast()
                return True
            if now - self.park_wait_start_time < self.park_wait_duration:
                self._apply_coast()
                return True
            self.node.get_logger().warn(
                "Park bekleme süresi doldu — coast moduna geçildi.")
            self.park_finished = True
            self._apply_coast()
            return True
        if not self.park_point or self.has_braked_at_park:
            return False
        px, py = self.park_point
        dist_sq = (cx - px) ** 2 + (cy - py) ** 2
        #self.node.get_logger().info(
        #    f"[PARK DEBUG] araç=({cx:.2f},{cy:.2f}) "
        #    f"hedef=({px:.2f},{py:.2f}) "
        #    f"mesafe={math.sqrt(dist_sq):.2f}m eşik=0.5m"
        #)
        if not self._within_park_threshold(cx, cy, px, py):
            return False
        self.node.get_logger().warn(
            "!!! GÖREV TAMAMLANDI — Park noktasına ulaşıldı. Kalıcı fren. !!!")
        self.has_braked_at_park = True
        self.park_reached       = True
        self.park_stop_time     = now
        self._apply_brake()
        return True
    def _check_station(self, cx: float, cy: float, now: float) -> bool:
        """Öncelik 4: Tüm durak noktalarını bağımsız olarak kontrol et."""
        for i, (point, state) in enumerate(
                zip(self.brake_points, self.station_states)):
            if state["done"]:
                continue
            tx, ty = point
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
            if not state["is_waiting"]:
                if now - state["stop_start_time"] < self.brake_duration:
                    self._apply_brake()
                    return True
                self.node.get_logger().warn(
                    f"{i + 1}. durak freni tamamlandı — "
                    f"{self.station_wait_duration}s bekleme başlıyor.")
                state["is_waiting"]      = True
                state["wait_start_time"] = now
                self._apply_coast()
                return True
            if now - state["wait_start_time"] < self.station_wait_duration:
                self._apply_coast()
                return True
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
            if not state["has_braked"]:
                dist_sq = (cx - tx) ** 2 + (cy - ty) ** 2
                #self.node.get_logger().info(
                #    f"[GÖREV DEBUG] araç=({cx:.2f},{cy:.2f}) "
                #    f"hedef={name}({tx:.2f},{ty:.2f}) "
                #    f"mesafe={math.sqrt(dist_sq):.2f}m "
                #    f"eşik={math.sqrt(self.gorev_dist_threshold_sq):.1f}m"
                #)
                if not self._within_gorev_threshold(cx, cy, tx, ty):
                    continue
                self.node.get_logger().warn(
                    f"Görev noktasına ulaşıldı → {name} "
                    f"({tx:.2f}, {ty:.2f}) — {self.brake_duration}s fren.")
                state["has_braked"]      = True
                state["stop_start_time"] = now
                state["is_waiting"]      = False
                self._apply_brake()
                return True
            if not state["is_waiting"]:
                if now - state["stop_start_time"] < self.brake_duration:
                    self._apply_brake()
                    return True
                self.node.get_logger().warn(
                    f"{name} freni tamamlandı — "
                    f"{self.gorev_wait_duration}s bekleme başlıyor.")
                state["is_waiting"]      = True
                state["wait_start_time"] = now
                self._apply_coast()
                return True
            if now - state["wait_start_time"] < self.gorev_wait_duration:
                self._apply_coast()
                return True
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
        for name, default_value in DEFAULT_PARAMS.items():
            self.declare_parameter(name, default_value)
        params = {
            name: self.get_parameter(name).value
            for name in DEFAULT_PARAMS
        }
        self.brake_manager = BrakeManager(self, **params)
        self.create_subscription(
            Odometry, '/astrid/odometry_local', self.odom_callback, 10
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
