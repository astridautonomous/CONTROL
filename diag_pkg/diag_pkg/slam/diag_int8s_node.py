#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import (
    QoSProfile, QoSReliabilityPolicy,
    QoSDurabilityPolicy, QoSHistoryPolicy,
)
import threading
from datetime import datetime
from std_msgs.msg import Int8
from sensor_msgs.msg import NavSatFix
from diag_pkg.watchdog_mixin import WatchdogMixin

# ─── Sabitler ────────────────────────────────────────────────────────────────

MSG_TYPE_INT8     = "std_msgs/Int8"
MSG_TYPE_NAVSATFIX = "sensor_msgs/NavSatFix"

GPS_TOPIC = "/clap/ros/gps_nav_sat_fix"

TOPICS = [
    "/astrid/slam/dynamic_mode",
    "/astrid/slam/station_status",
    GPS_TOPIC,
]

SENSOR_QOS = QoSProfile(
    reliability=QoSReliabilityPolicy.BEST_EFFORT,
    durability=QoSDurabilityPolicy.VOLATILE,
    history=QoSHistoryPolicy.KEEP_LAST,
    depth=10,
)

RELIABLE_QOS = QoSProfile(
    reliability=QoSReliabilityPolicy.RELIABLE,
    durability=QoSDurabilityPolicy.VOLATILE,
    history=QoSHistoryPolicy.KEEP_LAST,
    depth=10,
)

SENSOR_TOPICS = {
    "/camera/camera/depth/img_rect_raw",
    "/camera/camera/color/image_raw",
    "/velodyne_points",
    "/scan",
    "/clap/ros/odometry",
    "/clap/odometry",
    GPS_TOPIC,
}

# Terminal renk kodları
RED    = "\033[91m"
GREEN  = "\033[92m"
YELLOW = "\033[93m"
CYAN   = "\033[96m"
BOLD   = "\033[1m"
RESET  = "\033[0m"

# GPS status değer açıklamaları
GPS_STATUS_LABELS = {
    -1: "NO_FIX",
     0: "FIX",
     1: "SBAS_FIX",
     2: "GBAS_FIX",
}


# ─── Node ────────────────────────────────────────────────────────────────────

class TopicWatchdog(WatchdogMixin, Node):

    def __init__(self):
        Node.__init__(self, "diag_int8s_node")
        WatchdogMixin.__init__(self)

        self.declare_parameter("timeout",    5.0)
        self.declare_parameter("check_rate", 1.0)

        self.timeout    = self.get_parameter("timeout").value
        self.check_rate = self.get_parameter("check_rate").value

        self._lock              = threading.Lock()
        self._last_received     = {t: None  for t in TOPICS}
        self._error_active      = {t: False for t in TOPICS}
        self._subscribed_topics: set[str] = set()
        self._generic_subs      = []

        # GPS'e özel: son gelen status değeri
        self._gps_status: int | None = None
        # GPS değer hatası ayrı takip edilir (timeout'tan bağımsız)
        self._gps_value_error_active: bool = False

        self._log_info(
            f"{CYAN}{BOLD}[TopicWatchdog]{RESET} "
            f"{len(TOPICS)} topic izleniyor | "
            f"timeout={self.timeout}s | check_rate={self.check_rate}Hz"
        )

        self._discovery_timer = self.create_timer(2.0, self._discover_and_subscribe)
        self._watchdog_timer  = self.create_timer(1.0 / self.check_rate, self._check_all_topics)
        self._summary_timer   = self.create_timer(10.0, self._request_summary)

    # ── Topic keşfi ───────────────────────────────────────────────────────────
    def _discover_and_subscribe(self):
        topic_type_map: dict[str, list[str]] = dict(self.get_topic_names_and_types())

        pending = [t for t in TOPICS if t not in self._subscribed_topics]
        if not pending:
            self._discovery_timer.cancel()
            self._log_info(
                f"{GREEN}{BOLD}[TopicWatchdog]{RESET} "
                f"Tüm topic'lere subscribe olundu."
            )
            return

        for topic in pending:
            if topic not in topic_type_map or not topic_type_map[topic]:
                continue

            qos = SENSOR_QOS if topic in SENSOR_TOPICS else RELIABLE_QOS
            try:
                
                if topic == GPS_TOPIC:
                    sub = self.create_subscription(
                        NavSatFix,
                        topic,
                        self._make_gps_callback(topic),
                        qos,
                    )
                else:
                    sub = self.create_subscription(
                        Int8,
                        topic,
                        self._make_callback(topic),
                        qos,
                    )
                self._generic_subs.append(sub)
                self._subscribed_topics.add(topic)
                self.get_logger().debug(
                    f"[TopicWatchdog] Abone olundu: {topic}"
                )
            except Exception as exc:
                self.get_logger().warn(
                    f"[TopicWatchdog] Subscribe hatası ({topic}): {exc}"
                )

    # ── Callback (Int8 topic'ler) ───────────────────────────────────
    def _make_callback(self, topic: str):
        def _cb(_msg):
            with self._lock:
                was_error = self._error_active[topic]
                self._last_received[topic] = self.get_clock().now()
                if was_error:
                    self._error_active[topic] = False
                    self._log_warn(
                        f"{GREEN}{BOLD}[TopicWatchdog] RECOVERED{RESET}  "
                        f"'{topic}' yeniden yayın yapıyor."
                    )
        return _cb

    # ── GNSS callback (NavSatFix — status değeri de kontrol edilir) ────────────
    def _make_gps_callback(self, topic: str):
        def _cb(msg: NavSatFix):
            with self._lock:
                was_error = self._error_active[topic]
                self._last_received[topic] = self.get_clock().now()
                self._gps_status = msg.status.status  # 0, -1, 1 veya 2

                # Timeout'tan çıkış (RECOVERED)
                if was_error:
                    self._error_active[topic] = False
                    self._log_warn(
                        f"{GREEN}{BOLD}[TopicWatchdog] RECOVERED{RESET}  "
                        f"'{topic}' yeniden yayın yapıyor."
                    )

                ts = datetime.now().strftime("%H:%M:%S")
                label = GPS_STATUS_LABELS.get(self._gps_status, f"UNKNOWN({self._gps_status})")

                # status -1 → ERROR
                if self._gps_status == -1:
                    if not self._gps_value_error_active:
                        self._gps_value_error_active = True
                        self._log_error(
                            f"{RED}{BOLD}[TopicWatchdog] ERROR [{ts}]{RESET}  "
                            f"'{topic}'  ──  GNSS status={self._gps_status}"
                        )
                # status 1 veya 2 → WARN
                elif self._gps_status in (1, 2):
                    if not self._gps_value_error_active:
                        self._gps_value_error_active = True
                        self._log_warn(
                            f"{YELLOW}{BOLD}[TopicWatchdog] WARN [{ts}]{RESET}  "
                            f"'{topic}'  ──  GNSS status={self._gps_status}"
                        )
                # status 0 → OK, aktif uyarı/hata varsa kapat
                elif self._gps_status == 0:
                    if self._gps_value_error_active:
                        self._gps_value_error_active = False
                        self._log_warn(
                            f"{GREEN}{BOLD}[TopicWatchdog] RECOVERED{RESET}  "
                            f"'{topic}'  ──  GNSS status 0'a döndü ({label})."
                        )
        return _cb

    # ── Periyodik kontrol ─────────────────────────────────────────────────────
    def _check_all_topics(self):
        now = self.get_clock().now()
        with self._lock:
            for topic in TOPICS:
                last    = self._last_received[topic]
                elapsed = float("inf") if last is None else (now - last).nanoseconds / 1e9

                # ── Önce bağlantı/timeout durumunu belirle ────────────────
                if topic not in self._subscribed_topics:
                    status     = "NO_PUBLISHER"
                    pub_elapse = None
                elif last is None:
                    status     = "NO_MSG"
                    pub_elapse = -1
                elif elapsed > self.timeout:
                    status     = "TIMEOUT"
                    pub_elapse = elapsed
                else:
                    status     = "OK"
                    pub_elapse = elapsed

                # ── GNSS topic'i için ek değer kontrolü ────────────────────
                if topic == GPS_TOPIC and status == "OK":
                    label = GPS_STATUS_LABELS.get(self._gps_status, "?")
                    if self._gps_status == -1:
                        status = f"GNSS Error (status=-1)"
                    elif self._gps_status in (1, 2):
                        status = f"GNSS Warn (status={self._gps_status})"
                    # 0 → status "OK" olarak kalır

                # Msg type: GPS için NavSatFix, diğerleri Int8
                msg_type = MSG_TYPE_NAVSATFIX if topic == GPS_TOPIC else MSG_TYPE_INT8
                self.publish_status(topic, status, pub_elapse, msg_type, self.timeout)

                # ── Timeout log — sadece yeni hata açılırken ──────────────
                if elapsed > self.timeout and not self._error_active[topic]:
                    self._error_active[topic] = True
                    ts = datetime.now().strftime("%H:%M:%S")

                    if topic not in self._subscribed_topics:
                        self._log_error(
                            f"{RED}{BOLD}[TopicWatchdog] ERROR [{ts}]{RESET}  "
                            f"'{topic}'  ──  Topic henüz ROS ağında bulunamadı! "
                            f"(publisher yok)"
                        )
                    elif last is None:
                        self._log_error(
                            f"{RED}{BOLD}[TopicWatchdog] ERROR [{ts}]{RESET}  "
                            f"'{topic}'  ──  Hiç mesaj gelmedi!"
                        )
                    else:
                        self._log_error(
                            f"{RED}{BOLD}[TopicWatchdog] ERROR [{ts}]{RESET}  "
                            f"'{topic}'  ──  {elapsed:.1f}s süredir mesaj yok "
                            f"(eşik: {self.timeout}s)"
                        )

    # ── Summary tetikleyici ───────────────────────────────────────────────────
    def _request_summary(self):
        self.publish_status("__summary__", "SUMMARY", None, MSG_TYPE_INT8, self.timeout)


# ─── Giriş noktası ───────────────────────────────────────────────────────────

def main(args=None):
    rclpy.init(args=args)
    node = TopicWatchdog()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
