#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import (
    QoSProfile, QoSReliabilityPolicy,
    QoSDurabilityPolicy, QoSHistoryPolicy,
)
import threading
from datetime import datetime
from visualization_msgs.msg import MarkerArray
from diag_pkg.watchdog_mixin import WatchdogMixin

# ─── Sabitler ────────────────────────────────────────────────────────────────

MSG_TYPE = "visualization_msgs/MarkerArray"

TOPICS = [
    "/astrid/slam/global_map",
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
}

# Terminal renk kodları
RED    = "\033[91m"
GREEN  = "\033[92m"
YELLOW = "\033[93m"
CYAN   = "\033[96m"
BOLD   = "\033[1m"
RESET  = "\033[0m"


# ─── Node ────────────────────────────────────────────────────────────────────

class TopicWatchdog(WatchdogMixin, Node):

    def __init__(self):
        Node.__init__(self, "diag_markerarray_node")
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

        # Başlangıç log mesajı
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
                sub = self.create_subscription(
                    MarkerArray,
                    topic,
                    self._make_callback(topic),
                    qos,
                )
                self._generic_subs.append(sub)
                self._subscribed_topics.add(topic)
                self.get_logger().debug(
                    f"[TopicWatchdog] Abone olundu: {topic} [{MSG_TYPE}]"
                )
            except Exception as exc:
                self.get_logger().warn(
                    f"[TopicWatchdog] Subscribe hatası ({topic}): {exc}"
                )

    # ── Callback ──────────────────────────────────────────────────────────────
    def _make_callback(self, topic: str):
        def _cb(_msg):
            with self._lock:
                was_error = self._error_active[topic]
                self._last_received[topic] = self.get_clock().now()
                if was_error:
                    self._error_active[topic] = False
                    # RECOVERED
                    self._log_warn(
                        f"{GREEN}{BOLD}[TopicWatchdog] RECOVERED{RESET}  "
                        f"'{topic}' yeniden yayın yapıyor."
                    )
        return _cb

    # ── Periyodik kontrol ─────────────────────────────────────────────────────
    def _check_all_topics(self):
        now = self.get_clock().now()
        with self._lock:
            for topic in TOPICS:
                last    = self._last_received[topic]
                elapsed = float("inf") if last is None else (now - last).nanoseconds / 1e9

                # Durum belirle
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

                # Dashboard'a durum gönder
                self.publish_status(topic, status, pub_elapse, MSG_TYPE, self.timeout)

                # Yeni hata açılırken log gönder
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
                            f"'{topic}'  ──  Subscribe edildi ama HİÇ mesaj gelmedi!"
                        )
                    else:
                        self._log_error(
                            f"{RED}{BOLD}[TopicWatchdog] ERROR [{ts}]{RESET}  "
                            f"'{topic}'  ──  {elapsed:.1f}s süredir mesaj yok "
                            f"(eşik: {self.timeout}s)"
                        )

    # ── Summary tetikleyici (10sn'de bir) ────────────────────────────────────
    def _request_summary(self):
        # Dashboard zaten anlık veriyi tutuyor; bu sinyal ile
        # summary tablosunu basmaya zorlarız
        self.publish_status("__summary__", "SUMMARY", None, MSG_TYPE, self.timeout)


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