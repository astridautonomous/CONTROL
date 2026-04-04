#!/usr/bin/env python3
"""
Topic Watchdog Node — ROS 2 (rclpy)
=====================================================
"""
 
import rclpy
from rclpy.node import Node
from rclpy.qos import (
    QoSProfile, QoSReliabilityPolicy,
    QoSDurabilityPolicy, QoSHistoryPolicy,
)
import threading
from datetime import datetime
 
 
# ─────────────────────────────────────────────
#  İzlenecek topic listesi
# ─────────────────────────────────────────────
TOPICS = [
    "/astrid/slam/global_map",
]
 
# Terminal renk kodları
RED    = "\033[91m"
GREEN  = "\033[92m"
YELLOW = "\033[93m"
CYAN   = "\033[96m"
BOLD   = "\033[1m"
RESET  = "\033[0m"
 
# Best-effort QoS (sensör verisi için uygundur)
SENSOR_QOS = QoSProfile(
    reliability=QoSReliabilityPolicy.BEST_EFFORT,
    durability=QoSDurabilityPolicy.VOLATILE,
    history=QoSHistoryPolicy.KEEP_LAST,
    depth=10,
)
 
# Reliable QoS (komut ve harita topic'leri için)
RELIABLE_QOS = QoSProfile(
    reliability=QoSReliabilityPolicy.RELIABLE,
    durability=QoSDurabilityPolicy.VOLATILE,
    history=QoSHistoryPolicy.KEEP_LAST,
    depth=10,
)
 
# Sensor verisi sayılan topic'ler (best-effort QoS kullanılır)
SENSOR_TOPICS = {
    "/camera/camera/depth/img_rect_raw",
    "/camera/camera/color/image_raw",
    "/velodyne_points",
    "/scan",
    "/clap/ros/odometry",
    "/clap/odometry",
}
 
 
class TopicWatchdog(Node):
    def __init__(self):
        super().__init__("topic_watchdog")
 
        # Parametre tanımları
        self.declare_parameter("timeout",    5.0)
        self.declare_parameter("check_rate", 1.0)
 
        self.timeout    = self.get_parameter("timeout").value
        self.check_rate = self.get_parameter("check_rate").value
 
        self._lock              = threading.Lock()
        self._last_received     = {t: None  for t in TOPICS}
        self._error_active      = {t: False for t in TOPICS}
        self._subscribed_topics = set()
        self._generic_subs      = []   # referans tutmak için
 
        self.get_logger().info(
            f"{CYAN}{BOLD}[TopicWatchdog]{RESET} "
            f"{len(TOPICS)} topic izleniyor | "
            f"timeout={self.timeout}s | check_rate={self.check_rate}Hz"
        )
 
        # Topic tipi keşif timer'ı (2 sn'de bir kontrol)
        self._discovery_timer = self.create_timer(2.0, self._discover_and_subscribe)
 
        # Watchdog kontrol timer'ı
        self._watchdog_timer = self.create_timer(
            1.0 / self.check_rate,
            self._check_all_topics,
        )
 
        # Özet rapor timer'ı (10 sn'de bir)
        self._summary_timer = self.create_timer(10.0, self._print_summary)
 
    # ── Topic tipi otomatik keşfi ─────────────────────────────────────────
    def _discover_and_subscribe(self):
        topic_type_map: dict[str, list[str]] = dict(
            self.get_topic_names_and_types()
        )
 
        pending = [t for t in TOPICS if t not in self._subscribed_topics]
        if not pending:
            # Hepsi bağlandıysa keşif timer'ını durdur
            self._discovery_timer.cancel()
            self.get_logger().info(
                f"{GREEN}{BOLD}[TopicWatchdog]{RESET} "
                f"Tüm topic'lere subscribe olundu."
            )
            return
 
        for topic in pending:
            if topic not in topic_type_map:
                continue  # henüz yayınlanmıyor, bir sonraki turda tekrar dene
 
            types = topic_type_map[topic]
            if not types:
                continue
 
            from visualization_msgs.msg import MarkerArray
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
                    f"[TopicWatchdog] Abone olundu: {topic} [{MarkerArray}]"
                )
            except Exception as exc:
                self.get_logger().warn(
                    f"[TopicWatchdog] Subscribe hatası ({topic}): {exc}"
                )
 
    # ── Callback fabrikası ────────────────────────────────────────────────
    def _make_callback(self, topic: str):
        def _cb(_serialized_msg):
            with self._lock:
                was_error = self._error_active[topic]
                self._last_received[topic] = self.get_clock().now()
                if was_error:
                    self._error_active[topic] = False
                    self.get_logger().warn(
                        f"{GREEN}{BOLD}[TopicWatchdog] RECOVERED{RESET}  "
                        f"'{topic}' yeniden yayın yapıyor."
                    )
        return _cb
 
    # ── Periyodik kontrol ─────────────────────────────────────────────────
    def _check_all_topics(self):
        now = self.get_clock().now()
        with self._lock:
            for topic in TOPICS:
                last = self._last_received[topic]
 
                if last is None:
                    elapsed = float("inf")
                else:
                    elapsed = (now - last).nanoseconds / 1e9
 
                if elapsed > self.timeout:
                    if not self._error_active[topic]:
                        self._error_active[topic] = True
                        ts = datetime.now().strftime("%H:%M:%S")
 
                        if topic not in self._subscribed_topics:
                            self.get_logger().error(
                                f"{RED}{BOLD}[TopicWatchdog] ERROR [{ts}]{RESET}  "
                                f"'{topic}'  ──  Topic henüz ROS ağında bulunamadı! "
                                f"(publisher yok)"
                            )
                        elif last is None:
                            self.get_logger().error(
                                f"{RED}{BOLD}[TopicWatchdog] ERROR [{ts}]{RESET}  "
                                f"'{topic}'  ──  Subscribe edildi ama HİÇ mesaj gelmedi!"
                            )
                        else:
                            self.get_logger().error(
                                f"{RED}{BOLD}[TopicWatchdog] ERROR [{ts}]{RESET}  "
                                f"'{topic}'  ──  {elapsed:.1f}s süredir mesaj yok "
                                f"(eşik: {self.timeout}s)"
                            )
 
    # ── Özet raporu ────────────────────────────────────────────────────────
    def _print_summary(self):
        now = self.get_clock().now()
        lines = [
            f"\n{BOLD}{'─'*65}{RESET}",
            f"{BOLD}  Topic Watchdog — Anlık Durum{RESET}",
            f"{'─'*65}{RESET}",
        ]
        with self._lock:
            for topic in TOPICS:
                last = self._last_received[topic]
                if topic not in self._subscribed_topics:
                    status = f"{RED}⚠  Yayınlanmıyor{RESET}"
                elif last is None:
                    status = f"{RED}✖  Hiç mesaj gelmedi{RESET}"
                else:
                    elapsed = (now - last).nanoseconds / 1e9
                    if elapsed <= self.timeout:
                        status = f"{GREEN}✔  Son mesaj: {elapsed:.2f}s önce{RESET}"
                    else:
                        status = f"{RED}✖  Son mesaj: {elapsed:.1f}s önce (ZAMAN AŞIMI){RESET}"
                short = topic if len(topic) <= 42 else "…" + topic[-41:]
                lines.append(f"  {short:<43} {status}")
        lines.append(f"{BOLD}{'─'*65}{RESET}\n")
        print("\n".join(lines), flush=True)
 
 
# ─────────────────────────────────────────────
#  Giriş noktası
# ─────────────────────────────────────────────
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