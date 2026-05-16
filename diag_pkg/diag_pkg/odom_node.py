#!/usr/bin/env python3
"""
========================
GNSS status değerine göre kullanılacak odometry(gnss/lidar) seçilir ve
seçilen odometry /astrid/slam/odometry topic'i üzerinden yayınlar.

seçim mantığı:
  status == 0  (FIX)      → /clap/ros/odometry   (GNSS odometry)
  status == -1 (NO_FIX)   → /ekf_odom            (LiDAR odometry)
  status ==  1 (SBAS_FIX) → /ekf_odom            (LiDAR odometry)
  status ==  2 (GBAS_FIX) → /ekf_odom            (LiDAR odometry)
"""

import threading

import rclpy
from rclpy.node import Node
from rclpy.qos import (
    QoSProfile,
    QoSReliabilityPolicy,
    QoSDurabilityPolicy,
    QoSHistoryPolicy,
)
from nav_msgs.msg import Odometry
from sensor_msgs.msg import NavSatFix

# ─── Sabitler ─────────────────────────────────────────────────────────────────

GPS_TOPIC        = "/clap/ros/gps_nav_sat_fix"
GNSS_ODOM_TOPIC  = "/clap/ros/odometry"
LIDAR_ODOM_TOPIC = "/ekf_odom"
OUTPUT_TOPIC     = "/astrid/slam/odometry"

# GNSS status değer açıklamaları (sensor_msgs/NavSatStatus)
GPS_STATUS_LABELS = {
    -1: "NO_FIX",
     0: "FIX",
     1: "SBAS_FIX",
     2: "GBAS_FIX",
}

# GNSS odometry kullanmak için geçerli (güvenilir) status değerleri
GNSS_OK_STATUSES = {0}          # Yalnızca tam FIX güvenilir kabul edilir
LIDAR_FALLBACK_STATUSES = {-1, 1, 2}  # Bu durumlarda LiDAR odom kullanılır

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

# Terminal renk kodları
RED    = "\033[91m"
GREEN  = "\033[92m"
YELLOW = "\033[93m"
CYAN   = "\033[96m"
BOLD   = "\033[1m"
RESET  = "\033[0m"


# ─── Node ─────────────────────────────────────────────────────────────────────

class LocalizationSwitcher(Node):
    """
    GNSS status'üne göre /clap/ros/odometry veya /ekf_odom
    kaynaklarından birini seçerek /astrid/slam/odometry'ye yayınlar.
    """

    def __init__(self):
        super().__init__("odom_node")

        # ── Parametreler ───────────────────────────────────────────────────────
        self.declare_parameter("verbose", True)

        self._verbose = self.get_parameter("verbose").value

        # ── Durum değişkenleri ─────────────────────────────────────────────────
        self._lock = threading.Lock()

        # Mevcut GNSS status değeri (None = henüz mesaj gelmedi)
        self._gps_status: int | None = None

        # Aktif kaynak: "gnss" | "lidar" | None
        self._active_source: str | None = None

        # Son seçim geçişini loglamak için önceki kaynak
        self._prev_source: str | None = None

        # ── Yayıncı ───────────────────────────────────────────────────────────
        self._pub = self.create_publisher(
            Odometry,
            OUTPUT_TOPIC,
            RELIABLE_QOS,
        )

        # ── Abonelikler ────────────────────────────────────────────────────────
        self._gps_sub = self.create_subscription(
            NavSatFix,
            GPS_TOPIC,
            self._gps_callback,
            SENSOR_QOS,
        )

        self._gnss_odom_sub = self.create_subscription(
            Odometry,
            GNSS_ODOM_TOPIC,
            self._gnss_odom_callback,
            SENSOR_QOS,
        )

        self._lidar_odom_sub = self.create_subscription(
            Odometry,
            LIDAR_ODOM_TOPIC,
            self._lidar_odom_callback,
            SENSOR_QOS,
        )

#        self.get_logger().info(
#            f"{CYAN}{BOLD}[LocalizationSwitcher]{RESET} Node başlatıldı.\n"
#            f"  GPS topic    : {GPS_TOPIC}\n"
#            f"  GNSS odom    : {GNSS_ODOM_TOPIC}\n"
#            f"  LiDAR odom   : {LIDAR_ODOM_TOPIC}\n"
#            f"  Çıkış topic  : {OUTPUT_TOPIC}"
#        )

    # ── GPS / GNSS status callback ─────────────────────────────────────────────

    def _gps_callback(self, msg: NavSatFix) -> None:
        """GNSS durum değişikliklerini izler ve aktif kaynağı günceller."""
        with self._lock:
            new_status = msg.status.status
            label      = GPS_STATUS_LABELS.get(new_status, f"UNKNOWN({new_status})")

            # Status değişmemişse gereksiz log üretme
            if new_status == self._gps_status:
                return

            self._gps_status = new_status

            # Yeni kaynağı belirle
            if new_status in GNSS_OK_STATUSES:
                self._active_source = "gnss"
            else:
                self._active_source = "lidar"

            # Kaynak değişimini logla
            if self._active_source != self._prev_source:
                self._log_source_change(new_status, label)
                self._prev_source = self._active_source

    # ── GNSS odometry callback ────────────────────────────────────────────────

    def _gnss_odom_callback(self, msg: Odometry) -> None:
        """
        /clap/ros/odometry mesajını alır.
        Aktif kaynak "gnss" ise yayınlar, değilse yoksayar.
        """
        with self._lock:
            if self._active_source != "gnss":
                return

        self._publish(msg, source="gnss")

    # ── LiDAR odometry callback ───────────────────────────────────────────────

    def _lidar_odom_callback(self, msg: Odometry) -> None:
        """
        /ekf_odom mesajını alır.
        Aktif kaynak "lidar" ise yayınlar, değilse yoksayar.
        """
        with self._lock:
            if self._active_source != "lidar":
                return

        self._publish(msg, source="lidar")

    # ── Yayın yardımcısı ──────────────────────────────────────────────────────

    def _publish(self, msg: Odometry, source: str) -> None:
        """
        frame_id'yi üzerine yazar ve /astrid/slam/odometry'ye gönderir.
        child_frame_id orijinal mesajdan korunur.
        """
        msg.header.frame_id = "odom"
        self._pub.publish(msg)

        if self._verbose:
            self.get_logger().debug(
                f"[LocalizationSwitcher] Yayınlandı ({source}) → {OUTPUT_TOPIC}"
            )

    # ── Log yardımcısı ────────────────────────────────────────────────────────

    def _log_source_change(self, status: int, label: str) -> None:
        """Kaynak geçişini renkli olarak loglar."""
        if self._active_source == "gnss":
            self.get_logger().info(
                f"{GREEN}{BOLD} GNSS ODOM seçildi{RESET}  "
                f"(status={status})  "
                f"kaynak: {GNSS_ODOM_TOPIC}"
            )
        else:
            severity = "ERROR" if status == -1 else "WARN"
            color    = RED if status == -1 else YELLOW
            self.get_logger().warn(
                f"{color}{BOLD} LiDAR ODOM'a geçildi [{severity}]{RESET}  "
                f"(status={status})  "
                f"kaynak: {LIDAR_ODOM_TOPIC}"
            )


# ─── Giriş noktası ────────────────────────────────────────────────────────────

def main(args=None):
    rclpy.init(args=args)
    node = LocalizationSwitcher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()