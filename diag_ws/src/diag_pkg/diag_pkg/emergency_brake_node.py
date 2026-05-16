#!/usr/bin/env python3
"""
=================
Emergency Brake Node

Belirli topic'leri izler (timeout ile).
Herhangi bir topic'ten mesaj gelmezse acil durum freni komutu (0) yayınlar.
WatchdogMixin kullanarak durum takip eder.

Grup mantığı:
  - critical_topics   : Herhangi biri timeout → fren tetiklenir.
  - sensor_group_topics : Gruptaki tüm topicler aynı anda timeout → fren tetiklenir.
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import (
    QoSProfile, QoSReliabilityPolicy,
    QoSDurabilityPolicy, QoSHistoryPolicy,
)

from sensor_msgs.msg import NavSatFix, PointCloud2, Image
from visualization_msgs.msg import MarkerArray
from nav_msgs.msg import Path, Odometry
from std_msgs.msg import Int8

import time
from typing import Dict
import sys
import os

# WatchdogMixin'i ekle
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', '..', 'diag_pkg'))

try:
    from watchdog_mixin import WatchdogMixin
except ImportError:
    print("⚠  WatchdogMixin bulunamadı. Yerel WatchdogMixin tanımı kullanılıyor...")
    # Fallback: WatchdogMixin yerel tanımı
    from std_msgs.msg import String
    import json
    
    _PUB_QOS = QoSProfile(
        reliability=QoSReliabilityPolicy.RELIABLE,
        durability=QoSDurabilityPolicy.VOLATILE,
        history=QoSHistoryPolicy.KEEP_LAST,
        depth=100,
    )
    
    class WatchdogMixin:
        def __init__(self):
            self._log_pub    = self.create_publisher(String, "/watchdog/log",    _PUB_QOS)
            self._status_pub = self.create_publisher(String, "/watchdog/status", _PUB_QOS)

        def _log_info(self, text: str):
            self._publish_log("info", text)

        def _log_warn(self, text: str):
            self._publish_log("warn", text)

        def _log_error(self, text: str):
            self._publish_log("error", text)

        def _publish_log(self, level: str, text: str):
            msg = String()
            msg.data = json.dumps({
                "node":  self.get_name(),
                "level": level,
                "text":  text,
            })
            self._log_pub.publish(msg)

        def publish_status(self, topic: str, status: str, elapsed, msg_type: str = "unknown", timeout: float = 5.0):
            msg = String()
            msg.data = json.dumps({
                "node":     self.get_name(),
                "topic":    topic,
                "status":   status,
                "elapsed":  elapsed,
                "msg_type": msg_type,
                "timeout":  timeout,
            })
            self._status_pub.publish(msg)


# QoS ayarları
_SUB_QOS = QoSProfile(
    reliability=QoSReliabilityPolicy.BEST_EFFORT,
    durability=QoSDurabilityPolicy.VOLATILE,
    history=QoSHistoryPolicy.KEEP_LAST,
    depth=5,
)

_PUB_QOS = QoSProfile(
    reliability=QoSReliabilityPolicy.RELIABLE,
    durability=QoSDurabilityPolicy.VOLATILE,
    history=QoSHistoryPolicy.KEEP_LAST,
    depth=10,
)


class EmergencyBrakeNode(Node, WatchdogMixin):
    def __init__(self):
        Node.__init__(self, 'emergency_brake_node')
        WatchdogMixin.__init__(self)

        # === PARAMETRELER ===
        self.declare_parameter('timeout', 2.0)  # Topic timeout (saniye)
        self.timeout = self.get_parameter('timeout').value

        # === Kritik Topicler: Herhangi biri timeout → fren tetiklenir ===
        self.critical_topics = {
            "/astrid/slam/global_map": {
                "msg_type": MarkerArray,
                "last_msg_time": None,
                "status": "TIMEOUT",
            },
            "/astrid/navigation/fusion_path": {
                "msg_type": Path,
                "last_msg_time": None,
                "status": "TIMEOUT",
            },
            "/astrid/slam/odometry": {
                "msg_type": Odometry,
                "last_msg_time": None,
                "status": "TIMEOUT",
            },
        }

        # === Sensör Grubu: tüm topicler aynı anda timeout → fren tetiklenir ===
        self.sensor_group_topics = {
            "/velodyne_points": {
                "msg_type": PointCloud2,
                "last_msg_time": None,
                "status": "TIMEOUT",
            },
            "/camera/camera/color/image_raw": {
                "msg_type": Image,
                "last_msg_time": None,
                "status": "TIMEOUT",
            },
            "/camera/camera/depth/img_rect_raw": {
                "msg_type": Image,
                "last_msg_time": None,
                "status": "TIMEOUT",
            },
        }

        # === Acil Durum Freni Publisher ===
        self.emergency_brake_pub = self.create_publisher(
            Int8,
            '/astrid/control/emergency_brake',
            _PUB_QOS
        )

        # === Subscriptions: critical_topics ===
        for topic_name, topic_info in self.critical_topics.items():
            self.create_subscription(
                topic_info["msg_type"],
                topic_name,
                lambda msg, tn=topic_name: self._on_topic_received(msg, tn, self.critical_topics),
                _SUB_QOS
            )

        # === Subscriptions: sensor_group_topics ===
        for topic_name, topic_info in self.sensor_group_topics.items():
            self.create_subscription(
                topic_info["msg_type"],
                topic_name,
                lambda msg, tn=topic_name: self._on_topic_received(msg, tn, self.sensor_group_topics),
                _SUB_QOS
            )

        # === Timer: Topic durumunu kontrol et (0.5 s) ===
        self.create_timer(0.5, self._check_topics_health)

        # === Başlangıç ===
        self.emergency_brake_active = False
        self.get_logger().info(
            f"✔ Emergency Brake Node başlatıldı (timeout: {self.timeout}s)\n"
            f"   Kritik topicler (herhangi biri): {list(self.critical_topics.keys())}\n"
            f"   Sensör grubu (hepsi birden)    : {list(self.sensor_group_topics.keys())}"
        )
        self._log_info(f"Emergency Brake Node başlatıldı. Timeout: {self.timeout}s")

    # ──────────────────────────────────────────────────────────────────────
    # Topic mesajı alındığında
    # ──────────────────────────────────────────────────────────────────────
    def _on_topic_received(self, msg, topic_name: str, topic_dict: dict):
        """Topic'ten mesaj alındığında ilgili dict'teki zaman damgasını güncelle."""
        current_time = time.time()
        topic_dict[topic_name]["last_msg_time"] = current_time
        topic_dict[topic_name]["status"] = "OK"

    # ──────────────────────────────────────────────────────────────────────
    # Timer callback: Topic sağlığını kontrol et
    # ──────────────────────────────────────────────────────────────────────
    def _check_topics_health(self):
        """Tüm topic gruplarının sağlığını kontrol et."""
        current_time = time.time()
        failed_topics = []

        # ── 1) Kritik topicler: herhangi biri timeout → fren ──────────────
        for topic_name, topic_info in self.critical_topics.items():
            if topic_info["last_msg_time"] is None:
                elapsed = "N/A"
                topic_info["status"] = "TIMEOUT"
                failed_topics.append(topic_name)
            else:
                elapsed_time = current_time - topic_info["last_msg_time"]
                elapsed = f"{elapsed_time:.2f}s"

                if elapsed_time > self.timeout:
                    topic_info["status"] = "TIMEOUT"
                    failed_topics.append(topic_name)
                else:
                    topic_info["status"] = "OK"

            self.publish_status(
                topic=topic_name,
                status=topic_info["status"],
                elapsed=elapsed,
                msg_type=topic_info["msg_type"].__name__,
                timeout=self.timeout,
            )

        # ── 2) Sensör grubu: tüm topicler timeout ise → fren ──────────────
        sensor_group_failed = []
        for topic_name, topic_info in self.sensor_group_topics.items():
            if topic_info["last_msg_time"] is None:
                elapsed = "N/A"
                topic_info["status"] = "TIMEOUT"
                sensor_group_failed.append(topic_name)
            else:
                elapsed_time = current_time - topic_info["last_msg_time"]
                elapsed = f"{elapsed_time:.2f}s"

                if elapsed_time > self.timeout:
                    topic_info["status"] = "TIMEOUT"
                    sensor_group_failed.append(topic_name)
                else:
                    topic_info["status"] = "OK"

            self.publish_status(
                topic=topic_name,
                status=topic_info["status"],
                elapsed=elapsed,
                msg_type=topic_info["msg_type"].__name__,
                timeout=self.timeout,
            )

        # Sensör grubundaki tüm topicler aynı anda failed ise fren listesine ekle
        if len(sensor_group_failed) == len(self.sensor_group_topics):
            failed_topics.extend(sensor_group_failed)

        #self.print_status_summary()

        # ────────────────────────────────────────────────────────────────
        # Acil Durum Freni Tetikleme
        # ────────────────────────────────────────────────────────────────
        if failed_topics:
            # En az bir koşul sağlandı → fren
            if not self.emergency_brake_active:
                # İlk kez failed → Log et
                self._log_error(f" Acil durum freni tetiklendi. Failed topicler: {', '.join(failed_topics)}")
                self.get_logger().error(
                    f"\n{'='*70}\n"
                    f" Acil durum freni tetiklendi!\n"
                    f" Error veren topicler:\n"
                    f"{chr(10).join(['  ✖ ' + t for t in failed_topics])}\n"
                    f"{'='*70}"
                )
                self.emergency_brake_active = True

            # 0 gönder (acil durum freni)
            brake_msg = Int8()
            brake_msg.data = 0
            self.emergency_brake_pub.publish(brake_msg)

        else:
            # Tüm topicler sağlıklı
            if self.emergency_brake_active:
                # Freni kaldır
                self._log_info("✔ Tüm critical topicler normal. Acil durum freni kaldırılıyor...")
                self.get_logger().info("✔ Tüm critical topicler normal. Acil durum freni kaldırılıyor...")
                self.emergency_brake_active = False

    # ──────────────────────────────────────────────────────────────────────
    # Durum özeti (terminal output)
    # ──────────────────────────────────────────────────────────────────────
    def print_status_summary(self):
        """Durum özetini ekrana yazdır."""
        self.get_logger().info("\n" + "="*70)
        self.get_logger().info(" Emergency Brake Node - Durum Özeti")
        self.get_logger().info("="*70)

        self.get_logger().info("[ Kritik Topicler ]")
        for topic_name, topic_info in self.critical_topics.items():
            status_symbol = "✔" if topic_info["status"] == "OK" else "✖"
            self.get_logger().info(f"{status_symbol} {topic_name}: {topic_info['status']}")

        self.get_logger().info("[ Sensör Grubu (hepsi birden) ]")
        for topic_name, topic_info in self.sensor_group_topics.items():
            status_symbol = "✔" if topic_info["status"] == "OK" else "✖"
            self.get_logger().info(f"{status_symbol} {topic_name}: {topic_info['status']}")

        self.get_logger().info("="*70 + "\n")


def main(args=None):
    rclpy.init(args=args)
    node = EmergencyBrakeNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info(" Node durduruldu (Ctrl+C).")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()