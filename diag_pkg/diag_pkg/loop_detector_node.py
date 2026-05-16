#!/usr/bin/env python3
"""
/astrid/slam/current_lanelet_id topic'ini dinler; aynı lanelet ID'si
ardı ardına CONSECUTIVE_THRESHOLD kez yayınlanırsa
/astrid/control/loop_info topic'ine uyarı mesajı gönderir.
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
from std_msgs.msg import Int32, String

# ─── Sabitler ─────────────────────────────────────────────────────────────────

INPUT_TOPIC  = "/astrid/slam/current_lanelet_id"
OUTPUT_TOPIC = "/astrid/control/loop_info"
LOOP_MESSAGE = "Rota döngüye girdi"

# Kaç ardışık aynı ID → döngü sayılır
CONSECUTIVE_THRESHOLD = 3

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

class LoopDetector(Node):
    

    def __init__(self):
        super().__init__("loop_detector_node")

        # ── Parametreler ───────────────────────────────────────────────────────
        self.declare_parameter("consecutive_threshold", CONSECUTIVE_THRESHOLD)

        self._threshold: int = (
            self.get_parameter("consecutive_threshold").value
        )

        # ── Durum değişkenleri ─────────────────────────────────────────────────
        self._lock         = threading.Lock()
        self._prev_id: int | None = None   # Son görülen lanelet ID
        self._count:   int        = 0      # Ardışık aynı ID sayısı

        # ── Yayıncı ───────────────────────────────────────────────────────────
        self._loop_pub = self.create_publisher(
            String,
            OUTPUT_TOPIC,
            RELIABLE_QOS,
        )

        # ── Abonelik ──────────────────────────────────────────────────────────
        self._lanelet_sub = self.create_subscription(
            Int32,
            INPUT_TOPIC,
            self._lanelet_callback,
            RELIABLE_QOS,
        )

        self.get_logger().info(
            f"{CYAN}{BOLD}[LaneletLoopDetector]{RESET} Node başlatıldı.\n"
            f"  Giriş topic  : {INPUT_TOPIC}\n"
            f"  Çıkış topic  : {OUTPUT_TOPIC}\n"
            f"  Eşik         : {self._threshold} ardışık aynı ID"
        )

    # ── Lanelet ID callback ───────────────────────────────────────────────────

    def _lanelet_callback(self, msg: Int32) -> None:
        current_id = msg.data

        with self._lock:
            if current_id == self._prev_id:
                # Aynı ID tekrar geldi → sayacı artır
                self._count += 1
                self.get_logger().debug(
                    f"[LoopDetector] ID={current_id}  "
                    f"ardışık sayı={self._count}/{self._threshold}"
                )
            else:
                # Farklı ID → sayacı sıfırla, ID'yi güncelle
                self._prev_id = current_id
                self._count   = 1
                return

            # Eşiğe ulaşıldı mı?
            if self._count >= self._threshold:
                self._publish_loop_warning(current_id)
                # Bir sonraki ardışık grubu da yakalayabilmek için sayacı sıfırla
                self._count = 1

    # ── Uyarı yayıncısı ───────────────────────────────────────────────────────

    def _publish_loop_warning(self, lanelet_id: int) -> None:
        """Döngü uyarısını loglar ve topic'e yayınlar."""
        out_msg = String()
        out_msg.data = LOOP_MESSAGE
        self._loop_pub.publish(out_msg)

        self.get_logger().warn(
            f"{YELLOW}{BOLD} Döngü tespit edildi{RESET}  "
            f"Lanelet ID={lanelet_id} arka arkaya {self._threshold} kez geldi.  "
            f"→ '{OUTPUT_TOPIC}' : \"{LOOP_MESSAGE}\""
        )


# ─── Giriş noktası ────────────────────────────────────────────────────────────

def main(args=None):
    rclpy.init(args=args)
    node = LoopDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()