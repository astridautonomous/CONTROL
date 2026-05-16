#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import (
    QoSProfile, QoSReliabilityPolicy,
    QoSDurabilityPolicy, QoSHistoryPolicy,
)
from std_msgs.msg import String
import json
import threading
import time

# ─── Terminal renk kodları ────────────────────────────────
RED    = "\033[91m"
GREEN  = "\033[92m"
YELLOW = "\033[93m"
CYAN   = "\033[96m"
BOLD   = "\033[1m"
RESET  = "\033[0m"

_SUB_QOS = QoSProfile(
    reliability=QoSReliabilityPolicy.RELIABLE,
    durability=QoSDurabilityPolicy.VOLATILE,
    history=QoSHistoryPolicy.KEEP_LAST,
    depth=100,
)


class DashboardNode(Node):

    def __init__(self):
        super().__init__("watchdog_dashboard")

        self._lock = threading.Lock()

        # { topic_key: { status, elapsed, msg_type, timeout, node, last_update } }
        self._status: dict[str, dict] = {}

        # Summary timer tetiklendiğinde True olur
        self._summary_due = False

        # Log subscriber
        self.create_subscription(String, "/watchdog/log",    self._on_log,    _SUB_QOS)
        # Status subscriber
        self.create_subscription(String, "/watchdog/status", self._on_status, _SUB_QOS)

        # Summary yazdırma timer'ı — 10 sn'de bir
        self.create_timer(10.0, self._print_summary)

        print(
            f"\n{CYAN}{BOLD}[Dashboard]{RESET} "
            f"Tüm watchdog node'ları dinleniyor...\n",
            flush=True,
        )

    # ── /watchdog/log handler ─────────────────────────────────────────────────
    def _on_log(self, msg: String):
        try:
            payload = json.loads(msg.data)
            level   = payload.get("level", "info")
            text    = payload.get("text", "")
            node    = payload.get("node", "?")

            # prefix
            prefix = {
                "info":  f"[INFO]  [{node}]",
                "warn":  f"{YELLOW}[WARN]{RESET}  [{node}]",
                "error": f"{RED}[ERROR]{RESET} [{node}]",
            }.get(level, f"[{level.upper()}] [{node}]")

            print(f"{prefix}: {text}", flush=True)
        except Exception as e:
            self.get_logger().warn(f"[Dashboard] Log parse hatası: {e}")

    # ── /watchdog/status handler ──────────────────────────────────────────────
    def _on_status(self, msg: String):
        try:
            payload = json.loads(msg.data)
            topic   = payload.get("topic", "")

            # Summary sinyali
            if topic == "__summary__":
                return  # summary timer zaten 10sn'de bir çalışıyor

            with self._lock:
                self._status[topic] = {
                    "node":        payload.get("node", "?"),
                    "status":      payload.get("status", "UNKNOWN"),
                    "elapsed":     payload.get("elapsed", None),
                    "msg_type":    payload.get("msg_type", "?"),
                    "timeout":     payload.get("timeout", 5.0),
                    "last_update": time.monotonic(),
                }
        except Exception as e:
            self.get_logger().warn(f"[Dashboard] Status parse hatası: {e}")

    # ── Özet tablo ─────────
    def _print_summary(self):
        with self._lock:
            snapshot = dict(self._status)

        lines = [
            f"\n{BOLD}{'─'*65}{RESET}",
            f"{BOLD}  Topic Watchdog — Anlık Durum{RESET}",
            f"{'─'*65}{RESET}",
        ]

        # Topic'leri node adına göre grupla, sonra topic adına göre sırala
        for topic in sorted(snapshot.keys()):
            info    = snapshot[topic]
            status  = info["status"]
            elapsed = info["elapsed"]
            timeout = info["timeout"]

            if status == "NO_PUBLISHER":
                status_str = f"{RED}⚠  Yayınlanmıyor{RESET}"
            elif status == "NO_MSG" or elapsed == -1:
                status_str = f"{RED}✖  Hiç mesaj gelmedi{RESET}"
            elif status == "OK":
                status_str = f"{GREEN}✔  Son mesaj: {elapsed:.2f}s önce{RESET}"
            elif status == "TIMEOUT":
                status_str = f"{RED}✖  Son mesaj: {elapsed:.1f}s önce (ZAMAN AŞIMI){RESET}"
            elif status.startswith("GNSS Error"):
                status_str = f"{RED}✖  {status}{RESET}"
            elif status.startswith("GNSS Warn"):
                status_str = f"{YELLOW}⚠  {status}{RESET}"
            else:
                status_str = f"{YELLOW}?  {status}{RESET}"

            short = topic if len(topic) <= 42 else "…" + topic[-41:]
            lines.append(f"  {short:<43} {status_str}")

        lines.append(f"{BOLD}{'─'*65}{RESET}\n")
        print("\n".join(lines), flush=True)


# ─── Giriş noktası ───────────────────────────────────────────────────────────

def main(args=None):
    rclpy.init(args=args)
    node = DashboardNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()