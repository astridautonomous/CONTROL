#!/usr/bin/env python3
import json
from rclpy.qos import (
    QoSProfile, QoSReliabilityPolicy,
    QoSDurabilityPolicy, QoSHistoryPolicy,
)
from std_msgs.msg import String
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from builtin_interfaces.msg import Time

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
        self._diag_pub   = self.create_publisher(DiagnosticArray, "/diagnostics", _PUB_QOS)

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

    def publish_status(
        self,
        topic:    str,
        status:   str,
        elapsed,
        msg_type: str = "unknown",
        timeout:  float = 5.0,
    ):
        # Mevcut JSON publish — değişmedi
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

        # Foxglove için DiagnosticArray publish — YENİ
        level = self._status_to_diag_level(status)
        diag_msg = DiagnosticArray()
        diag_msg.header.stamp = self.get_clock().now().to_msg()

        s = DiagnosticStatus()
        s.level       = level
        s.name        = topic          # Foxglove'da başlık olarak görünür
        s.hardware_id = self.get_name()
        s.message     = self._status_to_message(status, elapsed, timeout)
        s.values      = [
            KeyValue(key="msg_type", value=str(msg_type)),
            KeyValue(key="timeout",  value=str(timeout)),
            KeyValue(key="elapsed",  value=str(elapsed) if elapsed is not None else "N/A"),
        ]

        diag_msg.status.append(s)
        self._diag_pub.publish(diag_msg)

    # ── Yardımcı metodlar ────────────────────────────────────────────────
    def _status_to_diag_level(self, status: str) -> int:
        if status == "OK":
            return DiagnosticStatus.OK      # 0
        elif status in ("TIMEOUT", "NO_MSG") or status.startswith("GNSS Error"):
            return DiagnosticStatus.ERROR   # 2
        elif status == "NO_PUBLISHER" or status.startswith("GNSS Warn"):
            return DiagnosticStatus.WARN    # 1
        else:
            return DiagnosticStatus.STALE   # 3

    def _status_to_message(self, status: str, elapsed, timeout: float) -> str:
        if status == "OK":
            return f"OK — {elapsed:.2f}s önce" if elapsed is not None else "OK"
        elif status == "TIMEOUT":
            return f"TIMEOUT — son mesaj {elapsed:.1f}s önce (limit: {timeout}s)"
        elif status == "NO_MSG":
            return "Hiç mesaj alınmadı"
        elif status == "NO_PUBLISHER":
            return "Publisher yok"
        else:
            return status