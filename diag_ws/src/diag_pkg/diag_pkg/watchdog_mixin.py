#!/usr/bin/env python3
"""
=================
Her diag_* node'una eklenir.

  - _log_info / _log_warn / _log_error  →  /watchdog/log    topic'ine JSON publish eder
  - publish_status()                    →  /watchdog/status topic'ine JSON publish eder

"""

import json
from rclpy.qos import (
    QoSProfile, QoSReliabilityPolicy,
    QoSDurabilityPolicy, QoSHistoryPolicy,
)
from std_msgs.msg import String

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

    # ── Log yayınlayıcılar ────────────────────────────────────────────────
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

    # ── Durum yayınlayıcı ─────────────────────────────────────────────────
    def publish_status(
        self,
        topic:    str,
        status:   str,
        elapsed,
        msg_type: str = "unknown",
        timeout:  float = 5.0,
    ):
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