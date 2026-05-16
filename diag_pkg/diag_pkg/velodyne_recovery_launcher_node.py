#!/usr/bin/env python3
"""
=============================
/watchdog/status topic'ini dinler; /velodyne_points için hata durumu
algılandığında belirlenen Python betiğini subprocess ile başlatır.
Velodyne yeniden OK durumuna dönünce çalışan süreci sonlandırır.

Hata sayılan status değerleri : TIMEOUT | NO_MSG | NO_PUBLISHER
OK    sayılan status değeri   : OK
"""

import json
import subprocess
import threading

import rclpy
from rclpy.node import Node
from rclpy.qos import (
    QoSProfile,
    QoSReliabilityPolicy,
    QoSDurabilityPolicy,
    QoSHistoryPolicy,
)
from std_msgs.msg import String

# ─── Sabitler ─────────────────────────────────────────────────────────────────

WATCHDOG_STATUS_TOPIC = "/watchdog/status"
VELODYNE_TOPIC        = "/velodyne_points"

# !! Çalıştırılacak Python betiğinin tam yolunu buraya yapıştırın !!
RECOVERY_SCRIPT_PATH  = "/home/irmak/diag_ws/src/diag_pkg/diag_pkg/duremri.py"

# /watchdog/status mesajlarında hata olarak değerlendirilen status değerleri
ERROR_STATUSES = {"TIMEOUT", "NO_MSG", "NO_PUBLISHER"}

RELIABLE_QOS = QoSProfile(
    reliability=QoSReliabilityPolicy.RELIABLE,
    durability=QoSDurabilityPolicy.VOLATILE,
    history=QoSHistoryPolicy.KEEP_LAST,
    depth=100,
)

# Terminal renk kodları
RED    = "\033[91m"
GREEN  = "\033[92m"
YELLOW = "\033[93m"
CYAN   = "\033[96m"
BOLD   = "\033[1m"
RESET  = "\033[0m"


# ─── Node ─────────────────────────────────────────────────────────────────────

class VelodyneRecoveryLauncher(Node):
    """
    /watchdog/status mesajlarını izleyerek /velodyne_points hata aldığında
    RECOVERY_SCRIPT_PATH ile belirtilen Python betiğini başlatır.
    Topic OK durumuna dönünce çalışan betik sonlandırılır.
    """

    def __init__(self):
        super().__init__("velodyne_recovery_launcher_node")

        # ── Parametreler ───────────────────────────────────────────────────────
        self.declare_parameter("recovery_script", RECOVERY_SCRIPT_PATH)

        self._script_path: str = (
            self.get_parameter("recovery_script").value or RECOVERY_SCRIPT_PATH
        )

        # ── Durum değişkenleri ─────────────────────────────────────────────────
        self._lock           = threading.Lock()
        self._error_active   = False          # Velodyne şu an hatalı mı?
        self._process: subprocess.Popen | None = None

        # ── Abonelik ──────────────────────────────────────────────────────────
        self._status_sub = self.create_subscription(
            String,
            WATCHDOG_STATUS_TOPIC,
            self._status_callback,
            RELIABLE_QOS,
        )

        # ── Başlangıç kontrolü ────────────────────────────────────────────────
        if not self._script_path:
            self.get_logger().warn(
                f"{YELLOW}{BOLD}[VelodyneRecoveryLauncher]{RESET} "
                f"RECOVERY_SCRIPT_PATH henüz tanımlanmamış! "
                f"Hata algılandığında betik başlatılamaz."
            )

        self.get_logger().info(
            f"{CYAN}{BOLD}[VelodyneRecoveryLauncher]{RESET} Node başlatıldı.\n"
            f"  İzlenen topic  : {VELODYNE_TOPIC}\n"
            f"  Watchdog topic : {WATCHDOG_STATUS_TOPIC}\n"
            f"  Recovery betik : {self._script_path or '(tanımlanmadı)'}"
        )

    # ── Watchdog status callback ───────────────────────────────────────────────

    def _status_callback(self, msg: String) -> None:
        """
        /watchdog/status mesajlarını ayrıştırır.
        Yalnızca /velodyne_points ile ilgili mesajlar işlenir.
        """
        try:
            data = json.loads(msg.data)
        except json.JSONDecodeError:
            self.get_logger().warn(
                f"[VelodyneRecoveryLauncher] JSON ayrıştırma hatası: {msg.data}"
            )
            return

        topic  = data.get("topic",  "")
        status = data.get("status", "")

        # Yalnızca velodyne ve anlamlı status değerlerini işle
        if topic != VELODYNE_TOPIC or status == "SUMMARY":
            return

        with self._lock:
            if status in ERROR_STATUSES and not self._error_active:
                self._error_active = True
                elapsed = data.get("elapsed")
                self.get_logger().error(
                    f"{RED}{BOLD}[VelodyneRecoveryLauncher] HATA ALGILANDI{RESET}  "
                    f"'{VELODYNE_TOPIC}'  ──  status={status}"
                    + (f"  elapsed={elapsed:.1f}s" if isinstance(elapsed, float) else "")
                )
                self._start_recovery()

            elif status == "OK" and self._error_active:
                self._error_active = False
                self.get_logger().info(
                    f"{GREEN}{BOLD}[VelodyneRecoveryLauncher] RECOVERED{RESET}  "
                    f"'{VELODYNE_TOPIC}' yeniden OK durumuna döndü. "
                    f"Recovery betiği durduruluyor."
                )
                self._stop_recovery()

    # ── başlatma / durdurma ─────────────────────────────────────────────

    def _start_recovery(self) -> None:
        """Recovery betiğini subprocess olarak başlatır."""
        if not self._script_path:
            self.get_logger().error(
                f"{RED}{BOLD}[VelodyneRecoveryLauncher]{RESET} "
                f"Recovery betiği yolu tanımlanmamış, başlatılamıyor!"
            )
            return

        # Önceki süreç hâlâ çalışıyorsa tekrar başlatma
        if self._process is not None and self._process.poll() is None:
            self.get_logger().warn(
                f"{YELLOW}[VelodyneRecoveryLauncher]{RESET} "
                f"Recovery betiği zaten çalışıyor (PID={self._process.pid})."
            )
            return

        try:
            self._process = subprocess.Popen(
                ["python3", self._script_path],
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
            )
            self.get_logger().warn(
                f"{YELLOW}{BOLD}[VelodyneRecoveryLauncher]{RESET} "
                f"Recovery betiği başlatıldı → {self._script_path}  "
                f"(PID={self._process.pid})"
            )
        except FileNotFoundError:
            self.get_logger().error(
                f"{RED}{BOLD}[VelodyneRecoveryLauncher]{RESET} "
                f"Betik bulunamadı: {self._script_path}"
            )
        except Exception as exc:
            self.get_logger().error(
                f"{RED}{BOLD}[VelodyneRecoveryLauncher]{RESET} "
                f"Betik başlatma hatası: {exc}"
            )

    def _stop_recovery(self) -> None:
        """Çalışan recovery betiğini sonlandırır."""
        if self._process is None:
            return

        if self._process.poll() is None:
            self._process.terminate()
            try:
                self._process.wait(timeout=5.0)
                self.get_logger().info(
                    f"{GREEN}[VelodyneRecoveryLauncher]{RESET} "
                    f"Recovery betiği durduruldu (PID={self._process.pid})."
                )
            except subprocess.TimeoutExpired:
                self._process.kill()
                self.get_logger().warn(
                    f"{YELLOW}[VelodyneRecoveryLauncher]{RESET} "
                    f"Betik terminate olmadı, kill gönderildi (PID={self._process.pid})."
                )
        self._process = None

    # ── Node temizliği ────────────────────────────────────────────────────────

    def destroy_node(self):
        """Node kapanırken çalışan betiği temiz şekilde sonlandırır."""
        with self._lock:
            self._stop_recovery()
        super().destroy_node()


# ─── Giriş noktası ────────────────────────────────────────────────────────────

def main(args=None):
    rclpy.init(args=args)
    node = VelodyneRecoveryLauncher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()