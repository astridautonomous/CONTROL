#!/usr.bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int8
import time
import struct
#import crc8
import smbus2

class BrakeCommandNode(Node):
    def __init__(self):
        super().__init__('brake_i2c_bridge')

        # I2C ayarları
        self.bus = smbus2.SMBus(7)  # Jetson AGX için I2C-7 (/dev/i2c-7)
        self.address = 0x20          # STM32 I2C slave adresi

        self.steer_cmd_left = 1
        self.steer_cmd_right = 1
        self.delayed_power_timer = None 
        
        self.subscription = self.create_subscription(
            Int8,
            '/astrid/control/brake_cmd',
            self.brake_callback,
            10
        )
        self.subscription = self.create_subscription(
            Int8,
            '/astrid/control/steer_cmd',
            self.steer_callback,
            10
        )
        self.get_logger().info("Node başlatıldı, /astrid/control/brake_cmd ve steer_cmd dinleniyor.")

    def steer_callback(self, msg):
        self.steer_cmd = msg.data
        if self.steer_cmd < 0:
            self.steer_cmd_left = abs(self.steer_cmd)
            self.steer_cmd_right = 1
        else:
            self.steer_cmd_right = self.steer_cmd
            self.steer_cmd_left = 1

    def brake_callback(self, msg):
        self.brake_value = msg.data
        
        # Eğer önceden ayarlanmış bir zamanlayıcı varsa, yeni komut geldiği için iptal et
        if self.delayed_power_timer:
            self.delayed_power_timer.cancel()
            self.get_logger().info("Mevcut zamanlayıcı iptal edildi.")

        # 2 salık  1 basık 
        if self.brake_value == 1:
            power_value = 1
            self.get_logger().info(f"Gönderilen brake_value: {self.brake_value}")
            self.get_logger().info(f"Gönderilen power_value: {power_value}")
            self.send_i2c(power_value, self.brake_value, self.steer_cmd_left, self.steer_cmd_right)

        elif self.brake_value == 2:
            # Önce power_value = 80 gönder
            power_value = 80
            self.get_logger().info(f"Gönderilen brake_value: {self.brake_value}")
            self.get_logger().info(f"İlk gönderilen power_value: {power_value}")
            self.send_i2c(power_value, self.brake_value, self.steer_cmd_left, self.steer_cmd_right)

            self.delayed_power_timer = self.create_timer(2.0, self.send_delayed_power_callback)
            self.get_logger().info("2 saniye sonra power_value=75 göndermek için zamanlayıcı kuruldu.")
            
        else:
             self.get_logger().warn(f"Tanımsız fren komutu alındı: {self.brake_value}")

    def send_delayed_power_callback(self):
        """Bu fonksiyon zamanlayıcı tarafından 2 saniye sonra çağrılır."""
        # power_value = 75 gönder
        delayed_power_value = 75
        self.get_logger().info(f"2 saniye geçti. Gecikmeli gönderilen power_value: {delayed_power_value}")
        
        self.send_i2c(delayed_power_value, self.brake_value, self.steer_cmd_left, self.steer_cmd_right)

        # Zamanlayıcı işini bitirdi, kendini imha etsin ve değişkeni sıfırla
        self.delayed_power_timer.cancel()
        self.delayed_power_timer = None


    def send_i2c(self, value1, value2, value3, value4):
        try:
            # 1 byte olarak paketle
            packed_data = struct.pack('BBBB', value1, value2, value3, value4)
            full_packet = list(packed_data) 

            # I2C üzerinden gönder
            self.bus.write_i2c_block_data(self.address, 0, full_packet)

            self.get_logger().info(f"I2C ile gönderildi: Paket: {full_packet} (power: {value1}, brake: {value2}, steer_L: {value3}, steer_R: {value4})")
        except Exception as e:
            self.get_logger().error(f"I2C gönderim hatası: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = BrakeCommandNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Node durduruldu (Ctrl+C).")
    finally:
        node.bus.close()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
