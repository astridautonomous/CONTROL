#!/usr/bin/env python3
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
        self.address = 0x20         # STM32 I2C slave adresi

        # /astrid/slam/brake_cmd topiğine abone ol
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
        self.get_logger().info("/astrid/control/brake_cmd topiğine abone olundu.")

    def steer_callback(self,msg):
        self.steer_cmd = msg.data
        if self.steer_cmd < 0:
            self.steer_cmd_left = abs(self.steer_cmd)
            self.steer_cmd_right = 1
        else:
            self.steer_cmd_right = self.steer_cmd
            self.steer_cmd_left = 1

    def brake_callback(self, msg):

        brake_value = msg.data

                
        if brake_value == 1:
            power_value = 1
        elif brake_value == 2:
            power_value = 78
            

                #2 salık  1 basık
        self.get_logger().info(f"Gelen fren komutu: {brake_value}")
        self.get_logger().info(f"gelen power verisi {power_value}")
        print(power_value,brake_value)

        self.send_i2c(power_value, brake_value, self.steer_cmd_left, self.steer_cmd_right)

    def send_i2c(self, value1, value2, value3, value4):
        try:
            # 1 byte olarak paketle
            packed_data = struct.pack('BBBB', value1, value2, value3, value4)

            # CRC hesapla (opsiyonel, pakete eklenmiyor)
            #hash_crc = crc8.crc8()
            #hash_crc.update(packed_data)
            #crc = hash_crc.digest()[0]

            full_packet = list(packed_data)  # İstersen + [crc] ekleyebilirsin

            # I2C üzerinden gönder
            self.bus.write_i2c_block_data(self.address, 0, full_packet)

            self.get_logger().info(f"I2C ile gönderildi: {full_packet} (değer: {value1, value2}")
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
