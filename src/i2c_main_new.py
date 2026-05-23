#!/usr.bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int8, Float32, Int16
import time
import struct
#import crc8
import smbus2

class BrakeCommandNode(Node):
    def __init__(self):
        super().__init__('brake_i2c_bridge')

        # I2C ayarları
        self.bus = smbus2.SMBus(7)  # Jetson AGX için I2C-7 (/dev/i2c-7)
        self.address = 0x20  # STM32 I2C slave adresi

        self.steer_cmd_left = 0
        self.steer_cmd_right = 0
        
        self.subscription = self.create_subscription(
            Int16,
            '/astrid/control/brake_cmd',
            self.brake_callback,
            10
        )
        self.subscription = self.create_subscription(
            Int16,
            '/astrid/control/steer_cmd',
            self.steer_callback,
            10
        )
   
   
        self.get_logger().info("Node başlatıldı, /astrid/control/brake_cmd ve steer_cmd dinleniyor.")

    def steer_callback(self, msg):
        self.steer_cmd = msg.data
        if self.steer_cmd < -255:
            self.steer_cmd = -255  
        if self.steer_cmd > 255:
            self.steer_cmd = 255 

        if self.steer_cmd < 0:
            self.steer_cmd_right = int(abs(self.steer_cmd))
            self.steer_cmd_left = 0
        elif self.steer_cmd > 0:
            self.steer_cmd_left = int(self.steer_cmd)
            self.steer_cmd_right = 0
        else:
            self.steer_cmd_left = 0
            self.steer_cmd_right = 0
            
        self.get_logger().info(f"Gönderilen steer_cmd: {self.steer_cmd}")
        #self.get_logger().info(f"Gönderilen steer_cmd_left: {self.steer_cmd_left}, steer_cmd_right: {self.steer_cmd_right}")

    def brake_callback(self, msg):
        self.brake_value = msg.data
        
        # 1 salık  2 basık  3 rölanti
        if self.brake_value == 2:
            power_value = 0
            #self.get_logger().info(f"Gönderilen brake_value: {self.brake_value}")
            #self.get_logger().info(f"Gönderilen power_value: {power_value}")
            self.send_i2c(power_value, self.brake_value, self.steer_cmd_right, self.steer_cmd_left)
        elif self.brake_value == 1:
            # Önce power_value = 80 gönder
            timer = []
            power_value = 50

            self.send_i2c(power_value, self.brake_value, self.steer_cmd_right, self.steer_cmd_left)
        else:
            self.get_logger().warn(f"Tanımsız fren komutu alındı: {self.brake_value}")
        self.get_logger().info(f"power_value: {power_value}, brake_value: {self.brake_value}, steer_L: {self.steer_cmd_left}, steer_R: {self.steer_cmd_right}")

        
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