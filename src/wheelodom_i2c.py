#!/usr/bin/env python3
import rclpy 
from rclpy.node import Node 
from std_msgs.msg import Int8, Float32, Int16 
from geometry_msgs.msg import TwistWithCovarianceStamped, Quaternion 
from nav_msgs.msg import Odometry  
import time 
import struct 
import math 
import numpy as np 
import smbus2 
from threading import Lock

class I2CUnifiedBridge(Node):
    def __init__(self):
        super().__init__('astrid_i2c_unified_bridge')

        # I2C ayarları ve Kilit 
        self.bus = smbus2.SMBus(7) 
        self.address = 0x20 
        self.i2c_lock = Lock()

        # Yazma değişkenleri
        self.steer_cmd_left = 0 
        self.steer_cmd_right = 0 
        self.throttle_cmd = 0 
        self.brake_value = 1 

        # Okuma değişkenleri
        self.wheelbase = 1.8 
        self.dt = 0.02 

        # --- POSE HESAPLAMA BAŞLANGIÇ DEĞERLERİ ---
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0

        # --- EKF Calibration
        self.ekf_sync_interval = 10.0  # Kaç saniyede bir kalibre edileceği 
        self.last_ekf_sync_time = self.get_clock().now().nanoseconds / 1e9 # Başlangıç zamanı (saniye)

        # Aboneler ve Yayıncılar
        self.twist_pub = self.create_publisher(TwistWithCovarianceStamped, '/astrid/slam/twist', 10) 
        self.odom_test_pub = self.create_publisher(Odometry, '/astrid/slam/test_pose', 10)
       
        self.ekf_sub = self.create_subscription(Odometry, '/localization/pose_twist_fusion_filter/ekf_odom', self.ekf_callback, 10)
        
        self.subscription_brake = self.create_subscription(Int16, '/astrid/control/brake_cmd', self.brake_callback, 10) 
        self.subscription_steer = self.create_subscription(Int16, '/astrid/control/steer_cmd', self.steer_callback, 10)
        self.subscription_throttle = self.create_subscription(Int16, '/astrid/control/throttle_cmd', self.throttle_callback, 10)

        self.steer_deg_pub = self.create_publisher(Float32, '/astrid/control/steering', 10)

        self.timer = self.create_timer(self.dt, self.read_i2c_data)
        self.get_logger().info("Node başlatıldı, /astrid/control/brake_cmd, /astrid/control/throttle_cmd, steer_cmd ve EKF odom dinleniyor.") 

    # --- EKF KALİBRASYON FONKSİYONU ---
    def ekf_callback(self, msg):
        # Şu anki zamanı saniye cinsinden al
        current_time = self.get_clock().now().nanoseconds / 1e9
        
        # Belirlenen süre (ekf_sync_interval) geçtiyse kalibrasyon yap
        if (current_time - self.last_ekf_sync_time) >= self.ekf_sync_interval:
            
            # 1. EKF'den gelen kesin X ve Y konumlarını al
            new_x = msg.pose.pose.position.x
            new_y = msg.pose.pose.position.y
            
            # 2. EKF'den gelen Quaternion yönelimini Euler açısına (Yaw / Theta) çevir
            q = msg.pose.pose.orientation
            siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
            cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
            new_theta = math.atan2(siny_cosp, cosy_cosp)
            
            # 3. Kendi başlangıç değerlerimizi EKF'nin verisiyle ez
            self.x = new_x
            self.y = new_y
            self.theta = new_theta
            
            # 4. Son kalibrasyon zamanını güncelle
            self.last_ekf_sync_time = current_time
            
            # Konsola kalibrasyon bilgisini bas
            self.get_logger().warn(f"--- EKF KALİBRASYONU YAPILDI! New Initial -> X:{self.x:.2f}, Y:{self.y:.2f}, Yaw:{math.degrees(self.theta):.1f}° ---")

    # --- YAZMA FONKSİYONLARI  ---
    def steer_callback(self, msg):
        self.steer_cmd = msg.data 
        if self.steer_cmd < -255:
            self.steer_cmd = -255 
        if self.steer_cmd > 255: 
            self.steer_cmd = 255 

        if self.steer_cmd > 0: 
            self.steer_cmd_right = int(self.steer_cmd) 
            self.steer_cmd_left = 0 
        elif self.steer_cmd < 0: 
            self.steer_cmd_left = int(abs(self.steer_cmd)) 
            self.steer_cmd_right = 0 
        else: 
            self.steer_cmd_left = 0
            self.steer_cmd_right = 0 

    def throttle_callback(self, msg):
        self.throttle_cmd = msg.data 

    def brake_callback(self, msg):
        self.brake_value = msg.data 
        
        if self.brake_value == 1: 
            power_value = 0
            self.send_i2c(power_value, self.brake_value, self.steer_cmd_right, self.steer_cmd_left) 
        elif self.brake_value == 2 and self.throttle_cmd == 0: 
            power_value = 0 
            self.send_i2c(power_value, self.brake_value, self.steer_cmd_right, self.steer_cmd_left) 
        elif self.brake_value == 2 and self.throttle_cmd > 0: 
            power_value = self.throttle_cmd 
            self.send_i2c(power_value, self.brake_value, self.steer_cmd_right, self.steer_cmd_left) 
        else: 
            power_value = 0
            self.get_logger().warn(f"Tanımsız fren komutu alındı: {self.brake_value}") 
            
        # self.get_logger().info(f"power_value: {power_value}, brake_value: {self.brake_value}, steer_L: {self.steer_cmd_left}, steer_R: {self.steer_cmd_right}") 

    def send_i2c(self, value1, value2, value3, value4):
        try:
            packed_data = struct.pack('BBBB', value1, value2, value3, value4) 
            full_packet = list(packed_data) 
            
            with self.i2c_lock:
                self.bus.write_i2c_block_data(self.address, 0, full_packet) 
                
            # self.get_logger().info(f"I2C ile gönderildi: Paket: {full_packet}") 
        except Exception as e:
            self.get_logger().error(f"I2C gönderim hatası: {e}") 

    # --- OKUMA FONKSİYONLARI ---
    def read_i2c_data(self):
        try:
            with self.i2c_lock:
                msg = smbus2.i2c_msg.read(self.address, 13)
                self.bus.i2c_rdwr(msg) 
                data = list(msg) 

            if all(b == 0xFF for b in data) or all(b == 0x00 for b in data): 
                return 

            calc_checksum = 0 
            for i in range(12): 
                calc_checksum ^= data[i] 
            calc_checksum &= 0xFF 

            if calc_checksum != data[12]: 
                self.get_logger().warn(f"Checksum hatası! Alınan: {list(data)}") 
                return

            unpacked_data = struct.unpack('<ff', bytearray(data[0:8])) 
            v_x = unpacked_data[0] 
            steering_deg = unpacked_data[1] 
            deg_set_points = unpacked_data[2]

            self.get_logger().info(f"[I2C READ] v_x: {v_x:.3f} m/s | steering_deg: {steering_deg:.2f}° | set_points: {deg_set_points:.2f}")

            steer_msg = Float32()
            steer_msg.data = float(deg_set_points)
            self.steer_deg_pub.publish(steer_msg)

            steering_rad = math.radians(steering_deg) 
            v_yaw = (v_x / self.wheelbase) * math.tan(steering_rad) 
            
            self.publish_twist_and_pose(v_x, v_yaw) 

        except Exception as e:
            self.get_logger().error(f"I2C Okuma Hatası: {e}") 

    def publish_twist_and_pose(self, v_x, v_yaw):
        current_time = self.get_clock().now().to_msg()

        # ---------------------------------------------------------
        # 1. TWIST (HIZ) 
        # ---------------------------------------------------------
        base_variances = [0.06, 1e-5, 1e-5, 1e-5, 1e-5, 0.05] 
        cov_matrix = np.diag(base_variances) 

        twist_msg = TwistWithCovarianceStamped() 
        twist_msg.header.stamp = current_time 
        twist_msg.header.frame_id = 'base_link' 
        
        twist_msg.twist.twist.linear.x = float(v_x) 
        twist_msg.twist.twist.angular.z = float(v_yaw) 
        twist_msg.twist.covariance = cov_matrix.flatten().tolist() 
        
        self.twist_pub.publish(twist_msg) 

        # ---------------------------------------------------------
        # 2. POSE 
        # ---------------------------------------------------------
        
        # Kinematik Entegrasyon (Twist -> Pose)
        delta_s = v_x * self.dt
        delta_theta = v_yaw * self.dt

        # Midpoint Runge-Kutta Integrasyonu (Daha hassas yay çizer)
        theta_mid = self.theta + (delta_theta / 2.0)
        self.x += delta_s * math.cos(theta_mid)
        self.y += delta_s * math.sin(theta_mid)
        self.theta += delta_theta
        
        # Theta açısını -PI ile +PI arasına sabitleme
        self.theta = math.atan2(math.sin(self.theta), math.cos(self.theta))

        # Euler (Theta) -> Quaternion (Z ekseni etrafında dönüş)
        q = Quaternion()
        q.x = 0.0
        q.y = 0.0
        q.z = math.sin(self.theta / 2.0)
        q.w = math.cos(self.theta / 2.0)

        # Mesajı Doldurma
        odom_msg = Odometry()
        odom_msg.header.stamp = current_time
        odom_msg.header.frame_id = 'odom'
        odom_msg.child_frame_id = 'base_link'

        # Konum (Pose) Ataması
        odom_msg.pose.pose.position.x = self.x
        odom_msg.pose.pose.position.y = self.y
        odom_msg.pose.pose.position.z = 0.0
        odom_msg.pose.pose.orientation = q
        
        self.odom_test_pub.publish(odom_msg)

        # Test gözlemi için terminale log bas (Çok hızla akmasını istemezsen yoruma alabilirsin)
        self.get_logger().info(f"Made my own odom: X:{self.x:.2f}, Y:{self.y:.2f}, Yaw:{math.degrees(self.theta):.1f}°")

def main(args=None):
    rclpy.init(args=args) 
    node = I2CUnifiedBridge()
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