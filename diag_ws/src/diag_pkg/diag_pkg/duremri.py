#!/usr/bin/env python3

import cv2
import numpy as np
from ultralytics import YOLO
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class ObjectDetectionNode(Node):
    def __init__(self):
        super().__init__('object_detection_node')
        self.publisher_ = self.create_publisher(String, 'astrid/perception/object_detection', 10)
        
        # Modeli yükle
        self.model = YOLO("yolov8n.pt")
        
        # Video dosyasını aç (Dosya yolunun doğruluğundan emin olun!)
        self.cap = cv2.VideoCapture("video.mp4")
        
        if not self.cap.isOpened():
            self.get_logger().error("Hata: Video dosyası açılamadı!")
            return

        # 10 FPS hızında çalışacak bir timer (0.1 saniye)
        self.timer = self.create_timer(0.1, self.process_frame)

    def process_frame(self):
        ret, frame = self.cap.read()
        
        if not ret:
            self.get_logger().info("Video sona erdi veya okunamıyor.")
            self.timer.cancel() # Video biterse timer'ı durdur
            return

        height, width, _ = frame.shape
        
        # --- ROI AYARLARI ---
        ROI_UST_ORAN, ROI_ALT_ORAN = 0.20, 0.85
        ROI_SOL_ORAN, ROI_SAG_ORAN = 0.38, 0.62
        DURMA_MESAFESI = 1500

        x_start, y_start = int(width * ROI_SOL_ORAN), int(height * ROI_UST_ORAN)
        x_end, y_end = int(width * ROI_SAG_ORAN), int(height * ROI_ALT_ORAN)

        # Modeli Çalıştır
        results = self.model(frame, verbose=False, classes=[0, 2, 5, 7])

        # Referans ROI Çizimi
        cv2.rectangle(frame, (x_start, y_start), (x_end, y_end), (255, 255, 0), 2)

        dur_emri_aktif = False
        for r in results:
            for box in r.boxes:
                x1, y1, x2, y2 = map(int, box.xyxy[0])
                merkez_x, merkez_y = int((x1 + x2) / 2), int((y1 + y2) / 2)
                h = y2 - y1
                dist = 100000 / (h + 1)

                if x_start < merkez_x < x_end and y_start < merkez_y < y_end:
                    if dist < DURMA_MESAFESI:
                        cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 0, 255), 3)
                        dur_emri_aktif = True
                    else:
                        cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)

        if dur_emri_aktif:
            msg = String()
            msg.data = "dur"
            self.publisher_.publish(msg)
            self.get_logger().info("Yayınlandı: 'dur'")

        # Görüntüleme
        cv2.imshow('Analiz', frame)
        if cv2.waitKey(1) & 0xFF == ord('q'): # 'q' ile çıkış yapabilirsin
            self.cap.release()
            cv2.destroyAllWindows()
            rclpy.shutdown()

def main(args=None):
    rclpy.init(args=args)
    node = ObjectDetectionNode()
    
    try:
        # Node'u canlı tutmak için spin kullanıyoruz
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.cap.release()
        cv2.destroyAllWindows()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()