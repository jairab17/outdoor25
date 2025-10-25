#!/usr/bin/env python3
import rclpy
import time
import os
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from px4_msgs.msg import VehicleGlobalPosition
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from PIL import Image as PILImage
import piexif
import cv2
from datetime import datetime

class TakePhotosNode(Node):
    def __init__(self):
        super().__init__('take_photos_node')
        self.bridge = CvBridge()
        self.current_lat = None
        self.current_lon = None
        self.current_alt = None
        self.last_saved_time = 0.0
        self.save_interval = 2.0  # segundos entre fotos
        self.photo_folder = "/home/jair17/images"
        os.makedirs(self.photo_folder, exist_ok=True)

        # QoS compatible con PX4
        qos_gps = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        # Suscripción al GPS
        self.gps_sub = self.create_subscription(
            VehicleGlobalPosition,
            '/fmu/out/vehicle_global_position',
            self.gps_callback,
            qos_gps
        )

        # Suscripción a la cámara
        self.image_sub = self.create_subscription(
            Image,
            '/pi_camera/image_raw',  # tópico simulado
            self.image_callback,
            10
        )

    def gps_callback(self, msg):
        self.current_lat = msg.lat
        self.current_lon = msg.lon
        self.current_alt = msg.alt

    def to_deg(self, value):
        """Convert decimal degrees to degrees, minutes, seconds tuple"""
        d = int(value)
        m = int((value - d) * 60)
        s = int(((value - d) * 60 - m) * 60)
        return ((d,1),(m,1),(s,1))

    def image_callback(self, data):
        if self.current_lat is None:
            self.get_logger().warn("Esperando datos del GPS antes de guardar imagen...")
            return

        current_time = time.time()
        if current_time - self.last_saved_time >= self.save_interval:
            frame = self.bridge.imgmsg_to_cv2(data, "bgr8")
            timestamp = datetime.now().strftime("%Y%m%d_%H%M%S_%f")
            filename = os.path.join(self.photo_folder, f"image_{timestamp}.jpg")

            # Convertir a RGB y crear PIL image
            frame_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
            pil_img = PILImage.fromarray(frame_rgb)

            # Preparar GPS en EXIF
            lat_ref = 'N' if self.current_lat >= 0 else 'S'
            lon_ref = 'E' if self.current_lon >= 0 else 'W'
            exif_dict = {"GPS": {}}
            exif_dict["GPS"][piexif.GPSIFD.GPSLatitudeRef] = lat_ref
            exif_dict["GPS"][piexif.GPSIFD.GPSLatitude] = self.to_deg(abs(self.current_lat))
            exif_dict["GPS"][piexif.GPSIFD.GPSLongitudeRef] = lon_ref
            exif_dict["GPS"][piexif.GPSIFD.GPSLongitude] = self.to_deg(abs(self.current_lon))
            exif_dict["GPS"][piexif.GPSIFD.GPSAltitude] = (int(self.current_alt*100), 100)
            exif_bytes = piexif.dump(exif_dict)

            # Guardar JPEG con EXIF
            pil_img.save(filename, "jpeg", exif=exif_bytes)
            self.get_logger().info(f"[OK] Imagen guardada con GPS en EXIF: {filename}")

            self.last_saved_time = current_time

def main(args=None):
    rclpy.init(args=args)
    node = TakePhotosNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
