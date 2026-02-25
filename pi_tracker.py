import cv2
import sys
import time
import json
import csv
import math
import os
from datetime import datetime
from ultralytics import YOLO
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class PiTracker(Node):
    def __init__(self):
        super().__init__('pi_tracker_node')
        self.publisher_ = self.create_publisher(String, 'tracking_data', 10)
        self.timer = self.create_timer(0.1, self.track_objects)
        self.camera_index = 0
        self.model_path = 'yolov8n.pt'
        self.cap = cv2.VideoCapture(self.camera_index, cv2.CAP_AVFOUNDATION)
        self.width, self.height = 1280, 720
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.width)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.height)
        self.model = YOLO(self.model_path)
        self.frame_id = 0
        self.locked_track_id = None
        self.log_filename = self.initialize_logging()

    def initialize_logging(self):
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        log_filename = f"tracker_log_{timestamp}.csv"
        with open(log_filename, 'w', newline='') as f:
            writer = csv.writer(f)
            writer.writerow(["Timestamp", "Frame_ID", "Target_Class", "Center_X", "Center_Y", "Target_X", "Target_Y", "Vector_X", "Vector_Y"])
        return log_filename

    def track_objects(self):
        ret, frame = self.cap.read()
        if not ret:
            self.get_logger().error("Frame capture failed")
            return
        
        self.frame_id += 1
        results = self.model.track(frame, persist=True, verbose=False)
        detections = self.parse_detections(results)

        target_obj = self.determine_target(detections)

        if target_obj:
            self.log_tracking_data(target_obj)
            self.publish_tracking_data(target_obj)

    def parse_detections(self, results):
        detections = []
        if results[0].boxes and results[0].boxes.id is not None:
            boxes = results[0].boxes
            track_ids = boxes.id.int().cpu().tolist()
            for box, track_id in zip(boxes, track_ids):
                x1, y1, x2, y2 = box.xyxy[0].cpu().numpy()
                cx = int((x1 + x2) / 2)
                cy = int((y1 + y2) / 2)
                dist = math.sqrt((cx - self.width // 2)**2 + (cy - self.height // 2)**2)
                detections.append({
                    "id": track_id,
                    "bbox": (int(x1), int(y1), int(x2), int(y2)),
                    "center": (cx, cy),
                    "dist": dist,
                    "name": self.model.names[int(box.cls[0])],
                    "conf": float(box.conf[0])
                })
        return detections

    def determine_target(self, detections):
        if self.locked_track_id is not None:
            return next((d for d in detections if d["id"] == self.locked_track_id), None)
        else:
            if detections:
                detections.sort(key=lambda x: x["dist"])
                return detections[0]
        return None

    def log_tracking_data(self, target_obj):
        with open(self.log_filename, 'a', newline='') as f:
            writer = csv.writer(f)
            writer.writerow([
                datetime.now().isoformat(), self.frame_id, target_obj["name"],
                self.width // 2, self.height // 2, target_obj['center'][0], target_obj['center'][1],
                target_obj['center'][0] - (self.width // 2), target_obj['center'][1] - (self.height // 2)
            ])

    def publish_tracking_data(self, target_obj):
        data_packet = {
            "frame": self.frame_id,
            "target_id": target_obj['id'],
            "name": target_obj['name'],
            "center": target_obj['center']
        }
        msg = String()
        msg.data = json.dumps(data_packet)
        self.publisher_.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    pi_tracker = PiTracker()
    rclpy.spin(pi_tracker)
    pi_tracker.cap.release()
    rclpy.shutdown()

if __name__ == "__main__":
    main()