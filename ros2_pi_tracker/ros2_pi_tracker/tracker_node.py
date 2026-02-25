import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
import cv2
import numpy as np
from ultralytics import YOLO
import os
import csv
from datetime import datetime

class PiTrackerNode(Node):
    def __init__(self):
        super().__init__('pi_tracker_node')
        
        # Initialize parameters
        self.camera_index = 0
        self.model_path = 'yolov8n.pt'
        self.locked_track_id = None
        self.frame_id = 0
        
        # Initialize logging
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        self.log_filename = f"tracker_log_{timestamp}.csv"
        with open(self.log_filename, 'w', newline='') as f:
            writer = csv.writer(f)
            writer.writerow(["Timestamp", "Frame_ID", "Target_Class", "Center_X", "Center_Y", "Target_X", "Target_Y", "Vector_X", "Vector_Y"])
        
        # Load YOLO model
        if not os.path.exists(self.model_path):
            self.get_logger().error(f"Model file '{self.model_path}' not found locally.")
            return
        self.model = YOLO(self.model_path)

        # Create a subscriber for camera feed
        self.create_subscription(Image, 'camera/image', self.camera_callback, 10)
        
        # Create a publisher for tracking results
        self.tracking_pub = self.create_publisher(String, 'tracking/results', 10)

    def camera_callback(self, msg):
        # Convert ROS Image message to OpenCV format
        frame = self.ros_image_to_cv2(msg)
        self.frame_id += 1
        
        # Perform tracking
        results = self.model.track(frame, persist=True, verbose=False)
        detections = self.parse_detections(results)
        
        # Logic for tracking
        target_obj = self.track_logic(detections)
        
        # Publish tracking results
        if target_obj:
            tracking_info = f"ID: {target_obj['id']}, Name: {target_obj['name']}"
            self.tracking_pub.publish(String(data=tracking_info))
        
        # Log tracking data
        self.log_tracking_data(target_obj)

    def ros_image_to_cv2(self, img_msg):
        # Convert ROS Image message to OpenCV format
        # Assuming img_msg is of type sensor_msgs/Image
        # Implement conversion logic here
        pass

    def parse_detections(self, results):
        detections = []
        if results[0].boxes and results[0].boxes.id is not None:
            boxes = results[0].boxes
            track_ids = boxes.id.int().cpu().tolist()
            for box, track_id in zip(boxes, track_ids):
                x1, y1, x2, y2 = box.xyxy[0].cpu().numpy()
                cx = int((x1 + x2) / 2)
                cy = int((y1 + y2) / 2)
                detections.append({
                    "id": track_id,
                    "bbox": (int(x1), int(y1), int(x2), int(y2)),
                    "center": (cx, cy),
                    "name": self.model.names[int(box.cls[0])],
                    "conf": float(box.conf[0])
                })
        return detections

    def track_logic(self, detections):
        target_obj = None
        if self.locked_track_id is not None:
            target_obj = next((d for d in detections if d["id"] == self.locked_track_id), None)
        else:
            if detections:
                detections.sort(key=lambda x: x["dist"])
                target_obj = detections[0]
        return target_obj

    def log_tracking_data(self, target_obj):
        if target_obj:
            with open(self.log_filename, 'a', newline='') as f:
                writer = csv.writer(f)
                writer.writerow([
                    datetime.now().isoformat(), self.frame_id, target_obj["name"],
                    target_obj["center"][0], target_obj["center"][1], 
                    target_obj["bbox"][0], target_obj["bbox"][1], 
                    target_obj["center"][0] - 640, target_obj["center"][1] - 360
                ])

def main(args=None):
    rclpy.init(args=args)
    tracker_node = PiTrackerNode()
    rclpy.spin(tracker_node)
    tracker_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()