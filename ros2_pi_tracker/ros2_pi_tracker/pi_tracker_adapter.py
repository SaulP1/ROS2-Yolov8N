import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
import cv2
import numpy as np
import json
import time
from ultralytics import YOLO
from datetime import datetime

class PiTrackerAdapter(Node):
    def __init__(self):
        super().__init__('pi_tracker_adapter')
        
        # Initialize parameters
        self.camera_index = 0
        self.model_path = 'yolov8n.pt'
        self.locked_track_id = None
        self.frame_id = 0
        self.screen_center = (640, 360)  # Default for 1280x720 resolution
        
        # Initialize YOLO model
        self.model = YOLO(self.model_path)
        
        # Create a publisher for tracking results
        self.tracking_pub = self.create_publisher(String, 'tracking_results', 10)
        
        # Create a subscription to the camera feed
        self.create_subscription(Image, 'camera/image_raw', self.camera_callback, 10)

        # Start the tracking process
        self.start_tracking()

    def camera_callback(self, msg):
        # Convert ROS Image message to OpenCV format
        frame = self.convert_ros_image_to_cv2(msg)
        self.process_frame(frame)

    def convert_ros_image_to_cv2(self, img_msg):
        # Convert the ROS Image message to a CV2 image
        # Assuming img_msg is of type sensor_msgs/Image
        # This function needs to be implemented based on the encoding of the image
        pass

    def process_frame(self, frame):
        self.frame_id += 1
        
        # Detection
        results = self.model.track(frame, persist=True, verbose=False)
        
        detections = []
        if results[0].boxes and results[0].boxes.id is not None:
            boxes = results[0].boxes
            track_ids = boxes.id.int().cpu().tolist()
            
            for box, track_id in zip(boxes, track_ids):
                x1, y1, x2, y2 = box.xyxy[0].cpu().numpy()
                cx = int((x1 + x2) / 2)
                cy = int((y1 + y2) / 2)
                dist = np.sqrt((cx - self.screen_center[0])**2 + (cy - self.screen_center[1])**2)
                
                detections.append({
                    "id": track_id,
                    "bbox": (int(x1), int(y1), int(x2), int(y2)),
                    "center": (cx, cy),
                    "dist": dist,
                    "name": self.model.names[int(box.cls[0])],
                    "conf": float(box.conf[0])
                })

        # Logic Controller
        target_obj = None
        
        if self.locked_track_id is not None:
            target_obj = next((d for d in detections if d["id"] == self.locked_track_id), None)
        else:
            if detections:
                detections.sort(key=lambda x: x["dist"])
                target_obj = detections[0]

        # Prepare tracking data for publishing
        data_packet = {
            "frame": self.frame_id,
            "status": "search" if self.locked_track_id is None else "locked",
            "target_id": self.locked_track_id,
            "dx": 0,
            "dy": 0
        }

        if target_obj:
            x1, y1, x2, y2 = target_obj['bbox']
            tx, ty = target_obj['center']
            dx = tx - self.screen_center[0]
            dy = ty - self.screen_center[1]
            data_packet["dx"] = dx
            data_packet["dy"] = dy
            
            # Publish tracking results
            self.tracking_pub.publish(json.dumps(data_packet))

    def start_tracking(self):
        # This method can be used to start the tracking loop if needed
        pass

def main(args=None):
    rclpy.init(args=args)
    pi_tracker_adapter = PiTrackerAdapter()
    rclpy.spin(pi_tracker_adapter)
    pi_tracker_adapter.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()