#!/usr/bin/env python3

import os
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from vision_msgs.msg import Detection2DArray, Detection2D, ObjectHypothesisWithPose
from cv_bridge import CvBridge
import cv2
from ultralytics import YOLO

class YOLONode(Node):
    def __init__(self):
        super().__init__('yolo_camera_node')

        # Subscriber to the camera topic
        self.subscription = self.create_subscription(
            Image,
            'camera/image_raw',
            self.image_callback,
            10
        )

        # Publisher for the detection results
        self.detection_publisher = self.create_publisher(Detection2DArray, 'detections', 10)

        # Publisher for the annotated image (commented out for now)
        self.publisher = self.create_publisher(Image, 'annotated_image', 10)

        # OpenCV bridge for converting images
        self.bridge = CvBridge()

        # YOLO model initialization
        model_directory = os.path.join(
            os.environ['HOME'], 'robocon_ws', 'src', 'object_detection', 'models', 'yolov8m.pt'
        )
        self.model = YOLO(model_directory)

    def image_callback(self, msg):
        # Convert ROS Image message to OpenCV image
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

        # YOLO inference
        results = self.model(frame)

        # Create a Detection2DArray message
        detection_array_msg = Detection2DArray()

        # Annotate the frame with YOLO results and populate Detection2DArray
        for r in results:
            boxes = r.boxes
            for box in boxes:
                b = box.xyxy[0].to('cpu').detach().numpy().copy()  # Box coordinates
                c = box.cls
                score = box.conf.item()  # Confidence score

                # Check if the detected object is a "sports ball"
                if self.model.names[int(c)] != "sports ball":
                    continue  # Skip if it's not a sports ball

                # Create a Detection2D message
                detection_msg = Detection2D()

                # Initialize the Pose2D object for bbox.center
                detection_msg.bbox.center.position.x = (b[0] + b[2]) / 2.0  # Correctly assign x
                detection_msg.bbox.center.position.y = (b[1] + b[3]) / 2.0  # Correctly assign y
                detection_msg.bbox.center.theta = 0.0  # Set orientation to 0 (default)

                # Set the size of the bounding box
                detection_msg.bbox.size_x = float(b[2] - b[0])
                detection_msg.bbox.size_y = float(b[3] - b[1])

                # Add object hypothesis
                hypothesis = ObjectHypothesisWithPose()
                hypothesis.hypothesis.class_id = str(int(c))  # Class ID as string
                hypothesis.hypothesis.score = score
                detection_msg.results.append(hypothesis)

                # Add detection to the array
                detection_array_msg.detections.append(detection_msg)

                # Draw bounding boxes on the frame
                cv2.rectangle(frame, (int(b[0]), int(b[1])), (int(b[2]), int(b[3])), (0, 0, 255), 2)
                cv2.putText(frame, text=f"{self.model.names[int(c)]} {score:.2f}", org=(int(b[0]), int(b[1])),
                            fontFace=cv2.FONT_HERSHEY_SIMPLEX, fontScale=0.7, color=(0, 0, 255), thickness=2)
                
                # Draw line from center of the screen to the detected object
                center_x = int((b[0] + b[2]) / 2)
                center_y = int((b[1] + b[3]) / 2)
                cv2.line(frame, (320, 460), (center_x, center_y), (0, 255, 0), 2)
                cv2.putText(frame, text=f"({center_x}, {center_y})", org=(center_x, center_y),
                            fontFace=cv2.FONT_HERSHEY_SIMPLEX, fontScale=0.7, color=(0, 255, 0), thickness=2)
                
        # Center line for visualization
        mid_screen_x = 320
        mid_screen_y = 460 
        cv2.line(frame, (mid_screen_x, 0), (mid_screen_x, frame.shape[0]), (255, 0, 0), 1)
        cv2.line(frame, (0, mid_screen_y), (frame.shape[1], mid_screen_y), (255, 0, 0), 1)
        

        # Publish the detection results
        self.detection_publisher.publish(detection_array_msg)

        # Publish the annotated image (commented out for now)
        annotated_msg = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
        self.publisher.publish(annotated_msg)

        # Optional: Display the annotated image locally (for debugging)
        # cv2.imshow("Annotated Image", frame)
        # if cv2.waitKey(1) & 0xFF == ord('q'):
        #     self.destroy_node()
        #     rclpy.shutdown()

def main(args=None):
    rclpy.init(args=args)
    node = YOLONode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()