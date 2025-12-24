#!/usr/bin/env python3
"""
Simple OpenCV Camera Viewer

A lightweight alternative to rqt_image_view using OpenCV.
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np


class OpenCVCameraViewer(Node):
    def __init__(self):
        super().__init__('opencv_camera_viewer')
        
        self.bridge = CvBridge()
        
        # Subscribe to camera topic
        self.image_sub = self.create_subscription(
            Image,
            '/space_panda/camera/image',
            self.image_callback,
            10
        )
        
        # Create OpenCV window
        cv2.namedWindow('Space Panda Camera', cv2.WINDOW_NORMAL)
        cv2.resizeWindow('Space Panda Camera', 640, 480)
        
        self.get_logger().info('OpenCV Camera Viewer started')
        self.get_logger().info('Press "q" to quit, "s" to save screenshot')
        
        self.frame_count = 0
        self.screenshot_count = 0
        
    def image_callback(self, msg):
        """Convert ROS Image message to OpenCV and display"""
        try:
            # Convert ROS Image to OpenCV format
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            
            # Add frame info overlay
            self.frame_count += 1
            text = f"Frame: {self.frame_count}"
            cv2.putText(cv_image, text, (10, 30), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
            
            # Display
            cv2.imshow('Space Panda Camera', cv_image)
            
            # Handle key presses
            key = cv2.waitKey(1) & 0xFF
            
            if key == ord('q'):
                self.get_logger().info('Quit requested')
                rclpy.shutdown()
            elif key == ord('s'):
                self.save_screenshot(cv_image)
            elif key == ord('f'):
                self.toggle_fullscreen()
                
        except Exception as e:
            self.get_logger().error(f'Error processing image: {e}')
    
    def save_screenshot(self, image):
        """Save current frame as image file"""
        filename = f'camera_screenshot_{self.screenshot_count:04d}.png'
        cv2.imwrite(filename, image)
        self.screenshot_count += 1
        self.get_logger().info(f'Screenshot saved: {filename}')
    
    def toggle_fullscreen(self):
        """Toggle fullscreen mode"""
        cv2.setWindowProperty('Space Panda Camera', 
                             cv2.WND_PROP_FULLSCREEN, 
                             cv2.WINDOW_FULLSCREEN)


def main(args=None):
    rclpy.init(args=args)
    
    viewer = OpenCVCameraViewer()
    
    try:
        rclpy.spin(viewer)
    except KeyboardInterrupt:
        pass
    finally:
        cv2.destroyAllWindows()
        viewer.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

