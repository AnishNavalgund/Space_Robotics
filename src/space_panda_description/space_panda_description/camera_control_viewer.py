#!/usr/bin/env python3
"""
Camera Control & Viewer

Single application that combines:
- Live camera feed display
- Pan/tilt control sliders
- Preset buttons
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import sys
import math
import cv2
import numpy as np

try:
    from PyQt5.QtWidgets import (QApplication, QWidget, QVBoxLayout, QHBoxLayout, 
                                  QSlider, QLabel, QPushButton, QGroupBox)
    from PyQt5.QtCore import Qt, QTimer, pyqtSignal
    from PyQt5.QtGui import QFont, QImage, QPixmap
except ImportError:
    print("PyQt5 not installed. Install with: sudo apt install python3-pyqt5")
    sys.exit(1)


class CameraControlViewer(QWidget):
    """GUI for camera control and viewing"""
    
    # Signal for thread-safe image updates
    image_signal = pyqtSignal(np.ndarray)
    
    def __init__(self, node):
        super().__init__()
        self.node = node
        self.bridge = CvBridge()
        
        # Current angles
        self.current_azimuth = 0.0
        self.current_elevation = 0.0
        
        # Frame counter
        self.frame_count = 0
        
        # Connect signal
        self.image_signal.connect(self.update_camera_display)
        
        # Subscribe to camera topic
        self.image_sub = self.node.create_subscription(
            Image,
            '/space_panda/camera/image',
            self.image_callback,
            10
        )
        
        self.init_ui()
        
    def init_ui(self):
        self.setWindowTitle('Realtime Camera Viewer')
        self.setGeometry(100, 100, 800, 700)
        
        main_layout = QVBoxLayout()
        
        # ========== TITLE ==========
        title = QLabel('Space Panda Camera System')
        title_font = QFont()
        title_font.setPointSize(18)
        title_font.setBold(True)
        title.setFont(title_font)
        title.setAlignment(Qt.AlignCenter)
        main_layout.addWidget(title)
        
        # ========== CAMERA FEED ==========
        camera_group = QGroupBox('Live Camera Feed')
        camera_layout = QVBoxLayout()
        
        self.camera_label = QLabel()
        self.camera_label.setAlignment(Qt.AlignCenter)
        self.camera_label.setMinimumSize(640, 480)
        self.camera_label.setStyleSheet("QLabel { background-color: #000000; border: 2px solid #4CAF50; }")
        self.camera_label.setText('Waiting for camera feed...')
        self.camera_label.setStyleSheet(
            "QLabel { background-color: #000000; border: 2px solid #4CAF50; color: #4CAF50; }"
        )
        camera_layout.addWidget(self.camera_label)
        
        # Camera info
        self.camera_info = QLabel('Frame: 0 | FPS: --')
        self.camera_info.setAlignment(Qt.AlignCenter)
        camera_layout.addWidget(self.camera_info)
        
        camera_group.setLayout(camera_layout)
        main_layout.addWidget(camera_group)
        
        # ========== CONTROL PANEL ==========
        control_panel = QHBoxLayout()
        
        # LEFT: Azimuth Control
        azimuth_group = QGroupBox('Azimuth - Horizontal Control')
        azimuth_layout = QVBoxLayout()
        
        self.azimuth_label = QLabel('0°')
        self.azimuth_label.setAlignment(Qt.AlignCenter)
        azimuth_label_font = QFont()
        azimuth_label_font.setPointSize(14)
        azimuth_label_font.setBold(True)
        self.azimuth_label.setFont(azimuth_label_font)
        azimuth_layout.addWidget(self.azimuth_label)
        
        self.azimuth_slider = QSlider(Qt.Horizontal)
        self.azimuth_slider.setMinimum(-180)
        self.azimuth_slider.setMaximum(180)
        self.azimuth_slider.setValue(0)
        self.azimuth_slider.setTickPosition(QSlider.TicksBelow)
        self.azimuth_slider.setTickInterval(45)
        self.azimuth_slider.valueChanged.connect(self.update_azimuth)
        azimuth_layout.addWidget(self.azimuth_slider)
        
        azimuth_range = QLabel('-180 to 180')
        azimuth_range.setAlignment(Qt.AlignCenter)
        azimuth_layout.addWidget(azimuth_range)
        
        azimuth_group.setLayout(azimuth_layout)
        control_panel.addWidget(azimuth_group)
        
        # RIGHT: Elevation Control
        elevation_group = QGroupBox('Elevation - Vertical Control')
        elevation_layout = QVBoxLayout()
        
        self.elevation_label = QLabel('0°')
        self.elevation_label.setAlignment(Qt.AlignCenter)
        elevation_label_font = QFont()
        elevation_label_font.setPointSize(14)
        elevation_label_font.setBold(True)
        self.elevation_label.setFont(elevation_label_font)
        elevation_layout.addWidget(self.elevation_label)
        
        self.elevation_slider = QSlider(Qt.Horizontal)
        self.elevation_slider.setMinimum(-90)
        self.elevation_slider.setMaximum(90)
        self.elevation_slider.setValue(0)
        self.elevation_slider.setTickPosition(QSlider.TicksBelow)
        self.elevation_slider.setTickInterval(30)
        self.elevation_slider.valueChanged.connect(self.update_elevation)
        elevation_layout.addWidget(self.elevation_slider)
        
        elevation_range = QLabel('-90 to 90')
        elevation_range.setAlignment(Qt.AlignCenter)
        elevation_layout.addWidget(elevation_range)
        
        elevation_group.setLayout(elevation_layout)
        control_panel.addWidget(elevation_group)
        
        main_layout.addLayout(control_panel)
        
        # ========== PRESET BUTTONS ==========
        preset_group = QGroupBox('Quick Presets')
        preset_layout = QHBoxLayout()
        
        btn_earth = QPushButton('Look at Earth')
        btn_earth.clicked.connect(self.preset_earth)
        btn_earth.setMinimumHeight(40)
        preset_layout.addWidget(btn_earth)
        
        btn_arm = QPushButton('Look at Arm')
        btn_arm.clicked.connect(self.preset_arm)
        btn_arm.setMinimumHeight(40)
        preset_layout.addWidget(btn_arm)
        
        preset_group.setLayout(preset_layout)
        main_layout.addWidget(preset_group)
        
        # ========== STATUS BAR ==========
        self.status_label = QLabel('Ready - Waiting for camera...')
        self.status_label.setAlignment(Qt.AlignCenter)
        self.status_label.setStyleSheet(
            "QLabel { background-color: #e0e0e0; padding: 8px; font-size: 12px; }"
        )
        main_layout.addWidget(self.status_label)
        
        self.setLayout(main_layout)
        
        # FPS calculation
        self.last_frame_time = None
        self.fps_counter = 0
        self.fps_timer = QTimer()
        self.fps_timer.timeout.connect(self.update_fps_display)
        self.fps_timer.start(1000)  # Update FPS every second
        
    def image_callback(self, msg):
        """ROS callback for camera images"""
        try:
            # Convert ROS Image to OpenCV format
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            self.frame_count += 1
            self.fps_counter += 1
            
            # Emit signal for Qt thread
            self.image_signal.emit(cv_image)
            
        except Exception as e:
            self.node.get_logger().error(f'Error processing image: {e}')
    
    def update_camera_display(self, cv_image):
        """Update Qt display with new image (runs in Qt thread)"""
        try:
            # Convert BGR to RGB
            rgb_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2RGB)
            
            # Add frame overlay
            text = f"Frame: {self.frame_count}"
            cv2.putText(rgb_image, text, (10, 30), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)
            
            # Convert to Qt format
            h, w, ch = rgb_image.shape
            bytes_per_line = ch * w
            qt_image = QImage(rgb_image.data, w, h, bytes_per_line, QImage.Format_RGB888)
            
            # Scale to fit label while maintaining aspect ratio
            pixmap = QPixmap.fromImage(qt_image)
            scaled_pixmap = pixmap.scaled(
                self.camera_label.width(), 
                self.camera_label.height(),
                Qt.KeepAspectRatio,
                Qt.SmoothTransformation
            )
            
            self.camera_label.setPixmap(scaled_pixmap)
            
        except Exception as e:
            self.node.get_logger().error(f'Error updating display: {e}')
    
    def update_fps_display(self):
        """Update FPS counter"""
        fps = self.fps_counter
        self.fps_counter = 0
        self.camera_info.setText(f'Frame: {self.frame_count} | FPS: {fps}')
    
    def update_azimuth(self, value):
        """Update azimuth angle from slider"""
        self.current_azimuth = math.radians(value)
        self.azimuth_label.setText(f'{value}°')
        self.publish_camera_angles()
        
    def update_elevation(self, value):
        """Update elevation angle from slider"""
        self.current_elevation = math.radians(value)
        self.elevation_label.setText(f'{value}°')
        self.publish_camera_angles()
        
    def publish_camera_angles(self):
        """Publish camera angles to ROS topics"""
        az_msg = Float64()
        az_msg.data = float(self.current_azimuth)
        self.node.azimuth_pub.publish(az_msg)
        
        el_msg = Float64()
        el_msg.data = float(self.current_elevation)
        self.node.elevation_pub.publish(el_msg)
        
        self.status_label.setText(
            f'Camera Position: Azimuth={math.degrees(self.current_azimuth):.1f}° | '
            f'Elevation={math.degrees(self.current_elevation):.1f}°'
        )
    
    def preset_earth(self):
        """Point camera toward Earth (180° azimuth)"""
        self.azimuth_slider.setValue(180)
        self.elevation_slider.setValue(0)
        self.status_label.setText('Earth View')
        
    def preset_arm(self):
        """Point camera toward arm"""
        self.azimuth_slider.setValue(0)
        self.elevation_slider.setValue(30)
        self.status_label.setText('Robot Arm View')


class CameraNode(Node):
    """ROS2 node for camera control"""
    
    def __init__(self):
        super().__init__('camera_control_viewer')
        
        # Publishers for camera control
        self.azimuth_pub = self.create_publisher(Float64, '/camera/azimuth_cmd', 10)
        self.elevation_pub = self.create_publisher(Float64, '/camera/elevation_cmd', 10)
        
        self.get_logger().info('Camera Control & Viewer started')
        self.get_logger().info('Subscribing to: /space_panda/camera/image')
        self.get_logger().info('Publishing to: /camera/azimuth_cmd, /camera/elevation_cmd')


def main(args=None):
    rclpy.init(args=args)
    
    # Create ROS node
    node = CameraNode()
    
    # Create Qt application
    app = QApplication(sys.argv)
    
    # Create GUI
    gui = CameraControlViewer(node)
    gui.show()
    
    # Timer to spin ROS
    timer = QTimer()
    timer.timeout.connect(lambda: rclpy.spin_once(node, timeout_sec=0))
    timer.start(10)  # 100 Hz
    
    # Run
    try:
        sys.exit(app.exec_())
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

