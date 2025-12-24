#!/usr/bin/env python3
"""
Interactive GUI Camera Controller with Mouse/Slider Control

Provides a simple Qt GUI to control camera azimuth and elevation with sliders.
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
import sys
import math

try:
    from PyQt5.QtWidgets import (QApplication, QWidget, QVBoxLayout, QHBoxLayout, 
                                  QSlider, QLabel, QPushButton, QGroupBox)
    from PyQt5.QtCore import Qt, QTimer
    from PyQt5.QtGui import QFont
except ImportError:
    print("PyQt5 not installed. Install with: sudo apt install python3-pyqt5")
    sys.exit(1)


class CameraGUIController(QWidget):
    def __init__(self, node):
        super().__init__()
        self.node = node
        self.init_ui()
        
        # Current angles
        self.current_azimuth = 0.0
        self.current_elevation = 0.0
        
    def init_ui(self):
        self.setWindowTitle('Space Panda Camera Controller')
        self.setGeometry(100, 100, 500, 400)
        
        layout = QVBoxLayout()
        
        # Title
        title = QLabel('Pan-Tilt Camera Control')
        title_font = QFont()
        title_font.setPointSize(16)
        title_font.setBold(True)
        title.setFont(title_font)
        title.setAlignment(Qt.AlignCenter)
        layout.addWidget(title)
        
        # Azimuth Control (Horizontal)
        azimuth_group = QGroupBox('Azimuth - Horizontal Control')
        azimuth_layout = QVBoxLayout()
        
        self.azimuth_label = QLabel('0°')
        self.azimuth_label.setAlignment(Qt.AlignCenter)
        azimuth_layout.addWidget(self.azimuth_label)
        
        self.azimuth_slider = QSlider(Qt.Horizontal)
        self.azimuth_slider.setMinimum(-180)
        self.azimuth_slider.setMaximum(180)
        self.azimuth_slider.setValue(0)
        self.azimuth_slider.setTickPosition(QSlider.TicksBelow)
        self.azimuth_slider.setTickInterval(45)
        self.azimuth_slider.valueChanged.connect(self.update_azimuth)
        azimuth_layout.addWidget(self.azimuth_slider)
        
        azimuth_range = QLabel('-180° to 180°')
        azimuth_range.setAlignment(Qt.AlignCenter)
        azimuth_layout.addWidget(azimuth_range)
        
        azimuth_group.setLayout(azimuth_layout)
        layout.addWidget(azimuth_group)
        
        # Elevation Control (Vertical)
        elevation_group = QGroupBox('Elevation - Vertical Control')
        elevation_layout = QVBoxLayout()
        
        self.elevation_label = QLabel('0°')
        self.elevation_label.setAlignment(Qt.AlignCenter)
        elevation_layout.addWidget(self.elevation_label)
        
        self.elevation_slider = QSlider(Qt.Horizontal)
        self.elevation_slider.setMinimum(-90)
        self.elevation_slider.setMaximum(90)
        self.elevation_slider.setValue(0)
        self.elevation_slider.setTickPosition(QSlider.TicksBelow)
        self.elevation_slider.setTickInterval(30)
        self.elevation_slider.valueChanged.connect(self.update_elevation)
        elevation_layout.addWidget(self.elevation_slider)
        
        elevation_range = QLabel('-90° to 90°')
        elevation_range.setAlignment(Qt.AlignCenter)
        elevation_layout.addWidget(elevation_range)
        
        elevation_group.setLayout(elevation_layout)
        layout.addWidget(elevation_group)
        
        # Preset Buttons
        preset_group = QGroupBox('Presets')
        preset_layout = QHBoxLayout()
        
        btn_earth = QPushButton('Look at Earth')
        btn_earth.clicked.connect(self.preset_earth)
        preset_layout.addWidget(btn_earth)
        
        btn_arm = QPushButton('Look at Arm')
        btn_arm.clicked.connect(self.preset_arm)
        preset_layout.addWidget(btn_arm)
        
        btn_center = QPushButton('Center')
        btn_center.clicked.connect(self.preset_center)
        preset_layout.addWidget(btn_center)
        
        btn_down = QPushButton('Look Down')
        btn_down.clicked.connect(self.preset_down)
        preset_layout.addWidget(btn_down)
        
        preset_group.setLayout(preset_layout)
        layout.addWidget(preset_group)
        
        # Status
        self.status_label = QLabel('Ready')
        self.status_label.setAlignment(Qt.AlignCenter)
        self.status_label.setStyleSheet("QLabel { background-color: #e0e0e0; padding: 5px; }")
        layout.addWidget(self.status_label)
        
        self.setLayout(layout)
        
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
            f'Camera: Azimuth={math.degrees(self.current_azimuth):.1f}°, '
            f'Elevation={math.degrees(self.current_elevation):.1f}°'
        )
    
    def preset_earth(self):
        """Point camera toward Earth (180° azimuth)"""
        self.azimuth_slider.setValue(180)
        self.elevation_slider.setValue(0)
        
    def preset_arm(self):
        """Point camera toward arm"""
        self.azimuth_slider.setValue(0)
        self.elevation_slider.setValue(30)
        
    def preset_center(self):
        """Center camera"""
        self.azimuth_slider.setValue(0)
        self.elevation_slider.setValue(0)
        
    def preset_down(self):
        """Point camera downward"""
        self.elevation_slider.setValue(90)


class CameraControlNode(Node):
    def __init__(self):
        super().__init__('camera_gui_controller')
        
        # Publishers
        self.azimuth_pub = self.create_publisher(Float64, '/camera/azimuth_cmd', 10)
        self.elevation_pub = self.create_publisher(Float64, '/camera/elevation_cmd', 10)
        
        self.get_logger().info('Camera GUI Controller started')


def main(args=None):
    rclpy.init(args=args)
    
    # Create ROS node
    node = CameraControlNode()
    
    # Create Qt application
    app = QApplication(sys.argv)
    
    # Create GUI
    gui = CameraGUIController(node)
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

