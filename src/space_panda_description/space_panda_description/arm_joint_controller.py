#!/usr/bin/env python3
"""
FR3 Arm Joint Controller GUI
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
import sys
import math

try:
    from PyQt5.QtWidgets import (QApplication, QWidget, QVBoxLayout, QHBoxLayout, 
                                  QSlider, QLabel, QGroupBox)
    from PyQt5.QtCore import Qt, QTimer
    from PyQt5.QtGui import QFont
except ImportError:
    print("PyQt5 not installed. Install with: sudo apt install python3-pyqt5")
    sys.exit(1)


class ArmJointController(QWidget):
    def __init__(self, node):
        super().__init__()
        self.node = node
        
        # Joint limits (radians)
        self.joint_limits = [
            (-2.8973, 2.8973),   # Joint 1
            (-1.7628, 1.7628),   # Joint 2
            (-2.8973, 2.8973),   # Joint 3
            (-3.0718, -0.0698),  # Joint 4
            (-2.8973, 2.8973),   # Joint 5
            (-0.0175, 3.7525),   # Joint 6
            (-2.8973, 2.8973),   # Joint 7
        ]
        
        # Gripper limits (meters)
        self.gripper_limits = (0.0, 0.04)
        
        self.init_ui()
        
    def init_ui(self):
        self.setWindowTitle('FR3 Arm Joint Controller')
        self.setGeometry(100, 100, 500, 600)
        
        main_layout = QVBoxLayout()
        
        # Title
        title = QLabel('7-DOF Joint Control')
        title_font = QFont()
        title_font.setPointSize(16)
        title_font.setBold(True)
        title.setFont(title_font)
        title.setAlignment(Qt.AlignCenter)
        main_layout.addWidget(title)
        
        # Create sliders for each joint
        self.sliders = []
        self.labels = []
        
        for i in range(7):
            group = QGroupBox(f'Joint {i+1}')
            layout = QVBoxLayout()
            
            # Value label
            label = QLabel('0.00')
            label.setAlignment(Qt.AlignCenter)
            label_font = QFont()
            label_font.setPointSize(12)
            label_font.setBold(True)
            label.setFont(label_font)
            layout.addWidget(label)
            self.labels.append(label)
            
            # Slider
            slider = QSlider(Qt.Horizontal)
            lower, upper = self.joint_limits[i]
            slider.setMinimum(int(math.degrees(lower)))
            slider.setMaximum(int(math.degrees(upper)))
            slider.setValue(0)
            slider.setTickPosition(QSlider.TicksBelow)
            slider.setTickInterval(30)
            slider.valueChanged.connect(lambda val, idx=i: self.update_joint(idx, val))
            layout.addWidget(slider)
            self.sliders.append(slider)
            
            # Range label
            range_label = QLabel(f'{math.degrees(lower):.0f} to {math.degrees(upper):.0f}')
            range_label.setAlignment(Qt.AlignCenter)
            layout.addWidget(range_label)
            
            group.setLayout(layout)
            main_layout.addWidget(group)
        
        # Gripper control
        gripper_group = QGroupBox('Gripper')
        gripper_layout = QVBoxLayout()
        
        # Gripper value label
        self.gripper_label = QLabel('0.00')
        self.gripper_label.setAlignment(Qt.AlignCenter)
        gripper_label_font = QFont()
        gripper_label_font.setPointSize(12)
        gripper_label_font.setBold(True)
        self.gripper_label.setFont(gripper_label_font)
        gripper_layout.addWidget(self.gripper_label)
        
        # Gripper slider
        self.gripper_slider = QSlider(Qt.Horizontal)
        self.gripper_slider.setMinimum(0)
        self.gripper_slider.setMaximum(40)  # 0 to 0.04 meters (scaled by 1000)
        self.gripper_slider.setValue(0)
        self.gripper_slider.setTickPosition(QSlider.TicksBelow)
        self.gripper_slider.setTickInterval(10)
        self.gripper_slider.valueChanged.connect(self.update_gripper)
        gripper_layout.addWidget(self.gripper_slider)
        
        gripper_group.setLayout(gripper_layout)
        main_layout.addWidget(gripper_group)
        
        # Status
        self.status_label = QLabel('Ready')
        self.status_label.setAlignment(Qt.AlignCenter)
        self.status_label.setStyleSheet("QLabel { background-color: #e0e0e0; padding: 5px; }")
        main_layout.addWidget(self.status_label)
        
        self.setLayout(main_layout)
        
    def update_joint(self, joint_idx, value_deg):
        """Update joint angle"""
        value_rad = math.radians(value_deg)
        self.labels[joint_idx].setText(f'{value_deg:.0f}')
        
        # Publish
        msg = Float64()
        msg.data = float(value_rad)
        self.node.joint_pubs[joint_idx].publish(msg)
        
        self.status_label.setText(f'Joint {joint_idx+1}: {value_deg:.0f} deg')
    
    def update_gripper(self, value):
        """Update gripper position"""
        value_m = value / 1000.0  # Convert to meters
        self.gripper_label.setText(f'{value:.0f}')
        
        # Publish
        msg = Float64()
        msg.data = float(value_m)
        self.node.gripper_pub.publish(msg)
        
        status = 'open' if value > 20 else 'closed'
        self.status_label.setText(f'Gripper: {value:.0f} ({status})')


class ArmNode(Node):
    def __init__(self):
        super().__init__('arm_joint_controller')
        
        # Publishers for 7 joints
        self.joint_pubs = []
        for i in range(7):
            pub = self.create_publisher(Float64, f'/fr3/joint{i+1}_cmd', 10)
            self.joint_pubs.append(pub)
        
        # Gripper publisher
        self.gripper_pub = self.create_publisher(Float64, '/fr3/gripper_cmd', 10)
        
        self.get_logger().info('Arm Joint Controller started')


def main(args=None):
    rclpy.init(args=args)
    
    node = ArmNode()
    app = QApplication(sys.argv)
    gui = ArmJointController(node)
    gui.show()
    
    timer = QTimer()
    timer.timeout.connect(lambda: rclpy.spin_once(node, timeout_sec=0))
    timer.start(10)
    
    try:
        sys.exit(app.exec_())
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

