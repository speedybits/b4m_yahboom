#!/usr/bin/env python3

import os
import sys
import json
import yaml
import numpy as np
import random
import traceback
import subprocess
import threading
from PIL import Image
from datetime import datetime

import rclpy
from rclpy.node import Node
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import PoseWithCovarianceStamped, Pose
# Import visualization messages separately to avoid any import issues
from visualization_msgs.msg import MarkerArray
from visualization_msgs.msg import Marker
from std_msgs.msg import String

# Import MQTT client
import paho.mqtt.client as mqtt

from PyQt5.QtWidgets import (QApplication, QMainWindow, QWidget, QVBoxLayout, 
                             QHBoxLayout, QLabel, QPushButton, QListWidget, QListWidgetItem,
                             QSplitter, QComboBox, QGroupBox, QStatusBar, QScrollArea,
                             QMessageBox, QFileDialog, QInputDialog, QSpinBox, QDoubleSpinBox,
                             QCheckBox, QTextEdit, QProgressBar, QLineEdit)
from PyQt5.QtCore import Qt, QSettings, QTimer, QRectF, QRect
from PyQt5.QtGui import QPainter, QColor, QPen, QBrush, QPixmap, QFont, QImage

class WaypointManagerGUI(QMainWindow):
    def __init__(self, ros_node):
        super().__init__()
        
        # Store reference to ROS node
        self.ros_node = ros_node
        
        # Parameter management
        self.parameters = {}
        self.parameter_widgets = {}
        self.parameters_modified = False
        self.build_required = False
        
        # Initialize UI
        self.initUI()
        
        # Load settings
        self.loadSettings()
        
    def initUI(self):
        # Set window properties
        self.setWindowTitle('Waypoint Manager')
        self.setGeometry(100, 100, 1200, 800)
        
        # Create central widget and main layout
        central_widget = QWidget()
        self.setCentralWidget(central_widget)
        main_layout = QHBoxLayout(central_widget)
        
        # Create splitter for resizable panels
        splitter = QSplitter(Qt.Horizontal)
        main_layout.addWidget(splitter)
        
        # Create left panel (controls)
        left_panel = QWidget()
        left_layout = QVBoxLayout(left_panel)
        
        # Configuration Management (at top to show it saves everything)
        config_group = QGroupBox('Configuration Management')
        config_layout = QVBoxLayout(config_group)
        
        config_buttons_layout = QHBoxLayout()
        save_config_btn = QPushButton('💾 Save Config')
        restore_config_btn = QPushButton('📁 Load Config')
        save_config_btn.setStyleSheet('background-color: #4CAF50; color: white; font-weight: bold;')
        restore_config_btn.setStyleSheet('background-color: #2196F3; color: white; font-weight: bold;')
        save_config_btn.clicked.connect(self.saveConfiguration)
        restore_config_btn.clicked.connect(self.loadConfiguration)
        config_buttons_layout.addWidget(save_config_btn)
        config_buttons_layout.addWidget(restore_config_btn)
        config_layout.addLayout(config_buttons_layout)
        
        # Add explanatory label
        config_help = QLabel('Saves/loads: parameters, map, and all waypoints')
        config_help.setStyleSheet('font-size: 10px; color: #666666; font-style: italic;')
        config_layout.addWidget(config_help)
        
        left_layout.addWidget(config_group)
        
        # Map info group
        map_group = QGroupBox('Map Information')
        map_layout = QVBoxLayout(map_group)
        self.map_label = QLabel('Using default map')
        map_layout.addWidget(self.map_label)
        left_layout.addWidget(map_group)
        
        # Waypoint list group
        waypoint_group = QGroupBox('Waypoints')
        waypoint_layout = QVBoxLayout(waypoint_group)
        self.waypoint_list = QListWidget()
        # Enable rich text interpretation for HTML formatting
        self.waypoint_list.setTextElideMode(Qt.ElideRight)
        self.waypoint_list.setWordWrap(True)
        waypoint_layout.addWidget(self.waypoint_list)
        left_layout.addWidget(waypoint_group)
        
        # Waypoint actions group
        actions_group = QGroupBox('Actions')
        actions_layout = QVBoxLayout(actions_group)
        self.add_waypoint_btn = QPushButton('Add Waypoint')
        self.edit_waypoint_btn = QPushButton('Edit Selected')
        self.delete_waypoint_btn = QPushButton('Delete Selected')
        self.navigate_waypoint_btn = QPushButton('Go to Selected Waypoint')
        self.navigate_waypoint_btn.setStyleSheet('background-color: #4CAF50; color: white; font-weight: bold;')
        self.navigate_waypoint_btn.setEnabled(False)  # Disabled until a waypoint is selected
        self.reset_view_btn = QPushButton('Reset Map View')
        self.reset_view_btn.setStyleSheet('background-color: #2196F3; color: white;')
        actions_layout.addWidget(self.add_waypoint_btn)
        actions_layout.addWidget(self.edit_waypoint_btn)
        actions_layout.addWidget(self.delete_waypoint_btn)
        actions_layout.addWidget(self.navigate_waypoint_btn)
        actions_layout.addWidget(self.reset_view_btn)
        left_layout.addWidget(actions_group)
        
        # Create status bar first (needed by parameter controls)
        self.status_bar = QStatusBar()
        self.setStatusBar(self.status_bar)
        self.status_bar.showMessage('Ready')
        
        # Navigation Parameters group
        self.params_group = QGroupBox('Navigation Parameters')
        self.params_group.setCheckable(True)
        self.params_group.setChecked(False)  # Collapsed by default
        params_layout = QVBoxLayout(self.params_group)
        
        # Parameter controls will be added here
        self.setupParameterControls(params_layout)
        
        left_layout.addWidget(self.params_group)
        
        # Create right panel (map view)
        right_panel = QWidget()
        right_layout = QVBoxLayout(right_panel)
        self.map_view = MapView(self)
        right_layout.addWidget(self.map_view)
        
        # Add panels to splitter
        splitter.addWidget(left_panel)
        splitter.addWidget(right_panel)
        
        # Set initial splitter sizes (30% left, 70% right)
        splitter.setSizes([300, 700])
        
        # Connect signals
        self.add_waypoint_btn.clicked.connect(self.addWaypoint)
        self.edit_waypoint_btn.clicked.connect(self.editWaypoint)
        self.delete_waypoint_btn.clicked.connect(self.deleteWaypoint)
        self.navigate_waypoint_btn.clicked.connect(self.navigateToWaypoint)
        self.reset_view_btn.clicked.connect(self.resetMapView)
        self.waypoint_list.itemClicked.connect(self.waypointSelected)
        
        # Load default map
        self.loadDefaultMap()
        
    def loadSettings(self):
        # Load application settings
        settings = QSettings('B4M', 'WaypointManager')
        
        # Restore window geometry if available
        if settings.contains('geometry'):
            self.restoreGeometry(settings.value('geometry'))
        
    def closeEvent(self, event):
        # Save application settings when closing
        settings = QSettings('B4M', 'WaypointManager')
        settings.setValue('geometry', self.saveGeometry())
        super().closeEvent(event)
    
    def loadDefaultMap(self):
        # Get the maps directory
        maps_dir = self.ros_node.maps_dir
        
        # Check if directory exists
        if not os.path.exists(maps_dir):
            self.status_bar.showMessage(f"Maps directory not found: {maps_dir}")
            return
        
        # Use the default map
        default_map = "yahboom_map"
        map_file = f"{default_map}.yaml"
        map_path = os.path.join(maps_dir, map_file)
        
        if not os.path.exists(map_path):
            self.status_bar.showMessage(f"Default map not found: {map_path}")
            return
            
        self.status_bar.showMessage(f"Loading default map: {default_map}")
        self.loadMap(default_map)
    
    def loadMap(self, map_name):
        # Get the map file path
        maps_dir = self.ros_node.maps_dir
        map_file = os.path.join(maps_dir, f"{map_name}.yaml")
        
        # Check if file exists
        if not os.path.exists(map_file):
            self.status_bar.showMessage(f"Map file not found: {map_file}")
            return
        
        # Load the map metadata
        try:
            with open(map_file, 'r') as f:
                map_data = yaml.safe_load(f)
                
            # Get the image file path
            image_file = os.path.join(os.path.dirname(map_file), map_data['image'])
            
            # Check if image file exists
            if not os.path.exists(image_file):
                self.status_bar.showMessage(f"Map image not found: {image_file}")
                return
                
            # Load the image with enhanced handling for different formats
            image = Image.open(image_file)
            
            # Debug: Log image information
            self.ros_node.get_logger().info(f'Loaded image: {image_file}')
            self.ros_node.get_logger().info(f'Image mode: {image.mode}, size: {image.size}')
            
            # Handle different image modes appropriately
            if image.mode == 'P':
                # Palette mode - convert to RGB to handle properly
                image = image.convert('RGB')
            elif image.mode == 'L':
                # Grayscale - convert to RGB for consistent handling
                image = image.convert('RGB')
            elif image.mode == '1':
                # Monochrome - convert to RGB
                image = image.convert('RGB')
            elif image.mode != 'RGB':
                # Any other mode - convert to RGB
                image = image.convert('RGB')
            
            # Verify image is not corrupted or empty
            if image.width == 0 or image.height == 0:
                self.status_bar.showMessage(f"Invalid image dimensions: {image.width}x{image.height}")
                return
                
            # Convert PIL image to QImage with proper format
            image_data = image.tobytes()
            qimage = QImage(image_data, image.width, image.height, QImage.Format_RGB888)
            
            # Verify QImage is valid
            if qimage.isNull():
                self.status_bar.showMessage("Failed to create QImage from map data")
                return
                
            pixmap = QPixmap.fromImage(qimage)
            
            # Verify pixmap is valid
            if pixmap.isNull():
                self.status_bar.showMessage("Failed to create QPixmap from QImage")
                return
            
            # Extract and validate map metadata
            resolution = map_data.get('resolution', 0.05)  # Default to 5cm/pixel
            origin = map_data.get('origin', [0, 0, 0])      # Default origin
            
            # Log map metadata for debugging
            self.ros_node.get_logger().info(f'Map resolution: {resolution} meters/pixel')
            self.ros_node.get_logger().info(f'Map origin: {origin}')
            self.ros_node.get_logger().info(f'Map image size: {image.width}x{image.height} pixels')
            
            # Calculate map bounds for debugging
            map_width_meters = image.width * resolution
            map_height_meters = image.height * resolution
            self.ros_node.get_logger().info(f'Map size: {map_width_meters:.2f}x{map_height_meters:.2f} meters')
            
            # Set the map in the map view
            self.map_view.setMap(pixmap, resolution, origin)
            
            # Test coordinate conversion for debugging
            self.map_view.testCoordinateConversion()
            
            # Update the ROS node with the current map
            self.ros_node.set_current_map(map_name)
            
            # Update waypoint list
            self.updateWaypointList()
            
            # Update map label
            self.map_label.setText(f"Map: {map_name}")
            
            # Update status
            self.status_bar.showMessage(f"Loaded map: {map_name}")
        except Exception as e:
            self.status_bar.showMessage(f"Error loading map: {str(e)}")
            traceback.print_exc()
    
    def updateWaypointList(self):
        # Update the waypoint list for the current map
        self.waypoint_list.clear()
        
        # Get waypoints for current map
        if self.ros_node.current_map not in self.ros_node.waypoints:
            return
        
        # Get waypoints for current map and ensure it's a dictionary
        map_waypoints = self.ros_node.waypoints[self.ros_node.current_map]
        if not isinstance(map_waypoints, dict):
            self.get_logger().warn(f'Waypoints for map {self.ros_node.current_map} is not a dictionary')
            return
        
        # Add waypoints to list with coordinates
        for name in sorted(map_waypoints.keys()):
            waypoint = map_waypoints[name]
            if 'position' in waypoint and isinstance(waypoint['position'], dict):
                pos_x = waypoint['position'].get('x', 0.0)
                pos_y = waypoint['position'].get('y', 0.0)
                # Create a list item with the waypoint name and coordinates
                item = QListWidgetItem()
                # Display coordinates in parentheses without HTML formatting
                item.setText(f"{name} ({pos_x:.2f}, {pos_y:.2f})")
                # Store the original name for reference
                item.setData(Qt.UserRole, name)
                self.waypoint_list.addItem(item)
            else:
                # Fallback for waypoints without valid position data
                item = QListWidgetItem(name)
                item.setData(Qt.UserRole, name)
                self.waypoint_list.addItem(item)
        
        # Refresh the map view to show updated waypoints
        self.map_view.update()
        
    def addWaypoint(self):
        # Add a new waypoint
        self.ros_node.get_logger().info('Add waypoint requested')
        self.status_bar.showMessage('Click on the map to place a waypoint')
        self.map_view.setAddWaypointMode(True)
    
    def editWaypoint(self):
        # Edit the selected waypoint
        self.ros_node.get_logger().info('Edit waypoint requested')
        
        # Check if a waypoint is selected
        selected_items = self.waypoint_list.selectedItems()
        if not selected_items:
            self.status_bar.showMessage('No waypoint selected')
            return
        
        # Get selected waypoint name
        waypoint_name = selected_items[0].text()
        
        # Enable map click mode for editing
        self.status_bar.showMessage(f'Click on the map to set new position for {waypoint_name}')
        self.map_view.setEditWaypointMode(True, waypoint_name)
    
    def deleteWaypoint(self):
        # Delete the selected waypoint
        self.ros_node.get_logger().info('Delete waypoint requested')
        
        # Check if a waypoint is selected
        selected_items = self.waypoint_list.selectedItems()
        if not selected_items:
            self.status_bar.showMessage('No waypoint selected')
            return
        
        # Get selected waypoint name from UserRole data if available
        item = selected_items[0]
        waypoint_name = item.data(Qt.UserRole) if item.data(Qt.UserRole) else item.text()
        
        # Confirm deletion
        from PyQt5.QtWidgets import QMessageBox
        reply = QMessageBox.question(
            self, 'Confirm Deletion',
            f'Are you sure you want to delete waypoint {waypoint_name}?',
            QMessageBox.Yes | QMessageBox.No, QMessageBox.No
        )
        
        if reply == QMessageBox.Yes:
            # Delete waypoint
            if self.ros_node.delete_waypoint(self.ros_node.current_map, waypoint_name):
                self.updateWaypointList()
                self.status_bar.showMessage(f'Deleted waypoint: {waypoint_name}')
                # Refresh the map view to reflect the deletion
                self.map_view.update()
            else:
                self.status_bar.showMessage(f'Failed to delete waypoint: {waypoint_name}')
    
    def waypointSelected(self, item):
        # Handle waypoint selection
        # Get the original waypoint name from UserRole data if available
        waypoint_name = item.data(Qt.UserRole) if item.data(Qt.UserRole) else item.text()
        
        self.ros_node.get_logger().info(f'Waypoint selected: {waypoint_name}')
        
        # Update map view to highlight selected waypoint
        self.map_view.selectWaypoint(waypoint_name)
        
        # Enable edit, delete, and navigate buttons
        self.edit_waypoint_btn.setEnabled(True)
        self.delete_waypoint_btn.setEnabled(True)
        self.navigate_waypoint_btn.setEnabled(True)
        
        # Store the selected waypoint name
        self.selected_waypoint = waypoint_name
        
        # Show waypoint details in status bar
        if self.ros_node.current_map in self.ros_node.waypoints and waypoint_name in self.ros_node.waypoints[self.ros_node.current_map]:
            waypoint = self.ros_node.waypoints[self.ros_node.current_map][waypoint_name]
            pos_x = waypoint['position']['x']
            pos_y = waypoint['position']['y']
            self.status_bar.showMessage(f'Selected waypoint: {waypoint_name} at ({pos_x:.2f}, {pos_y:.2f})')
    
    def navigateToWaypoint(self):
        """Send MQTT command to navigate to the selected waypoint
        
        Uses coordinate-based navigation commands in JSON format.
        """
        if hasattr(self, 'selected_waypoint') and self.selected_waypoint:
            # Show status message
            self.status_bar.showMessage(f'Navigating to waypoint: {self.selected_waypoint}...')
            
            # Get the current map and waypoint data
            current_map = self.ros_node.current_map
            waypoint_name = self.selected_waypoint
            
            # Check if waypoint exists
            if current_map not in self.ros_node.waypoints or waypoint_name not in self.ros_node.waypoints[current_map]:
                self.status_bar.showMessage(f'Cannot navigate to waypoint: {waypoint_name} not found in map {current_map}')
                QMessageBox.warning(self, 'Navigation Error', 
                                f'Waypoint {waypoint_name} not found in map {current_map}')
                return
            
            # Get waypoint data
            waypoint = self.ros_node.waypoints[current_map][waypoint_name]
            
            # Create coordinate-based navigation command
            command = {
                "command": "goto",
                "waypoint_id": waypoint_name,
                "position": waypoint["position"],
                "orientation": waypoint["orientation"]
            }
            
            # Convert to JSON
            import json
            command_json = json.dumps(command)
            
            # Send navigation command via MQTT
            try:
                # Get MQTT client from ros_node
                if not self.ros_node.mqtt_connected:
                    self.status_bar.showMessage('Cannot navigate to waypoint: not connected to MQTT broker')
                    QMessageBox.warning(self, 'Navigation Error', 'Not connected to MQTT broker')
                    return
                
                # Publish directly to MQTT
                full_topic = f"{self.ros_node.mqtt_topic_prefix}/navigation/command"
                result = self.ros_node.mqtt_client.publish(full_topic, command_json)
                
                if result.rc == mqtt.MQTT_ERR_SUCCESS:
                    self.status_bar.showMessage(f'Navigation command sent for waypoint: {self.selected_waypoint}')
                    print(f'Published coordinate-based navigation command to {full_topic}: {command_json}')
                else:
                    self.status_bar.showMessage(f'Failed to send navigation command for waypoint: {self.selected_waypoint}')
                    QMessageBox.warning(self, 'Navigation Error', 
                                    'Failed to send navigation command. Check MQTT connection and robot status.')
            except Exception as e:
                self.status_bar.showMessage(f'Error sending navigation command: {str(e)}')
                QMessageBox.warning(self, 'Navigation Error', 
                                f'Error sending navigation command: {str(e)}')
                import traceback
                traceback.print_exc()
        else:
            self.status_bar.showMessage('No waypoint selected for navigation')
    
    def updateMqttStatus(self, connected):
        """Update the UI to reflect MQTT connection status"""
        if connected:
            self.status_bar.showMessage('MQTT connected', 3000)  # Show for 3 seconds
            if hasattr(self, 'selected_waypoint') and self.selected_waypoint:
                self.navigate_waypoint_btn.setEnabled(True)
        else:
            self.status_bar.showMessage('MQTT disconnected', 3000)  # Show for 3 seconds
            self.navigate_waypoint_btn.setEnabled(False)
    
    def resetMapView(self):
        """Reset the map view to center and fit the map"""
        self.map_view.resetView()
        self.status_bar.showMessage('Map view reset to center')
    
    def setupParameterControls(self, layout):
        """Setup the parameter adjustment controls"""
        
        # Load parameters button
        load_params_btn = QPushButton('Reload Parameters from File')
        load_params_btn.clicked.connect(self.loadParameters)
        layout.addWidget(load_params_btn)
        
        # Parameter scroll area
        scroll_area = QScrollArea()
        scroll_area.setWidgetResizable(True)
        scroll_area.setMaximumHeight(400)
        
        self.params_widget = QWidget()
        self.params_layout = QVBoxLayout(self.params_widget)
        scroll_area.setWidget(self.params_widget)
        
        layout.addWidget(scroll_area)
        
        # Control buttons
        buttons_layout = QHBoxLayout()
        
        apply_btn = QPushButton('Apply Parameters')
        apply_btn.setStyleSheet('background-color: #4CAF50; color: white; font-weight: bold;')
        apply_btn.clicked.connect(self.applyParameters)
        
        self.build_btn = QPushButton('Build Required')
        self.build_btn.setStyleSheet('background-color: #FF9800; color: white; font-weight: bold;')
        self.build_btn.clicked.connect(self.buildPackage)
        self.build_btn.hide()  # Hidden until build is required
        
        reset_btn = QPushButton('Reset to Defaults')
        reset_btn.clicked.connect(self.resetParameters)
        
        buttons_layout.addWidget(apply_btn)
        buttons_layout.addWidget(self.build_btn)
        buttons_layout.addWidget(reset_btn)
        
        layout.addLayout(buttons_layout)
        
        # Build progress bar
        self.build_progress = QProgressBar()
        self.build_progress.hide()
        layout.addWidget(self.build_progress)
        
        # Load parameters on startup
        self.loadParameters()
    
    def loadParameters(self):
        """Load navigation parameters from YAML file"""
        try:
            params_file = "/home/yahboom/b4m_yahboom/yahboomcar_nav/params/dwb_nav_params.yaml"
            
            if not os.path.exists(params_file):
                self.status_bar.showMessage(f"Parameters file not found: {params_file}")
                return
            
            with open(params_file, 'r') as f:
                self.parameters = yaml.safe_load(f)
            
            self.createParameterWidgets()
            self.status_bar.showMessage('Parameters loaded successfully')
            
        except Exception as e:
            self.status_bar.showMessage(f'Error loading parameters: {str(e)}')
            self.ros_node.get_logger().error(f'Error loading parameters: {e}')
    
    def createParameterWidgets(self):
        """Create UI widgets for parameter editing"""
        # Clear existing widgets
        for i in reversed(range(self.params_layout.count())):
            self.params_layout.itemAt(i).widget().setParent(None)
        
        self.parameter_widgets.clear()
        
        if not self.parameters:
            return
        
        # Define parameter categories based on B4M_nav_adjustments.md
        param_categories = {
            'AMCL Parameters': [
                'alpha1', 'alpha2', 'alpha3', 'alpha4', 'alpha5',
                'max_particles', 'do_beamskip', 'transform_tolerance',
                'update_min_a', 'update_min_d', 'pf_err',
                'recovery_alpha_fast', 'recovery_alpha_slow',
                'z_hit', 'z_rand'
            ],
            'Navigation Controller': [
                'required_movement_radius', 'movement_time_allowance',
                'xy_goal_tolerance', 'yaw_goal_tolerance'
            ],
            'Other Parameters': []  # Will contain any parameters not in above categories
        }
        
        # Process parameters by category
        for category_name, param_names in param_categories.items():
            category_group = QGroupBox(category_name)
            category_group.setCheckable(True)
            category_group.setChecked(False)  # Collapsed by default
            category_layout = QVBoxLayout(category_group)
            
            category_has_params = False
            
            # Look for parameters in this category
            for param_name in param_names:
                widget = self.findAndCreateParameterWidget(param_name, self.parameters)
                if widget:
                    category_layout.addWidget(widget)
                    category_has_params = True
            
            # Add category to layout if it has parameters
            if category_has_params:
                self.params_layout.addWidget(category_group)
        
        # Handle uncategorized parameters
        other_params = self.findUncategorizedParameters(param_categories)
        if other_params:
            other_group = QGroupBox('Other Parameters')
            other_group.setCheckable(True)
            other_group.setChecked(False)
            other_layout = QVBoxLayout(other_group)
            
            for param_path, value in other_params:
                widget = self.createParameterWidget(param_path, value)
                if widget:
                    other_layout.addWidget(widget)
            
            self.params_layout.addWidget(other_group)
    
    def findAndCreateParameterWidget(self, param_name, params_dict, parent_path=""):
        """Recursively find and create widget for a parameter"""
        for key, value in params_dict.items():
            current_path = f"{parent_path}.{key}" if parent_path else key
            
            if key == param_name:
                return self.createParameterWidget(current_path, value)
            elif isinstance(value, dict):
                widget = self.findAndCreateParameterWidget(param_name, value, current_path)
                if widget:
                    return widget
        return None
    
    def findUncategorizedParameters(self, categories):
        """Find parameters not in any category"""
        categorized_params = set()
        for param_list in categories.values():
            categorized_params.update(param_list)
        
        uncategorized = []
        self.collectAllParameters(self.parameters, "", categorized_params, uncategorized)
        return uncategorized
    
    def collectAllParameters(self, params_dict, parent_path, categorized_params, uncategorized):
        """Recursively collect all parameters"""
        for key, value in params_dict.items():
            current_path = f"{parent_path}.{key}" if parent_path else key
            
            if isinstance(value, (int, float, bool, str)):
                if key not in categorized_params:
                    uncategorized.append((current_path, value))
            elif isinstance(value, dict):
                self.collectAllParameters(value, current_path, categorized_params, uncategorized)
    
    def createParameterWidget(self, param_path, value):
        """Create appropriate widget for parameter value"""
        container = QWidget()
        layout = QHBoxLayout(container)
        layout.setContentsMargins(0, 0, 0, 0)
        
        # Parameter name label
        name_label = QLabel(param_path.split('.')[-1])
        name_label.setMinimumWidth(150)
        layout.addWidget(name_label)
        
        # Value widget based on type
        if isinstance(value, bool):
            widget = QCheckBox()
            widget.setChecked(value)
            widget.stateChanged.connect(self.parameterChanged)
        elif isinstance(value, int):
            widget = QSpinBox()
            widget.setRange(-999999, 999999)
            widget.setValue(value)
            widget.valueChanged.connect(self.parameterChanged)
        elif isinstance(value, float):
            widget = QDoubleSpinBox()
            widget.setRange(-999999.0, 999999.0)
            widget.setDecimals(4)
            widget.setValue(value)
            widget.valueChanged.connect(self.parameterChanged)
        else:  # string
            from PyQt5.QtWidgets import QLineEdit
            widget = QLineEdit()
            widget.setText(str(value))
            widget.textChanged.connect(self.parameterChanged)
        
        layout.addWidget(widget)
        
        # Store widget reference
        self.parameter_widgets[param_path] = widget
        
        return container
    
    def parameterChanged(self):
        """Handle parameter value changes"""
        self.parameters_modified = True
        self.status_bar.showMessage('Parameters modified - click Apply to update')
    
    def applyParameters(self):
        """Apply parameters to running system or mark for build"""
        if not self.parameters_modified:
            self.status_bar.showMessage('No parameters to apply')
            return
        
        try:
            # Collect changed parameters
            changed_params = []
            for param_path, widget in self.parameter_widgets.items():
                if isinstance(widget, QCheckBox):
                    value = widget.isChecked()
                elif isinstance(widget, (QSpinBox, QDoubleSpinBox)):
                    value = widget.value()
                else:  # QLineEdit
                    value = widget.text()
                
                changed_params.append((param_path, value))
            
            # Try to apply parameters at runtime first
            runtime_success = self.applyParametersRuntime(changed_params)
            
            if runtime_success:
                self.status_bar.showMessage('Parameters applied successfully')
                self.parameters_modified = False
            else:
                # Mark for build if runtime application failed
                self.build_required = True
                self.build_btn.show()
                self.status_bar.showMessage('Parameters changed - build required for full effect')
        
        except Exception as e:
            self.status_bar.showMessage(f'Error applying parameters: {str(e)}')
            self.ros_node.get_logger().error(f'Error applying parameters: {e}')
    
    def applyParametersRuntime(self, changed_params):
        """Try to apply parameters to running nodes using ros2 param set"""
        try:
            # Check if navigation nodes are running
            result = subprocess.run(['ros2', 'node', 'list'], 
                                 capture_output=True, text=True, timeout=5)
            
            if result.returncode != 0:
                return False
            
            running_nodes = result.stdout.strip().split('\n')
            nav_nodes = [node for node in running_nodes if any(nav_name in node.lower() 
                        for nav_name in ['amcl', 'controller', 'planner', 'nav'])]
            
            if not nav_nodes:
                self.ros_node.get_logger().info('No navigation nodes running for runtime parameter updates')
                return False
            
            # Apply parameters to relevant nodes
            success_count = 0
            for param_path, value in changed_params:
                for node in nav_nodes:
                    try:
                        # Convert parameter path for ros2 param set
                        param_name = param_path.split('.')[-1]
                        
                        cmd = ['ros2', 'param', 'set', node, param_name, str(value)]
                        result = subprocess.run(cmd, capture_output=True, text=True, timeout=5)
                        
                        if result.returncode == 0:
                            success_count += 1
                            self.ros_node.get_logger().info(f'Set {param_name}={value} on {node}')
                        
                    except subprocess.TimeoutExpired:
                        continue
                    except Exception as e:
                        self.ros_node.get_logger().debug(f'Failed to set {param_name} on {node}: {e}')
                        continue
            
            return success_count > 0
            
        except Exception as e:
            self.ros_node.get_logger().error(f'Error in runtime parameter application: {e}')
            return False
    
    def buildPackage(self):
        """Build the navigation package with updated parameters"""
        try:
            self.build_btn.setEnabled(False)
            self.build_progress.show()
            self.build_progress.setRange(0, 0)  # Indeterminate progress
            
            # Update YAML file with current parameter values
            self.saveParametersToFile()
            
            # Start build in separate thread
            self.build_thread = threading.Thread(target=self.runBuildProcess)
            self.build_thread.daemon = True
            self.build_thread.start()
            
        except Exception as e:
            self.status_bar.showMessage(f'Error starting build: {str(e)}')
            self.build_btn.setEnabled(True)
            self.build_progress.hide()
    
    def runBuildProcess(self):
        """Run colcon build process in background thread"""
        try:
            # Change to workspace directory and run colcon build
            workspace_dir = "/home/yahboom/b4m_yahboom"
            cmd = ["colcon", "build", "--packages-select", "yahboomcar_nav"]
            
            process = subprocess.Popen(
                cmd, 
                cwd=workspace_dir,
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                text=True
            )
            
            stdout, stderr = process.communicate()
            
            # Update UI from main thread
            if process.returncode == 0:
                self.status_bar.showMessage('Build completed successfully')
                self.build_required = False
                self.parameters_modified = False
            else:
                self.status_bar.showMessage(f'Build failed: {stderr}')
                
        except Exception as e:
            self.status_bar.showMessage(f'Build error: {str(e)}')
        
        finally:
            # Hide progress and re-enable button
            self.build_progress.hide()
            self.build_btn.setEnabled(True)
            if not self.build_required:
                self.build_btn.hide()
    
    def saveParametersToFile(self):
        """Save current parameter values to YAML file"""
        try:
            # Update parameters dict with current widget values
            for param_path, widget in self.parameter_widgets.items():
                if isinstance(widget, QCheckBox):
                    value = widget.isChecked()
                elif isinstance(widget, (QSpinBox, QDoubleSpinBox)):
                    value = widget.value()
                else:  # QLineEdit
                    value = widget.text()
                    try:
                        # Try to convert to appropriate type
                        if '.' in value:
                            value = float(value)
                        else:
                            value = int(value)
                    except:
                        pass  # Keep as string
                
                # Update nested dict structure
                self.setNestedValue(self.parameters, param_path, value)
            
            # Write updated parameters to file
            params_file = "/home/yahboom/b4m_yahboom/yahboomcar_nav/params/dwb_nav_params.yaml"
            with open(params_file, 'w') as f:
                yaml.dump(self.parameters, f, default_flow_style=False, indent=2)
                
            self.ros_node.get_logger().info('Parameters saved to YAML file')
            
        except Exception as e:
            self.ros_node.get_logger().error(f'Error saving parameters: {e}')
    
    def setNestedValue(self, dictionary, path, value):
        """Set value in nested dictionary using dot notation path"""
        keys = path.split('.')
        current = dictionary
        
        for key in keys[:-1]:
            if key not in current:
                current[key] = {}
            current = current[key]
        
        current[keys[-1]] = value
    
    def resetParameters(self):
        """Reset parameters to defaults"""
        reply = QMessageBox.question(
            self, 'Reset Parameters',
            'Are you sure you want to reset all parameters to defaults?',
            QMessageBox.Yes | QMessageBox.No, QMessageBox.No
        )
        
        if reply == QMessageBox.Yes:
            # Reload parameters from file
            self.loadParameters()
            self.parameters_modified = False
            self.status_bar.showMessage('Parameters reset to defaults')
    
    def saveConfiguration(self):
        """Save complete configuration (parameters + map + waypoints)"""
        try:
            # Get filename from user
            filename, _ = QFileDialog.getSaveFileName(
                self, 'Save Configuration', 
                '/home/yahboom/b4m_yahboom/configs/',
                'JSON files (*.json)'
            )
            
            if not filename:
                return
            
            # Automatically add .json extension if not present
            if not filename.lower().endswith('.json'):
                filename += '.json'
            
            # Ensure directory exists
            os.makedirs(os.path.dirname(filename), exist_ok=True)
            
            # Collect current parameter values
            current_params = {}
            for param_path, widget in self.parameter_widgets.items():
                if isinstance(widget, QCheckBox):
                    value = widget.isChecked()
                elif isinstance(widget, (QSpinBox, QDoubleSpinBox)):
                    value = widget.value()
                else:  # QLineEdit
                    value = widget.text()
                current_params[param_path] = value
            
            # Create configuration data
            config_data = {
                'timestamp': datetime.now().isoformat(),
                'description': f'Configuration saved from waypoint manager',
                'map_name': self.ros_node.current_map,
                'parameters': current_params,
                'waypoints': self.ros_node.waypoints,
                'version': '1.0'
            }
            
            # Save configuration
            with open(filename, 'w') as f:
                json.dump(config_data, f, indent=2)
            
            self.status_bar.showMessage(f'Configuration saved: {filename}')
            
        except Exception as e:
            self.status_bar.showMessage(f'Error saving configuration: {str(e)}')
            self.ros_node.get_logger().error(f'Error saving configuration: {e}')
    
    def loadConfiguration(self):
        """Load complete configuration (parameters + map + waypoints)"""
        try:
            # Get filename from user
            filename, _ = QFileDialog.getOpenFileName(
                self, 'Load Configuration',
                '/home/yahboom/b4m_yahboom/configs/',
                'JSON files (*.json)'
            )
            
            if not filename:
                return
            
            # Load configuration
            with open(filename, 'r') as f:
                config_data = json.load(f)
            
            # Confirm with user
            reply = QMessageBox.question(
                self, 'Load Configuration',
                f'Load configuration from {os.path.basename(filename)}?\n'
                f'Created: {config_data.get("timestamp", "unknown")}\n'
                f'Description: {config_data.get("description", "No description")}\n\n'
                f'This will replace current parameters, map, and waypoints.',
                QMessageBox.Yes | QMessageBox.No, QMessageBox.No
            )
            
            if reply != QMessageBox.Yes:
                return
            
            # Load parameters
            if 'parameters' in config_data:
                for param_path, value in config_data['parameters'].items():
                    if param_path in self.parameter_widgets:
                        widget = self.parameter_widgets[param_path]
                        if isinstance(widget, QCheckBox):
                            widget.setChecked(value)
                        elif isinstance(widget, (QSpinBox, QDoubleSpinBox)):
                            widget.setValue(value)
                        else:  # QLineEdit
                            widget.setText(str(value))
            
            # Load waypoints
            if 'waypoints' in config_data:
                self.ros_node.waypoints = config_data['waypoints']
                self.ros_node.save_waypoints()
                self.updateWaypointList()
            
            # Load map
            if 'map_name' in config_data:
                map_name = config_data['map_name']
                if map_name != self.ros_node.current_map:
                    self.loadMap(map_name)
            
            self.parameters_modified = True
            self.status_bar.showMessage(f'Configuration loaded: {os.path.basename(filename)}')
            
        except Exception as e:
            self.status_bar.showMessage(f'Error loading configuration: {str(e)}')
            self.ros_node.get_logger().error(f'Error loading configuration: {e}')

class MapView(QWidget):
    def __init__(self, parent):
        super().__init__(parent)
        self.parent = parent
        self.map_image = None
        self.map_resolution = 0.05  # Default map resolution (meters/pixel)
        self.map_origin = (0, 0)   # Default map origin (x, y) in meters
        self.waypoints = {}        # Dictionary to store waypoints
        self.selected_waypoint = None
        self.add_waypoint_mode = False
        self.edit_waypoint_mode = False
        self.edit_waypoint_name = None
        self.scale_factor = 1.0    # Scale factor for map display
        self.map_offset_x = 0      # Offset for map display
        self.map_offset_y = 0      # Offset for map display
        
        # For panning support
        self.panning = False
        self.pan_start_x = 0
        self.pan_start_y = 0
        
        # Enable mouse tracking for panning
        self.setMouseTracking(True)
        
        # Enable focus to receive keyboard events
        self.setFocusPolicy(Qt.StrongFocus)
        
        # Set background color
        self.setAutoFillBackground(True)
        palette = self.palette()
        palette.setColor(self.backgroundRole(), QColor(240, 240, 240))
        self.setPalette(palette)
    
    def setMap(self, pixmap, resolution, origin):
        """Set the map image and properties"""
        self.map_image = pixmap
        self.map_resolution = resolution
        self.map_origin = origin
        self.scale_factor = 1.0
        self.map_offset_x = 0
        self.map_offset_y = 0
        
        # Reset view to center and fit the map
        self.resetView()
        
        self.update()  # Trigger repaint
    
    def setWaypoints(self, waypoints):
        """Set the waypoints to display"""
        self.waypoints = waypoints
        self.update()  # Trigger repaint
    
    def selectWaypoint(self, name):
        """Select a waypoint by name"""
        self.selected_waypoint = name
        self.update()  # Trigger repaint
    
    def setAddWaypointMode(self, enabled):
        """Enable or disable add waypoint mode"""
        self.add_waypoint_mode = enabled
        self.edit_waypoint_mode = False
        self.edit_waypoint_name = None
        if enabled:
            self.setCursor(Qt.CrossCursor)
        else:
            self.setCursor(Qt.ArrowCursor)
            
    def setEditWaypointMode(self, enabled, waypoint_name=None):
        """Enable or disable edit waypoint mode"""
        self.edit_waypoint_mode = enabled
        self.edit_waypoint_name = waypoint_name if enabled else None
        self.add_waypoint_mode = False
        if enabled:
            self.setCursor(Qt.CrossCursor)
        else:
            self.setCursor(Qt.ArrowCursor)
    
    def mousePressEvent(self, event):
        if self.map_image:
            if event.button() == Qt.LeftButton:
                # Convert click position to map coordinates
                x, y = self.pixelToMapCoordinates(event.x(), event.y())
                
                if self.add_waypoint_mode:
                    # Handle adding a new waypoint
                    # Create a dialog for waypoint name input
                    from PyQt5.QtWidgets import QInputDialog
                    
                    # Generate a suggested name
                    existing_waypoints = self.parent.ros_node.waypoints.get(self.parent.ros_node.current_map, {})
                    suggested_name = self.generateUniqueWaypointName(existing_waypoints)
                    
                    name, ok = QInputDialog.getText(
                        self, 'New Waypoint', 'Enter waypoint name:', 
                        text=suggested_name
                    )
                    
                    if ok and name:
                        # Ensure name is unique
                        if name in existing_waypoints:
                            QMessageBox.warning(
                                self, 'Duplicate Name', 
                                f'Waypoint name "{name}" already exists. Please choose a different name.'
                            )
                        else:
                            # Add the waypoint
                            self.parent.ros_node.add_waypoint(self.parent.ros_node.current_map, name, x, y)
                            self.parent.updateWaypointList()
                            self.parent.status_bar.showMessage(f'Added waypoint: {name}')
                            
                            self.update()  # Redraw the map view to show the new waypoint
                    
                    # Exit add waypoint mode
                    self.add_waypoint_mode = False
                    self.setCursor(Qt.ArrowCursor)
                    
                elif self.edit_waypoint_mode and self.edit_waypoint_name:
                    # Handle editing an existing waypoint
                    # Update the waypoint position
                    if self.parent.ros_node.edit_waypoint(self.parent.ros_node.current_map, self.edit_waypoint_name, x, y):
                        self.parent.status_bar.showMessage(f'Updated waypoint: {self.edit_waypoint_name}')
                        self.update()  # Redraw the map view
                    else:
                        self.parent.status_bar.showMessage(f'Failed to update waypoint: {self.edit_waypoint_name}')
                    
                    # Exit edit waypoint mode
                    self.edit_waypoint_mode = False
                    self.edit_waypoint_name = None
                    self.setCursor(Qt.ArrowCursor)
                else:
                    # Start panning mode
                    self.panning = True
                    self.pan_start_x = event.x()
                    self.pan_start_y = event.y()
                    self.setCursor(Qt.ClosedHandCursor)
            
            elif event.button() == Qt.RightButton:
                # Right-click to cancel add/edit mode
                if self.add_waypoint_mode or self.edit_waypoint_mode:
                    self.add_waypoint_mode = False
                    self.edit_waypoint_mode = False
                    self.edit_waypoint_name = None
                    self.setCursor(Qt.ArrowCursor)
                    self.parent.status_bar.showMessage('Operation cancelled')
    
    def mouseMoveEvent(self, event):
        if self.map_image and self.panning and event.buttons() & Qt.LeftButton:
            # Calculate the distance moved
            dx = event.x() - self.pan_start_x
            dy = event.y() - self.pan_start_y
            
            # Update the map offset
            self.map_offset_x += dx
            self.map_offset_y += dy
            
            # Update the pan start position
            self.pan_start_x = event.x()
            self.pan_start_y = event.y()
            
            # Redraw the map
            self.update()
    
    def mouseReleaseEvent(self, event):
        if event.button() == Qt.LeftButton and self.panning:
            # End panning mode
            self.panning = False
            self.setCursor(Qt.ArrowCursor)
    
    def generateUniqueWaypointName(self, existing_waypoints):
        """Generate a unique waypoint name"""
        base_name = "Waypoint"
        counter = 1
        while f"{base_name} {counter}" in existing_waypoints:
            counter += 1
        return f"{base_name} {counter}"
    
    def pixelToMapCoordinates(self, pixel_x, pixel_y):
        """Convert pixel coordinates to map coordinates using ROS map conventions
        
        ROS map coordinate system:
        - Origin (0,0) is typically at the bottom-left of the map in meters
        - X-axis points right (east)
        - Y-axis points up (north)
        
        Image coordinate system:
        - Origin (0,0) is at the top-left of the image in pixels
        - X-axis points right
        - Y-axis points down
        """
        if not self.map_image:
            return 0, 0
            
        # Adjust for scale and offset to get image pixel coordinates
        image_x = (pixel_x - self.map_offset_x) / self.scale_factor
        image_y = (pixel_y - self.map_offset_y) / self.scale_factor
        
        # Clamp to image bounds to prevent invalid coordinates
        image_x = max(0, min(self.map_image.width() - 1, image_x))
        image_y = max(0, min(self.map_image.height() - 1, image_y))
        
        # Convert from image coordinates to map coordinates
        # ROS maps have origin at bottom-left, images have origin at top-left
        # So we need to flip the Y coordinate
        map_x = image_x * self.map_resolution + self.map_origin[0]
        map_y = (self.map_image.height() - 1 - image_y) * self.map_resolution + self.map_origin[1]
        
        return map_x, map_y
    
    def mapToPixelCoordinates(self, map_x, map_y):
        """Convert map coordinates to pixel coordinates using ROS map conventions
        
        This is the inverse of pixelToMapCoordinates
        """
        if not self.map_image:
            return 0, 0
            
        # Convert from map coordinates to image coordinates
        # First, convert map position to image pixel position
        image_x = (map_x - self.map_origin[0]) / self.map_resolution
        # Flip Y coordinate: ROS origin at bottom-left, image origin at top-left
        image_y = (self.map_image.height() - 1) - (map_y - self.map_origin[1]) / self.map_resolution
        
        # Clamp to image bounds
        image_x = max(0, min(self.map_image.width() - 1, image_x))
        image_y = max(0, min(self.map_image.height() - 1, image_y))
        
        # Apply scale and offset for display
        display_x = image_x * self.scale_factor + self.map_offset_x
        display_y = image_y * self.scale_factor + self.map_offset_y
        
        return display_x, display_y
    
    def testCoordinateConversion(self):
        """Test coordinate conversion functions for debugging
        
        This function tests the round-trip conversion from pixel to map coordinates
        and back to verify the coordinate transformation is working correctly.
        """
        if not self.map_image:
            return
            
        # Test several points
        test_points = [
            (0, 0),  # Top-left corner
            (self.map_image.width() - 1, 0),  # Top-right corner
            (0, self.map_image.height() - 1),  # Bottom-left corner
            (self.map_image.width() - 1, self.map_image.height() - 1),  # Bottom-right corner
            (self.map_image.width() // 2, self.map_image.height() // 2),  # Center
        ]
        
        print("=== Coordinate Conversion Test ===")
        print(f"Map image size: {self.map_image.width()}x{self.map_image.height()}")
        print(f"Map resolution: {self.map_resolution}")
        print(f"Map origin: {self.map_origin}")
        
        for i, (pixel_x, pixel_y) in enumerate(test_points):
            # Convert to map coordinates
            map_x, map_y = self.pixelToMapCoordinates(pixel_x, pixel_y)
            
            # Convert back to pixel coordinates
            back_pixel_x, back_pixel_y = self.mapToPixelCoordinates(map_x, map_y)
            
            print(f"Test {i+1}: Pixel({pixel_x}, {pixel_y}) -> Map({map_x:.3f}, {map_y:.3f}) -> Pixel({back_pixel_x:.1f}, {back_pixel_y:.1f})")
            
        print("=== End Coordinate Test ===\n")
    
    def resetView(self):
        """Reset the map view to center and fit the map in the widget"""
        if not self.map_image:
            return
            
        # Get widget dimensions
        widget_width = self.width()
        widget_height = self.height()
        
        if widget_width <= 0 or widget_height <= 0:
            return
            
        # Calculate scale to fit map in widget with some padding
        padding = 50  # pixels of padding
        scale_x = (widget_width - 2 * padding) / self.map_image.width()
        scale_y = (widget_height - 2 * padding) / self.map_image.height()
        
        # Use the smaller scale to ensure the entire map fits
        self.scale_factor = min(scale_x, scale_y, 1.0)  # Don't scale larger than 1:1
        
        # Center the map in the widget
        scaled_width = self.map_image.width() * self.scale_factor
        scaled_height = self.map_image.height() * self.scale_factor
        
        self.map_offset_x = (widget_width - scaled_width) / 2
        self.map_offset_y = (widget_height - scaled_height) / 2
        
        # Update the display
        self.update()
    
    def showEvent(self, event):
        """Called when the widget is shown - ensure map is properly centered"""
        super().showEvent(event)
        if self.map_image:
            # Small delay to ensure widget is properly sized
            QTimer.singleShot(100, self.resetView)
    
    def resizeEvent(self, event):
        """Called when the widget is resized - reset view to maintain centering"""
        super().resizeEvent(event)
        if self.map_image:
            self.resetView()
    
    def keyPressEvent(self, event):
        """Handle keyboard events"""
        if event.key() == Qt.Key_R:
            # R key to reset view
            self.resetView()
            if hasattr(self.parent, 'statusBar'):
                self.parent.status_bar.showMessage('Map view reset (R key)')
        elif event.key() == Qt.Key_T:
            # T key to test coordinate conversion
            self.testCoordinateConversion()
            if hasattr(self.parent, 'statusBar'):
                self.parent.status_bar.showMessage('Coordinate conversion test run (T key)')
        else:
            super().keyPressEvent(event)
    
    def paintEvent(self, event):
        painter = QPainter(self)
        painter.setRenderHint(QPainter.Antialiasing)
        
        # Draw map if available
        if self.map_image:
            # Draw the map with scaling and offset
            scaled_width = int(self.map_image.width() * self.scale_factor)
            scaled_height = int(self.map_image.height() * self.scale_factor)
            
            # Convert floating point offsets to integers to avoid type errors
            x_offset = int(self.map_offset_x)
            y_offset = int(self.map_offset_y)
            
            # Check if the map would be visible in the current view
            widget_rect = self.rect()
            map_rect = QRect(x_offset, y_offset, scaled_width, scaled_height)
            
            if widget_rect.intersects(map_rect):
                # Only draw if the map is at least partially visible
                painter.drawPixmap(
                    x_offset, y_offset, 
                    scaled_width, scaled_height,
                    self.map_image
                )
            else:
                # Draw a placeholder to indicate the map is off-screen
                painter.setPen(QColor(150, 150, 150))
                painter.drawText(self.rect(), Qt.AlignCenter, 
                               f'Map off-screen\nZoom: {self.scale_factor:.1f}x\nOffset: ({x_offset}, {y_offset})')
            
            # Draw waypoints
            if self.parent.ros_node.current_map in self.parent.ros_node.waypoints:
                waypoints = self.parent.ros_node.waypoints[self.parent.ros_node.current_map]
                for name, waypoint in waypoints.items():
                    # Get waypoint position
                    if 'position' in waypoint:
                        pos_x = waypoint['position']['x']
                        pos_y = waypoint['position']['y']
                        
                        # Convert to pixel coordinates
                        pixel_x, pixel_y = self.mapToPixelCoordinates(pos_x, pos_y)
                        
                        # Get waypoint visualization properties
                        vis = waypoint.get('visualization', {})
                        color = vis.get('color', {'r': 0.0, 'g': 0.0, 'b': 1.0})
                        scale = vis.get('scale', 0.3) * 20 * self.scale_factor  # Convert to pixels
                        
                        # Draw waypoint
                        painter.setPen(QPen(QColor(
                            int(color['r'] * 255),
                            int(color['g'] * 255),
                            int(color['b'] * 255)
                        ), 2))
                        
                        # Fill with semi-transparent color
                        painter.setBrush(QBrush(QColor(
                            int(color['r'] * 255),
                            int(color['g'] * 255),
                            int(color['b'] * 255),
                            128  # Alpha (semi-transparent)
                        )))
                        
                        # Draw circle for waypoint
                        painter.drawEllipse(
                            int(pixel_x - scale/2),
                            int(pixel_y - scale/2),
                            int(scale),
                            int(scale)
                        )
                        
                        # Highlight selected waypoint
                        if name == self.selected_waypoint:
                            painter.setPen(QPen(Qt.white, 2))
                            painter.setBrush(Qt.NoBrush)
                            painter.drawEllipse(
                                int(pixel_x - scale/2 - 3),
                                int(pixel_y - scale/2 - 3),
                                int(scale + 6),
                                int(scale + 6)
                            )
                        
                        # Draw waypoint name
                        painter.setPen(Qt.black)
                        painter.drawText(
                            int(pixel_x + scale/2 + 5),
                            int(pixel_y),
                            name
                        )
        else:
            # Draw placeholder text if no map is loaded
            painter.setPen(QColor(100, 100, 100))
            painter.drawText(self.rect(), Qt.AlignCenter, 'No map loaded')
    
    def wheelEvent(self, event):
        """Handle mouse wheel events for zooming"""
        try:
            if self.map_image:
                # Handle zoom in/out with mouse wheel
                zoom_factor = 1.2
                
                # Get the position of the mouse in the widget
                mouse_pos = event.pos()
                
                # Debug information
                if hasattr(self.parent, 'ros_node') and self.parent.ros_node:
                    self.parent.ros_node.get_logger().debug(f'Mouse wheel event at position: {mouse_pos.x()}, {mouse_pos.y()}')
                    self.parent.ros_node.get_logger().debug(f'Current scale factor: {self.scale_factor}')
                    self.parent.ros_node.get_logger().debug(f'Current offsets: {self.map_offset_x}, {self.map_offset_y}')
                
                # Calculate the mouse position relative to the map
                mouse_x = (mouse_pos.x() - self.map_offset_x) / self.scale_factor
                mouse_y = (mouse_pos.y() - self.map_offset_y) / self.scale_factor
                
                # Determine zoom direction
                delta = event.angleDelta().y()
                
                if delta > 0:
                    # Zoom in
                    self.scale_factor *= zoom_factor
                    if hasattr(self.parent, 'ros_node') and self.parent.ros_node:
                        self.parent.ros_node.get_logger().debug(f'Zooming in, new scale factor: {self.scale_factor}')
                else:
                    # Zoom out
                    self.scale_factor /= zoom_factor
                    if hasattr(self.parent, 'ros_node') and self.parent.ros_node:
                        self.parent.ros_node.get_logger().debug(f'Zooming out, new scale factor: {self.scale_factor}')
                
                # Limit zoom level
                self.scale_factor = max(0.1, min(10.0, self.scale_factor))
                
                # Adjust offset to keep the point under the mouse in the same place
                self.map_offset_x = mouse_pos.x() - mouse_x * self.scale_factor
                self.map_offset_y = mouse_pos.y() - mouse_y * self.scale_factor
                
                # Debug final values
                if hasattr(self.parent, 'ros_node') and self.parent.ros_node:
                    self.parent.ros_node.get_logger().debug(f'New offsets: {self.map_offset_x}, {self.map_offset_y}')
                
                # Update the view
                self.update()
                
                # Accept the event to prevent it from being passed to parent widgets
                event.accept()
        except Exception as e:
            print(f"\nERROR IN WHEEL EVENT: {e}")
            if hasattr(self.parent, 'ros_node') and self.parent.ros_node:
                self.parent.ros_node.get_logger().error(f'Error in wheel event: {e}')
            if hasattr(self.parent, 'statusBar'):
                self.parent.status_bar.showMessage(f'Zoom error: {e}')
            import traceback
            traceback.print_exc()
            print("\nStack trace printed above. Please check for any error messages.")
            sys.stdout.flush()  # Force output to be displayed immediately


class WaypointManager(Node):
    def __init__(self):
        try:
            super().__init__('waypoint_manager')
            self.get_logger().info('Waypoint Manager starting...')
            
            # Get parameters
            self.declare_parameter('connected_mode', False)
            self.connected_mode = self.get_parameter('connected_mode').value
            
            # MQTT parameters
            self.declare_parameter('mqtt_broker', 'localhost')
            self.declare_parameter('mqtt_port', 1883)
            self.declare_parameter('mqtt_topic_prefix', 'yahboom')
            
            self.mqtt_broker = self.get_parameter('mqtt_broker').value
            self.mqtt_port = self.get_parameter('mqtt_port').value
            self.mqtt_topic_prefix = self.get_parameter('mqtt_topic_prefix').value
            self.mqtt_client = None
            self.mqtt_connected = False
            
            # Initialize waypoints dictionary for the default map
            self.waypoints = {"yahboom_map": {}}
            self.current_map = "yahboom_map"
            
            # Set maps directory
            self.maps_dir = os.path.join(get_package_share_directory('yahboomcar_nav'), 'maps')
            self.get_logger().info(f'Maps directory: {self.maps_dir}')
            
            # Store waypoints in the repository root
            self.pkg_share = get_package_share_directory('b4m_waypoint_nav')
            self.repo_root = os.path.abspath(os.path.join(self.pkg_share, '..', '..'))
            self.waypoints_file = os.path.join(self.repo_root, 'waypoints.json')
            
            # Load waypoints
            try:
                self.load_waypoints()
            except Exception as e:
                print(f"Error loading waypoints: {e}")
                import traceback
                traceback.print_exc()
                # Reset to empty waypoints
                self.waypoints = {"yahboom_map": {}}
                
            # Setup MQTT client
            self.setup_mqtt_client()
        except Exception as e:
            print(f"Error in WaypointManager.__init__: {e}")
            import traceback
            traceback.print_exc()
            raise
        
        # Set up ROS2 publishers and subscribers if in connected mode
        if self.connected_mode:
            self.get_logger().info('Running in connected mode')
            
            # Subscribe to robot pose
            self.pose_subscription = self.create_subscription(
                PoseWithCovarianceStamped,
                '/amcl_pose',
                self.pose_callback,
                10
            )
            self.current_pose = None
            
            # Create publisher for waypoint visualization
            self.marker_publisher = self.create_publisher(
                MarkerArray,
                '/waypoint_markers',
                10
            )
            
            # Create publisher for MQTT status messages
            self.mqtt_status_publisher = self.create_publisher(
                String,
                'mqtt_status',
                10
            )
        else:
            self.get_logger().info('Running in standalone mode')
        
        # Create GUI
        self.gui = WaypointManagerGUI(self)
        self.gui.show()
        
        # Create timer for ROS2 callbacks
        self.timer = QTimer()
        self.timer.timeout.connect(self.spin_once)
        self.timer.start(100)  # 10 Hz
    
    def spin_once(self):
        rclpy.spin_once(self, timeout_sec=0)
    
    def pose_callback(self, msg):
        """Callback for robot pose updates"""
        self.current_pose = msg.pose.pose
    
    def set_current_map(self, map_name):
        self.get_logger().info(f'Setting current map to: {map_name}')
        self.current_map = map_name
        
        # Load waypoints for this map
        self.load_waypoints()
        
        # Publish waypoint markers
        self.publish_waypoint_markers()
    
    def load_waypoints(self):
        """Load waypoints from JSON file"""
        if os.path.exists(self.waypoints_file):
            try:
                with open(self.waypoints_file, 'r') as f:
                    data = json.load(f)
                
                # Check if the waypoints are in the new format (map-specific)
                if isinstance(data, dict) and any(isinstance(v, dict) for v in data.values()):
                    # New format with map-specific waypoints
                    self.waypoints = data
                    
                    # Ensure all map entries are dictionaries, not sets
                    for map_name, waypoints in list(self.waypoints.items()):
                        if not isinstance(waypoints, dict):
                            self.get_logger().warn(f'Converting waypoints for map {map_name} from {type(waypoints)} to dict')
                            # If it's a set or list, convert to a dictionary with numeric keys
                            if isinstance(waypoints, (set, list)):
                                self.waypoints[map_name] = {f'Waypoint {i+1}': wp for i, wp in enumerate(waypoints)}
                else:
                    # Old format with single waypoint list
                    # Convert to new format with 'default' map
                    if isinstance(data, (set, list)):
                        self.waypoints = {'default': {f'Waypoint {i+1}': wp for i, wp in enumerate(data)}}
                    else:
                        self.waypoints = {'default': data}
                    
                self.get_logger().info(f'Loaded waypoints for {len(self.waypoints)} maps')
            except Exception as e:
                self.get_logger().error(f'Failed to load waypoints: {e}')
                # Start with empty waypoints
                self.waypoints = {"yahboom_map": {}}
        else:
            self.get_logger().info('No waypoints file found, starting with empty waypoints')
            self.waypoints = {"yahboom_map": {}}
    
    def save_waypoints(self):
        """Save waypoints to JSON file"""
        try:
            with open(self.waypoints_file, 'w') as f:
                json.dump(self.waypoints, f, indent=2)
            
            total_waypoints = sum(len(waypoints) for waypoints in self.waypoints.values())
            self.get_logger().info(f'Saved {total_waypoints} waypoints across {len(self.waypoints)} maps')
        except Exception as e:
            self.get_logger().error(f'Failed to save waypoints: {e}')
    
    def add_waypoint(self, map_name, name, x, y):
        self.get_logger().info(f'Adding waypoint {name} at ({x}, {y}) to map {map_name}')
        
        # Check if waypoint with this name already exists
        if name in self.waypoints[map_name]:
            self.get_logger().warn(f'Waypoint {name} already exists, overwriting')
        
        # Add the waypoint
        self.waypoints[map_name][name] = {
            'position': {'x': x, 'y': y},
            'orientation': {'x': 0.0, 'y': 0.0, 'z': 0.0, 'w': 1.0}
        }
        
        # If we have current robot pose, update orientation
        if hasattr(self, 'current_pose') and self.current_pose is not None:
            self.waypoints[map_name][name]['orientation'] = {
                'x': self.current_pose.orientation.x,
                'y': self.current_pose.orientation.y,
                'z': self.current_pose.orientation.z,
                'w': self.current_pose.orientation.w
            }
        
        # Save waypoints
        self.save_waypoints()
        
        # Publish waypoint markers
        self.publish_waypoint_markers()
        
        # Return the waypoint
        return self.waypoints[map_name][name]
    
    def edit_waypoint(self, map_name, name, x, y):
        self.get_logger().info(f'Editing waypoint {name} to ({x}, {y}) on map {map_name}')
        
        # Check if waypoint exists
        if name not in self.waypoints[map_name]:
            self.get_logger().error(f'Waypoint {name} not found')
            return None
        
        # Update the waypoint position
        self.waypoints[map_name][name]['position']['x'] = x
        self.waypoints[map_name][name]['position']['y'] = y
        
        # Save waypoints
        self.save_waypoints()
        
        # Publish waypoint markers
        self.publish_waypoint_markers()
        
        # Return the updated waypoint
        return self.waypoints[map_name][name]
    
    def delete_waypoint(self, map_name, name):
        self.get_logger().info(f'Deleting waypoint {name} from map {map_name}')
        
        # Check if waypoint exists
        if name not in self.waypoints[map_name]:
            self.get_logger().error(f'Waypoint {name} not found')
            return False
        
        # Delete the waypoint
        del self.waypoints[map_name][name]
        
        # Save waypoints
        self.save_waypoints()
        
        # Publish waypoint markers
        self.publish_waypoint_markers()
        
        return True
    
    def setup_mqtt_client(self):
        """Set up MQTT client and connection"""
        try:
            # Create MQTT client
            client_id = f'waypoint_manager_{random.randint(0, 1000)}'  # Random client ID to avoid conflicts
            self.mqtt_client = mqtt.Client(client_id=client_id, protocol=mqtt.MQTTv5)
            
            # Set up callbacks
            self.mqtt_client.on_connect = self.on_mqtt_connect
            self.mqtt_client.on_disconnect = self.on_mqtt_disconnect
            self.mqtt_client.on_publish = self.on_mqtt_publish
            
            # Attempt connection
            try:
                self.get_logger().info(f'Connecting to MQTT broker at {self.mqtt_broker}:{self.mqtt_port}')
                self.mqtt_client.connect_async(self.mqtt_broker, self.mqtt_port, 60)
                self.mqtt_client.loop_start()  # Start the loop in a separate thread
            except Exception as e:
                self.get_logger().error(f'Failed to connect to MQTT broker: {e}')
                self.mqtt_connected = False
        except Exception as e:
            self.get_logger().error(f'Error setting up MQTT client: {e}')
            traceback.print_exc()
    
    def on_mqtt_connect(self, client, userdata, flags, rc, properties=None):
        """Callback when connected to MQTT broker"""
        if rc == 0:
            self.get_logger().info('Connected to MQTT broker')
            self.mqtt_connected = True
            
            # Publish a status message
            if self.connected_mode:
                status_msg = String()
                status_msg.data = f'MQTT connected to {self.mqtt_broker}:{self.mqtt_port}'
                self.mqtt_status_publisher.publish(status_msg)
                
            # Update GUI status
            if hasattr(self, 'gui') and self.gui is not None:
                self.gui.updateMqttStatus(True)
        else:
            self.get_logger().error(f'Failed to connect to MQTT broker with code {rc}')
            self.mqtt_connected = False
            
            # Update GUI status
            if hasattr(self, 'gui') and self.gui is not None:
                self.gui.updateMqttStatus(False)
    
    def on_mqtt_disconnect(self, client, userdata, rc):
        """Callback when disconnected from MQTT broker"""
        self.get_logger().warn(f'Disconnected from MQTT broker with code {rc}')
        self.mqtt_connected = False
        
        # Update GUI status
        if hasattr(self, 'gui') and self.gui is not None:
            self.gui.updateMqttStatus(False)
    
    def on_mqtt_publish(self, client, userdata, mid):
        """Callback when a message is published"""
        self.get_logger().debug(f'Message {mid} published')
    
    def publish_mqtt_message(self, topic, message):
        """Publish a message to an MQTT topic"""
        if not self.mqtt_connected:
            self.get_logger().warn('Cannot publish MQTT message: not connected to broker')
            return False
        
        try:
            # Prepend the topic prefix
            full_topic = f"{self.mqtt_topic_prefix}/{topic}"
            
            # Publish the message
            result = self.mqtt_client.publish(full_topic, message)
            if result.rc != mqtt.MQTT_ERR_SUCCESS:
                self.get_logger().error(f'Failed to publish MQTT message: {mqtt.error_string(result.rc)}')
                return False
            
            self.get_logger().info(f'Published MQTT message to {full_topic}: {message}')
            return True
        except Exception as e:
            self.get_logger().error(f'Error publishing MQTT message: {e}')
            traceback.print_exc()
            return False
            
    def navigate_to_waypoint(self, waypoint_name):
        """Send MQTT command to navigate to a waypoint
        
        Uses coordinate-based navigation commands in the format:
        {"command": "goto", "waypoint_id": "debug_name", 
         "position": {"x": float, "y": float}, 
         "orientation": {"x": float, "y": float, "z": float, "w": float}}
        """
        if not self.mqtt_connected:
            self.get_logger().warn('Cannot navigate to waypoint: not connected to MQTT broker')
            return False
            
        if self.current_map is None:
            self.get_logger().warn('Cannot navigate to waypoint: no map selected')
            return False
            
        # Check if waypoint exists
        if self.current_map not in self.waypoints or waypoint_name not in self.waypoints[self.current_map]:
            self.get_logger().warn(f'Cannot navigate to waypoint: {waypoint_name} not found in map {self.current_map}')
            return False
        
        # Get waypoint data
        waypoint = self.waypoints[self.current_map][waypoint_name]
        
        # Create coordinate-based navigation command
        command = {
            "command": "goto",
            "waypoint_id": waypoint_name,
            "position": waypoint["position"],
            "orientation": waypoint["orientation"]
        }
        
        # Convert to JSON and publish
        command_json = json.dumps(command)
        self.get_logger().info(f'Sending coordinate-based navigation command: {command_json}')
        
        # Publish navigation command directly to avoid any issues
        try:
            # Prepend the topic prefix
            full_topic = f"{self.mqtt_topic_prefix}/navigation/command"
            
            # Publish the message
            result = self.mqtt_client.publish(full_topic, command_json)
            if result.rc != mqtt.MQTT_ERR_SUCCESS:
                self.get_logger().error(f'Failed to publish MQTT message: {mqtt.error_string(result.rc)}')
                return False
            
            self.get_logger().info(f'Published coordinate-based navigation command to {full_topic}: {command_json}')
            return True
        except Exception as e:
            self.get_logger().error(f'Error publishing coordinate-based navigation command: {e}')
            traceback.print_exc()
            return False
    
    def publish_waypoint_markers(self):
        """Publish waypoint visualization markers"""
        try:
            if not self.connected_mode or not self.current_map:
                return
            
            # Import required message types
            from geometry_msgs.msg import Point
            from std_msgs.msg import ColorRGBA
            
            # Create marker array
            marker_array = MarkerArray()
            
            # Debug information
            self.get_logger().debug(f'Current map: {self.current_map}')
            self.get_logger().debug(f'Waypoints keys: {list(self.waypoints.keys())}')
            
            # Get waypoints for current map
            if self.current_map not in self.waypoints:
                self.get_logger().warn(f'No waypoints found for map: {self.current_map}')
                return
                
            # Get waypoints for current map and ensure it's a dictionary
            map_waypoints = self.waypoints[self.current_map]
            if not isinstance(map_waypoints, dict):
                self.get_logger().warn(f'Waypoints for map {self.current_map} is not a dictionary')
                return
            
            # Loop through waypoints
            for i, (name, waypoint) in enumerate(map_waypoints.items()):
                try:
                    # Create marker
                    marker = Marker()
                    marker.header.frame_id = 'map'
                    marker.header.stamp = self.get_clock().now().to_msg()
                    marker.ns = 'waypoints'
                    marker.id = i
                    marker.type = Marker.SPHERE
                    marker.action = Marker.ADD
                
                    # Set position
                    if 'position' in waypoint and isinstance(waypoint['position'], dict):
                        marker.pose.position.x = waypoint['position'].get('x', 0.0)
                        marker.pose.position.y = waypoint['position'].get('y', 0.0)
                        marker.pose.position.z = 0.0
                    else:
                        self.get_logger().warn(f'Invalid position for waypoint {name}')
                        continue
                
                    # Set orientation
                    if 'orientation' in waypoint and isinstance(waypoint['orientation'], dict):
                        marker.pose.orientation.x = waypoint['orientation'].get('x', 0.0)
                        marker.pose.orientation.y = waypoint['orientation'].get('y', 0.0)
                        marker.pose.orientation.z = waypoint['orientation'].get('z', 0.0)
                        marker.pose.orientation.w = waypoint['orientation'].get('w', 1.0)
                    else:
                        # Default to identity quaternion
                        marker.pose.orientation.w = 1.0
                
                    # Set scale
                    vis = waypoint.get('visualization', {})
                    scale = vis.get('scale', 0.3)
                    marker.scale.x = scale
                    marker.scale.y = scale
                    marker.scale.z = scale
                
                    # Set color
                    color = vis.get('color', {'r': 0.0, 'g': 0.0, 'b': 1.0})
                    marker.color.r = float(color.get('r', 0.0))
                    marker.color.g = float(color.get('g', 0.0))
                    marker.color.b = float(color.get('b', 1.0))
                    marker.color.a = 0.7  # Semi-transparent
                
                    # Add to marker array
                    marker_array.markers.append(marker)
                
                    # Create text marker for waypoint name
                    text_marker = Marker()
                    text_marker.header.frame_id = 'map'
                    text_marker.header.stamp = self.get_clock().now().to_msg()
                    text_marker.ns = 'waypoint_names'
                    text_marker.id = i
                    text_marker.type = Marker.TEXT_VIEW_FACING
                    text_marker.action = Marker.ADD
                
                    # Set position (slightly above waypoint)
                    text_marker.pose.position.x = marker.pose.position.x
                    text_marker.pose.position.y = marker.pose.position.y
                    text_marker.pose.position.z = 0.5  # Above the waypoint
                
                    # Set orientation (identity)
                    text_marker.pose.orientation.w = 1.0
                
                    # Set scale
                    text_marker.scale.z = 0.2  # Text height
                
                    # Set color (white)
                    text_marker.color.r = 1.0
                    text_marker.color.g = 1.0
                    text_marker.color.b = 1.0
                    text_marker.color.a = 1.0
                
                    # Set text
                    text_marker.text = str(name)
                
                    # Add to marker array
                    marker_array.markers.append(text_marker)
                except Exception as e:
                    self.get_logger().error(f'Error creating marker for waypoint {name}: {e}')
                    traceback.print_exc()
                    continue
            
            # Publish marker array
            try:
                self.marker_publisher.publish(marker_array)
            except Exception as e:
                self.get_logger().error(f'Error publishing marker array: {e}')
                traceback.print_exc()
        except Exception as e:
            self.get_logger().error(f'Error publishing waypoint markers: {e}')
            traceback.print_exc()

def main(args=None):
    print("Starting Waypoint Manager...")
    try:
        rclpy.init(args=args)
        print("ROS2 initialized")
        
        # Import datetime for timestamps
        global datetime
        from datetime import datetime
        print("Datetime imported")
        
        # Import QImage for map display
        global QImage
        from PyQt5.QtGui import QImage
        print("QImage imported")
        
        # Create Qt application
        print("Creating Qt application...")
        app = QApplication(sys.argv)
        print("Qt application created")
        
        # Set application style
        app.setStyle('Fusion')
        print("Application style set to Fusion")
        
        # Create the ROS2 node with detailed error handling
        print("Creating WaypointManager node...")
        try:
            # Initialize waypoints dictionary first to avoid any issues
            print("Initializing empty waypoints dictionary")
            waypoints = {}
            
            # Create the node
            print("Creating WaypointManager instance")
            waypoint_manager = WaypointManager()
            print("WaypointManager node created successfully")
            
            # Start the application
            print("Starting Qt application main loop")
            sys.exit(app.exec_())
        except TypeError as e:
            print(f"TypeError in WaypointManager: {e}")
            print(f"Error location: {sys._getframe().f_lineno}")
            import traceback
            traceback.print_exc()
            sys.exit(1)
        except Exception as e:
            print(f"Exception in WaypointManager: {e}")
            import traceback
            traceback.print_exc()
            sys.exit(1)
    except Exception as e:
        print(f"Error in main: {e}")
        import traceback
        traceback.print_exc()
        sys.exit(1)

if __name__ == '__main__':
    main()
