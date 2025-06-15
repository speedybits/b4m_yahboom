#!/usr/bin/env python3

import os
import sys
import json
import yaml
import numpy as np
import random
import traceback
from PIL import Image
from datetime import datetime

import rclpy
from rclpy.node import Node
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import PoseWithCovarianceStamped, Pose
# Import visualization messages separately to avoid any import issues
from visualization_msgs.msg import MarkerArray
from visualization_msgs.msg import Marker

from PyQt5.QtWidgets import (QApplication, QMainWindow, QWidget, QVBoxLayout, 
                             QHBoxLayout, QLabel, QPushButton, QListWidget, 
                             QSplitter, QComboBox, QGroupBox, QStatusBar,
                             QMessageBox, QFileDialog, QInputDialog)
from PyQt5.QtCore import Qt, QSettings, QTimer, QRectF
from PyQt5.QtGui import QPainter, QColor, QPen, QBrush, QPixmap, QFont, QImage

class WaypointManagerGUI(QMainWindow):
    def __init__(self, ros_node):
        super().__init__()
        
        # Store reference to ROS node
        self.ros_node = ros_node
        
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
        
        # Map selection group
        map_group = QGroupBox('Map Selection')
        map_layout = QVBoxLayout(map_group)
        self.map_combo = QComboBox()
        self.refresh_maps_btn = QPushButton('Refresh Maps')
        map_layout.addWidget(QLabel('Select Map:'))
        map_layout.addWidget(self.map_combo)
        map_layout.addWidget(self.refresh_maps_btn)
        left_layout.addWidget(map_group)
        
        # Waypoint list group
        waypoint_group = QGroupBox('Waypoints')
        waypoint_layout = QVBoxLayout(waypoint_group)
        self.waypoint_list = QListWidget()
        waypoint_layout.addWidget(self.waypoint_list)
        left_layout.addWidget(waypoint_group)
        
        # Waypoint actions group
        actions_group = QGroupBox('Actions')
        actions_layout = QVBoxLayout(actions_group)
        self.add_waypoint_btn = QPushButton('Add Waypoint')
        self.edit_waypoint_btn = QPushButton('Edit Selected')
        self.delete_waypoint_btn = QPushButton('Delete Selected')
        actions_layout.addWidget(self.add_waypoint_btn)
        actions_layout.addWidget(self.edit_waypoint_btn)
        actions_layout.addWidget(self.delete_waypoint_btn)
        left_layout.addWidget(actions_group)
        
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
        
        # Create status bar
        self.statusBar = QStatusBar()
        self.setStatusBar(self.statusBar)
        self.statusBar.showMessage('Ready')
        
        # Connect signals
        self.refresh_maps_btn.clicked.connect(self.refreshMaps)
        self.map_combo.currentIndexChanged.connect(self.loadSelectedMap)
        self.add_waypoint_btn.clicked.connect(self.addWaypoint)
        self.edit_waypoint_btn.clicked.connect(self.editWaypoint)
        self.delete_waypoint_btn.clicked.connect(self.deleteWaypoint)
        self.waypoint_list.itemClicked.connect(self.waypointSelected)
        
        # Initial UI setup
        self.refreshMaps()
        
    def loadSettings(self):
        # Load application settings
        settings = QSettings('B4M', 'WaypointManager')
        
        # Restore window geometry if available
        if settings.contains('geometry'):
            self.restoreGeometry(settings.value('geometry'))
        
        # Restore last used map if available
        if settings.contains('last_map'):
            last_map = settings.value('last_map')
            index = self.map_combo.findText(last_map)
            if index >= 0:
                self.map_combo.setCurrentIndex(index)
    
    def closeEvent(self, event):
        # Save application settings when closing
        settings = QSettings('B4M', 'WaypointManager')
        settings.setValue('geometry', self.saveGeometry())
        settings.setValue('last_map', self.map_combo.currentText())
        super().closeEvent(event)
    
    def refreshMaps(self):
        # Get maps from the yahboomcar_nav/maps directory
        self.ros_node.get_logger().info('Refreshing maps...')
        self.map_combo.clear()
        
        # Get the maps directory path
        maps_dir = os.path.join('/home/yahboom/b4m_yahboom/yahboomcar_nav/maps')
        
        if not os.path.exists(maps_dir):
            self.statusBar.showMessage(f'Maps directory not found: {maps_dir}')
            self.ros_node.get_logger().error(f'Maps directory not found: {maps_dir}')
            return
        
        # Find all yaml files (map configurations)
        yaml_files = [f for f in os.listdir(maps_dir) if f.endswith('.yaml')]
        
        if not yaml_files:
            self.statusBar.showMessage('No map files found')
            self.ros_node.get_logger().warn('No map files found')
            return
        
        # Add maps to combo box
        for yaml_file in sorted(yaml_files):
            map_name = os.path.splitext(yaml_file)[0]
            self.map_combo.addItem(map_name)
        
        self.statusBar.showMessage(f'Found {len(yaml_files)} maps')
    
    def loadSelectedMap(self):
        # Load the selected map
        map_name = self.map_combo.currentText()
        if not map_name:
            return
        
        self.ros_node.get_logger().info(f'Loading map: {map_name}')
        self.statusBar.showMessage(f'Loading map: {map_name}...')
        
        # Get map file paths
        maps_dir = os.path.join('/home/yahboom/b4m_yahboom/yahboomcar_nav/maps')
        yaml_path = os.path.join(maps_dir, f'{map_name}.yaml')
        
        if not os.path.exists(yaml_path):
            self.statusBar.showMessage(f'Map file not found: {yaml_path}')
            self.ros_node.get_logger().error(f'Map file not found: {yaml_path}')
            return
        
        # Parse YAML file to get map properties
        try:
            import yaml
            with open(yaml_path, 'r') as f:
                map_data = yaml.safe_load(f)
            
            # Get map image path
            image_path = os.path.join(maps_dir, map_data.get('image', ''))
            if not os.path.exists(image_path):
                self.statusBar.showMessage(f'Map image not found: {image_path}')
                self.ros_node.get_logger().error(f'Map image not found: {image_path}')
                return
            
            # Get map properties
            resolution = map_data.get('resolution', 0.05)  # meters/pixel
            origin = map_data.get('origin', [0, 0, 0])     # [x, y, theta] in meters
            
            # Load map image using PIL
            try:
                img = Image.open(image_path)
                # Convert to RGB if needed
                if img.mode != 'RGB':
                    img = img.convert('RGB')
                
                # Convert PIL image to QPixmap
                img_data = img.tobytes('raw', 'RGB')
                qimg = QPixmap.fromImage(
                    QImage(img_data, img.width, img.height, QImage.Format_RGB888)
                )
                
                # Update map view
                self.map_view.setMap(qimg, resolution, (origin[0], origin[1]))
                
                # Set current map in ROS node
                self.ros_node.set_current_map(map_name)
                
                # Load waypoints for this map
                self.updateWaypointList()
                
                self.statusBar.showMessage(f'Map loaded: {map_name}')
                self.ros_node.get_logger().info(f'Map loaded: {map_name}')
                
            except Exception as e:
                self.statusBar.showMessage(f'Failed to load map image: {e}')
                self.ros_node.get_logger().error(f'Failed to load map image: {e}')
        
        except Exception as e:
            self.statusBar.showMessage(f'Failed to parse map YAML: {e}')
            self.ros_node.get_logger().error(f'Failed to parse map YAML: {e}')
    
    def updateWaypointList(self):
        # Update the waypoint list for the current map
        self.waypoint_list.clear()
        
        map_name = self.map_combo.currentText()
        if not map_name or map_name not in self.ros_node.waypoints:
            return
        
        # Add waypoints to list
        waypoints = self.ros_node.waypoints.get(map_name, {})
        for name in sorted(waypoints.keys()):
            self.waypoint_list.addItem(name)
        
    def addWaypoint(self):
        # Add a new waypoint
        self.ros_node.get_logger().info('Add waypoint requested')
        self.statusBar.showMessage('Click on the map to place a waypoint')
        self.map_view.setAddWaypointMode(True)
    
    def editWaypoint(self):
        # Edit the selected waypoint
        self.ros_node.get_logger().info('Edit waypoint requested')
        
        # Check if a waypoint is selected
        selected_items = self.waypoint_list.selectedItems()
        if not selected_items:
            self.statusBar.showMessage('No waypoint selected')
            return
        
        # Get selected waypoint name
        waypoint_name = selected_items[0].text()
        map_name = self.map_combo.currentText()
        
        # Check if map and waypoint exist
        if map_name not in self.ros_node.waypoints or waypoint_name not in self.ros_node.waypoints[map_name]:
            self.statusBar.showMessage(f'Waypoint {waypoint_name} not found in map {map_name}')
            return
        
        # Enable map click mode for editing
        self.statusBar.showMessage(f'Click on the map to set new position for {waypoint_name}')
        self.map_view.setEditWaypointMode(True, waypoint_name)
    
    def deleteWaypoint(self):
        # Delete the selected waypoint
        self.ros_node.get_logger().info('Delete waypoint requested')
        
        # Check if a waypoint is selected
        selected_items = self.waypoint_list.selectedItems()
        if not selected_items:
            self.statusBar.showMessage('No waypoint selected')
            return
        
        # Get selected waypoint name
        waypoint_name = selected_items[0].text()
        map_name = self.map_combo.currentText()
        
        # Confirm deletion
        from PyQt5.QtWidgets import QMessageBox
        reply = QMessageBox.question(
            self, 'Confirm Deletion',
            f'Are you sure you want to delete waypoint {waypoint_name}?',
            QMessageBox.Yes | QMessageBox.No, QMessageBox.No
        )
        
        if reply == QMessageBox.Yes:
            # Delete waypoint
            if self.ros_node.delete_waypoint(map_name, waypoint_name):
                self.updateWaypointList()
                self.statusBar.showMessage(f'Deleted waypoint: {waypoint_name}')
            else:
                self.statusBar.showMessage(f'Failed to delete waypoint: {waypoint_name}')
    
    def waypointSelected(self, item):
        # Handle waypoint selection
        waypoint_name = item.text()
        self.ros_node.get_logger().info(f'Waypoint selected: {waypoint_name}')
        
        # Update map view to highlight selected waypoint
        self.map_view.selectWaypoint(waypoint_name)
        
        # Enable edit and delete buttons
        self.edit_waypoint_btn.setEnabled(True)
        self.delete_waypoint_btn.setEnabled(True)
        
        # Show waypoint details in status bar
        map_name = self.map_combo.currentText()
        if map_name in self.ros_node.waypoints and waypoint_name in self.ros_node.waypoints[map_name]:
            waypoint = self.ros_node.waypoints[map_name][waypoint_name]
            pos_x = waypoint['position']['x']
            pos_y = waypoint['position']['y']
            self.statusBar.showMessage(f'Selected waypoint: {waypoint_name} at ({pos_x:.2f}, {pos_y:.2f})')

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
                    map_name = self.parent.map_combo.currentText()
                    if map_name:
                        # Create a dialog for waypoint name input
                        from PyQt5.QtWidgets import QInputDialog
                        
                        # Generate a suggested name
                        existing_waypoints = self.parent.ros_node.waypoints.get(map_name, {})
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
                                self.parent.ros_node.add_waypoint(map_name, name, x, y)
                                self.parent.updateWaypointList()
                                self.parent.statusBar.showMessage(f'Added waypoint: {name}')
                    
                    # Exit add waypoint mode
                    self.add_waypoint_mode = False
                    self.setCursor(Qt.ArrowCursor)
                    
                elif self.edit_waypoint_mode and self.edit_waypoint_name:
                    # Handle editing an existing waypoint
                    map_name = self.parent.map_combo.currentText()
                    if map_name:
                        # Update the waypoint position
                        if self.parent.ros_node.edit_waypoint(map_name, self.edit_waypoint_name, x, y):
                            self.parent.statusBar.showMessage(f'Updated waypoint: {self.edit_waypoint_name}')
                            self.update()  # Redraw the map view
                        else:
                            self.parent.statusBar.showMessage(f'Failed to update waypoint: {self.edit_waypoint_name}')
                    
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
                    self.parent.statusBar.showMessage('Operation cancelled')
    
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
        """Convert pixel coordinates to map coordinates"""
        if not self.map_image:
            return 0, 0
            
        # Adjust for scale and offset
        adjusted_x = (pixel_x - self.map_offset_x) / self.scale_factor
        adjusted_y = (pixel_y - self.map_offset_y) / self.scale_factor
        
        # Convert to map coordinates (ROS uses a different coordinate system)
        # In ROS, the origin is typically at the bottom-left of the map
        # In Qt, the origin is at the top-left of the widget
        map_x = adjusted_x * self.map_resolution + self.map_origin[0]
        map_y = (self.map_image.height() - adjusted_y) * self.map_resolution + self.map_origin[1]
        
        return map_x, map_y
    
    def mapToPixelCoordinates(self, map_x, map_y):
        """Convert map coordinates to pixel coordinates"""
        if not self.map_image:
            return 0, 0
            
        # Convert from map coordinates to pixel coordinates
        pixel_x = (map_x - self.map_origin[0]) / self.map_resolution
        pixel_y = self.map_image.height() - (map_y - self.map_origin[1]) / self.map_resolution
        
        # Adjust for scale and offset
        adjusted_x = pixel_x * self.scale_factor + self.map_offset_x
        adjusted_y = pixel_y * self.scale_factor + self.map_offset_y
        
        return adjusted_x, adjusted_y
    
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
            
            # Use the correct overload of drawPixmap with integer coordinates
            painter.drawPixmap(
                x_offset, y_offset, 
                scaled_width, scaled_height,
                self.map_image
            )
            
            # Draw waypoints
            map_name = self.parent.map_combo.currentText()
            if map_name and map_name in self.parent.ros_node.waypoints:
                waypoints = self.parent.ros_node.waypoints[map_name]
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
                self.parent.statusBar.showMessage(f'Zoom error: {e}')
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
            
            # Initialize waypoints storage
            self.waypoints = {}
            self.current_map = None
            
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
                self.waypoints = {}
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
        """Set the current map"""
        self.current_map = map_name
        
        # Initialize waypoints for this map if not already present
        if map_name not in self.waypoints:
            self.waypoints[map_name] = {}
            self.save_waypoints()
        elif not isinstance(self.waypoints[map_name], dict):
            # Convert to dictionary if it's not already
            self.get_logger().warn(f'Converting waypoints for map {map_name} from {type(self.waypoints[map_name])} to dict')
            if isinstance(self.waypoints[map_name], (set, list)):
                self.waypoints[map_name] = {f'Waypoint {i+1}': wp for i, wp in enumerate(self.waypoints[map_name])}
            else:
                self.waypoints[map_name] = {}
            self.save_waypoints()
        
        # Update visualization if in connected mode
        if self.connected_mode:
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
                self.waypoints = {}
        else:
            self.get_logger().info('No waypoints file found, starting with empty waypoints')
            self.waypoints = {}
    
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
        """Add a new waypoint"""
        # Ensure the map exists in waypoints
        if map_name not in self.waypoints:
            self.waypoints[map_name] = {}
        
        # Generate random color for visualization
        import random
        r = random.random()
        g = random.random()
        b = random.random()
        
        # Create waypoint with default orientation (identity quaternion)
        waypoint = {
            'name': name,
            'position': {
                'x': x,
                'y': y
            },
            'orientation': {
                'x': 0.0,
                'y': 0.0,
                'z': 0.0,
                'w': 1.0
            },
            'timestamp': datetime.now().isoformat(),
            'visualization': {
                'color': {
                    'r': r,
                    'g': g,
                    'b': b
                },
                'scale': 0.3
            }
        }
        
        # Add to waypoints dictionary
        self.waypoints[map_name][name] = waypoint
        
        # Save waypoints
        self.save_waypoints()
        
        # Update visualization if in connected mode
        if self.connected_mode:
            self.publish_waypoint_markers()
        
        self.get_logger().info(f'Added waypoint {name} at ({x:.2f}, {y:.2f})')
        return True
    
    def edit_waypoint(self, map_name, name, x, y):
        """Edit an existing waypoint"""
        if map_name not in self.waypoints or name not in self.waypoints[map_name]:
            self.get_logger().error(f'Waypoint {name} not found in map {map_name}')
            return False
        
        # Update position
        self.waypoints[map_name][name]['position']['x'] = x
        self.waypoints[map_name][name]['position']['y'] = y
        self.waypoints[map_name][name]['timestamp'] = datetime.now().isoformat()
        
        # Save waypoints
        self.save_waypoints()
        
        # Update visualization if in connected mode
        if self.connected_mode:
            self.publish_waypoint_markers()
        
        self.get_logger().info(f'Updated waypoint {name} to ({x:.2f}, {y:.2f})')
        return True
    
    def delete_waypoint(self, map_name, name):
        """Delete a waypoint"""
        if map_name not in self.waypoints or name not in self.waypoints[map_name]:
            self.get_logger().error(f'Waypoint {name} not found in map {map_name}')
            return False
        
        # Remove waypoint
        del self.waypoints[map_name][name]
        
        # Save waypoints
        self.save_waypoints()
        
        # Update visualization if in connected mode
        if self.connected_mode:
            self.publish_waypoint_markers()
        
        self.get_logger().info(f'Deleted waypoint {name}')
        return True
    
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
                self.get_logger().error(f'Waypoints for map {self.current_map} is not a dictionary: {type(map_waypoints)}')
                if isinstance(map_waypoints, (set, list)):
                    self.get_logger().info(f'Converting {type(map_waypoints)} to dictionary')
                    self.waypoints[self.current_map] = {f'Waypoint {i+1}': wp for i, wp in enumerate(map_waypoints)}
                    map_waypoints = self.waypoints[self.current_map]
                    self.save_waypoints()
                else:
                    self.get_logger().error('Unable to convert waypoints to dictionary')
                    return
                    
            # Process each waypoint
            for i, (name, waypoint) in enumerate(map_waypoints.items()):
                try:
                    # Create marker for waypoint
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
            self.get_logger().error(f'Error in publish_waypoint_markers: {e}')
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
