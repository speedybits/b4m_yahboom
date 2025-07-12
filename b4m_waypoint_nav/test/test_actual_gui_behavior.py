#!/usr/bin/env python3

import unittest
import sys
import os
import time
import threading
from unittest.mock import Mock, patch

# Add the package path so we can import the actual GUI
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

try:
    from PyQt5.QtWidgets import QApplication
    from PyQt5.QtCore import QTimer
    PYQT5_AVAILABLE = True
except ImportError:
    PYQT5_AVAILABLE = False


@unittest.skipUnless(PYQT5_AVAILABLE, "PyQt5 not available")
class TestActualGUIBehavior(unittest.TestCase):
    """Test the ACTUAL GUI behavior - no mocks allowed"""
    
    @classmethod
    def setUpClass(cls):
        """Set up QApplication for the entire test class"""
        if not QApplication.instance():
            cls.app = QApplication([])
        else:
            cls.app = QApplication.instance()
    
    def test_real_stopall_dialog_behavior(self):
        """Test the REAL stopAll dialog behavior with actual GUI code"""
        
        # Import the real GUI class (with all dependencies mocked except PyQt5)
        with patch('rclpy.node.Node'), \
             patch('ament_index_python.packages.get_package_share_directory'), \
             patch('paho.mqtt.client'), \
             patch('PIL.Image'), \
             patch('subprocess.run') as mock_subprocess:
            
            # Mock subprocess to return no containers (quick shutdown)
            mock_subprocess.return_value.stdout = ''
            
            from b4m_waypoint_nav.b4m_robot_manager import B4MRobotManagerGUI
            
            # Create the actual GUI
            mock_ros_node = Mock()
            gui = B4MRobotManagerGUI(mock_ros_node)
            
            # Mock heavy operations but keep GUI behavior real
            gui.stopSystem = Mock()
            gui.updateControlButtonStates = Mock()
            gui.agent_status_display = Mock()
            gui.stop_all_btn = Mock()
            
            # Set initial state
            gui.agent_state = 'running'
            
            print("=== TESTING REAL GUI STOPALL BEHAVIOR ===")
            
            # Call the REAL stopAll method
            gui.stopAll()
            
            # Check if dialog was created
            dialog_created = hasattr(gui, 'shutdown_dialog') and gui.shutdown_dialog is not None
            print(f"Dialog created: {dialog_created}")
            
            if dialog_created:
                print(f"Dialog type: {type(gui.shutdown_dialog)}")
                print(f"Dialog initially visible: {gui.shutdown_dialog.isVisible()}")
                print(f"Dialog modal: {gui.shutdown_dialog.isModal()}")
                
                # Wait for the background shutdown thread to complete
                # and for any QTimer callbacks to execute
                max_wait = 3.0  # 3 seconds should be plenty
                wait_interval = 0.1
                total_waited = 0
                
                initial_visible = gui.shutdown_dialog.isVisible()
                
                while total_waited < max_wait:
                    # Process ALL pending events to allow QTimer callbacks
                    QApplication.processEvents()
                    
                    current_visible = gui.shutdown_dialog.isVisible()
                    
                    print(f"Time: {total_waited:.1f}s - Dialog visible: {current_visible}")
                    
                    if initial_visible and not current_visible:
                        print("SUCCESS: Dialog was closed!")
                        break
                        
                    time.sleep(wait_interval)
                    total_waited += wait_interval
                
                final_visible = gui.shutdown_dialog.isVisible()
                print(f"Final dialog state - Visible: {final_visible}")
                
                # The test assertion - this should PASS if our fix works
                self.assertFalse(final_visible, 
                               f"BUG CONFIRMED: Dialog is still visible after {max_wait}s!")
                
                # Clean up
                if gui.shutdown_dialog.isVisible():
                    gui.shutdown_dialog.close()
            else:
                self.fail("Dialog was not created by stopAll() method")
    
    def test_dialog_visibility_tracking(self):
        """Test that we can properly track dialog visibility changes"""
        
        from PyQt5.QtWidgets import QDialog, QVBoxLayout, QLabel
        from PyQt5.QtCore import Qt
        
        # Create a test dialog exactly like our implementation
        dialog = QDialog()
        dialog.setWindowTitle("Test Dialog")
        dialog.setModal(False)
        dialog.setWindowFlags(dialog.windowFlags() | Qt.WindowStaysOnTopHint)
        dialog.resize(250, 100)
        
        layout = QVBoxLayout(dialog)
        label = QLabel("Test Dialog Content")
        label.setAlignment(Qt.AlignCenter)
        layout.addWidget(label)
        
        # Show dialog
        dialog.show()
        QApplication.processEvents()
        
        print(f"Test dialog visible after show(): {dialog.isVisible()}")
        self.assertTrue(dialog.isVisible(), "Test dialog should be visible after show()")
        
        # Close dialog
        dialog.close()
        QApplication.processEvents()
        
        print(f"Test dialog visible after close(): {dialog.isVisible()}")
        self.assertFalse(dialog.isVisible(), "Test dialog should be hidden after close()")


if __name__ == '__main__':
    # Run with verbose output
    unittest.main(verbosity=2)