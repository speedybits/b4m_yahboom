#!/usr/bin/env python3

import unittest
import sys
import time
import threading
from unittest.mock import Mock, patch
import os

# Add the package path to sys.path so we can import the GUI
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

try:
    from PyQt5.QtWidgets import QApplication, QMessageBox
    from PyQt5.QtCore import QTimer, Qt
    from PyQt5.QtTest import QTest
    PYQT5_AVAILABLE = True
except ImportError:
    PYQT5_AVAILABLE = False


@unittest.skipUnless(PYQT5_AVAILABLE, "PyQt5 not available")
class TestGUIIntegration(unittest.TestCase):
    """Integration test for the actual GUI behavior"""
    
    @classmethod
    def setUpClass(cls):
        """Set up QApplication for the entire test class"""
        if not QApplication.instance():
            cls.app = QApplication([])
        else:
            cls.app = QApplication.instance()
    
    def setUp(self):
        """Set up test fixtures"""
        # Mock ROS node to avoid ROS dependencies
        self.mock_ros_node = Mock()
        
    def test_stop_all_dialog_closes_after_shutdown(self):
        """Integration test: Verify dialog actually closes after shutdown completes"""
        
        # Mock all the heavy dependencies but keep PyQt5 behavior intact
        with patch('rclpy.node.Node'), \
             patch('ament_index_python.packages.get_package_share_directory'), \
             patch('paho.mqtt.client'), \
             patch('PIL.Image'), \
             patch('subprocess.run') as mock_subprocess:
            
            # Mock subprocess to return no containers (quick shutdown)
            mock_subprocess.return_value.stdout = ''
            
            # Import and create the GUI (after mocking dependencies)
            from b4m_waypoint_nav.b4m_robot_manager import B4MRobotManagerGUI
            
            # Create GUI instance
            gui = B4MRobotManagerGUI(self.mock_ros_node)
            
            # Mock the stopSystem method to avoid complexity
            gui.stopSystem = Mock()
            
            # Set initial state
            gui.agent_state = 'running'
            
            # Call stopAll method
            gui.stopAll()
            
            # Check that dialog was created
            self.assertTrue(hasattr(gui, 'shutdown_dialog'), "Shutdown dialog was not created")
            self.assertIsNotNone(gui.shutdown_dialog, "Shutdown dialog is None")
            
            # Verify dialog is visible initially
            self.assertTrue(gui.shutdown_dialog.isVisible(), "Dialog should be visible initially")
            
            # Wait for the background thread to complete and dialog to close
            # The actual bug is that the dialog stays open, so this test should fail
            max_wait_time = 5.0  # seconds
            wait_interval = 0.1
            total_waited = 0
            
            while total_waited < max_wait_time:
                # Process events to allow QTimer callbacks to execute
                QApplication.processEvents()
                
                # Check if dialog is closed
                if not gui.shutdown_dialog.isVisible():
                    break
                    
                time.sleep(wait_interval)
                total_waited += wait_interval
            
            # This assertion should FAIL if the bug exists
            self.assertFalse(gui.shutdown_dialog.isVisible(), 
                           "BUG DETECTED: Shutdown dialog is still visible after shutdown completes!")
            
            # Clean up
            if hasattr(gui, 'shutdown_dialog') and gui.shutdown_dialog:
                gui.shutdown_dialog.close()
    
    def test_dialog_properties_are_correct(self):
        """Test that the dialog has the correct properties when created"""
        
        with patch('rclpy.node.Node'), \
             patch('ament_index_python.packages.get_package_share_directory'), \
             patch('paho.mqtt.client'), \
             patch('PIL.Image'), \
             patch('subprocess.run') as mock_subprocess:
            
            mock_subprocess.return_value.stdout = ''
            
            from b4m_waypoint_nav.b4m_robot_manager import B4MRobotManagerGUI
            
            gui = B4MRobotManagerGUI(self.mock_ros_node)
            gui.stopSystem = Mock()
            gui.agent_state = 'running'
            
            # Trigger dialog creation
            gui.stopAll()
            
            # Verify dialog properties
            self.assertTrue(hasattr(gui, 'shutdown_dialog'))
            dialog = gui.shutdown_dialog
            
            # Check dialog properties
            self.assertEqual(dialog.windowTitle(), "Shutting Down")
            self.assertEqual(dialog.text(), "Shutting Down. Please Wait.")
            self.assertTrue(dialog.isModal())
            self.assertEqual(dialog.standardButtons(), QMessageBox.NoButton)
            
            # Clean up
            dialog.close()
    
    def test_agent_state_changes_during_shutdown(self):
        """Test that agent state changes correctly during shutdown process"""
        
        with patch('rclpy.node.Node'), \
             patch('ament_index_python.packages.get_package_share_directory'), \
             patch('paho.mqtt.client'), \
             patch('PIL.Image'), \
             patch('subprocess.run') as mock_subprocess:
            
            mock_subprocess.return_value.stdout = ''
            
            from b4m_waypoint_nav.b4m_robot_manager import B4MRobotManagerGUI
            
            gui = B4MRobotManagerGUI(self.mock_ros_node)
            gui.stopSystem = Mock()
            
            # Initial state should be running
            gui.agent_state = 'running'
            self.assertEqual(gui.agent_state, 'running')
            
            # Trigger shutdown
            gui.stopAll()
            
            # Wait for thread to complete
            time.sleep(1)
            QApplication.processEvents()
            
            # Agent state should be stopped
            self.assertEqual(gui.agent_state, 'stopped')
            
            # Clean up
            if hasattr(gui, 'shutdown_dialog'):
                gui.shutdown_dialog.close()


class TestDialogBugDetection(unittest.TestCase):
    """Test specifically designed to detect the dialog closing bug"""
    
    def test_bug_simulation(self):
        """Simulate the exact bug: dialog created but not properly closed"""
        
        # Create a simple test that demonstrates the bug
        class MockGUI:
            def __init__(self):
                self.shutdown_dialog = None
                self.agent_state = 'running'
                
            def stopAll_buggy_version(self):
                """This simulates the buggy behavior where dialog doesn't close"""
                # Create dialog (this works)
                if PYQT5_AVAILABLE and QApplication.instance():
                    self.shutdown_dialog = QMessageBox()
                    self.shutdown_dialog.setWindowTitle("Shutting Down")
                    self.shutdown_dialog.setText("Shutting Down. Please Wait.")
                    self.shutdown_dialog.setStandardButtons(QMessageBox.NoButton)
                    self.shutdown_dialog.setModal(True)
                    self.shutdown_dialog.show()
                
                # Simulate shutdown work
                time.sleep(0.1)
                self.agent_state = 'stopped'
                
                # BUG: Dialog close is scheduled incorrectly or not executed
                # In the real bug, QTimer.singleShot might not be working as expected
                
        if PYQT5_AVAILABLE:
            if not QApplication.instance():
                app = QApplication([])
            
            mock_gui = MockGUI()
            mock_gui.stopAll_buggy_version()
            
            # Check if dialog exists and is visible (demonstrating the bug)
            if mock_gui.shutdown_dialog:
                self.assertTrue(mock_gui.shutdown_dialog.isVisible(), 
                              "This test demonstrates the bug - dialog stays visible")
                # Clean up
                mock_gui.shutdown_dialog.close()


if __name__ == '__main__':
    unittest.main()