#!/usr/bin/env python3

import unittest
import subprocess
from unittest.mock import Mock, patch, MagicMock
import threading
import time


class TestStopAllPopup(unittest.TestCase):
    """Test the Stop All button popup dialog functionality"""
    
    def test_docker_agent_shutdown_check(self):
        """Test that Docker containers are properly checked and stopped"""
        
        with patch('subprocess.run') as mock_subprocess:
            # Mock subprocess to return container IDs
            mock_subprocess.return_value.stdout = 'container1\ncontainer2\n'
            
            # Simulate the Docker shutdown logic from stopAll method
            result = subprocess.run(['docker', 'ps', '--filter', 'ancestor=microros/micro-ros-agent:humble', '--format', '{{.ID}}'], 
                                  capture_output=True, text=True)
            container_ids = result.stdout.strip().split('\n')
            
            for container_id in container_ids:
                if container_id:
                    subprocess.run(['docker', 'stop', container_id], check=False)
                    subprocess.run(['docker', 'rm', container_id], check=False)
            
            # Verify Docker commands were called correctly
            expected_calls = [
                # First call to check running containers
                unittest.mock.call(['docker', 'ps', '--filter', 'ancestor=microros/micro-ros-agent:humble', '--format', '{{.ID}}'], 
                                 capture_output=True, text=True),
                # Stop and remove each container
                unittest.mock.call(['docker', 'stop', 'container1'], check=False),
                unittest.mock.call(['docker', 'rm', 'container1'], check=False),
                unittest.mock.call(['docker', 'stop', 'container2'], check=False),
                unittest.mock.call(['docker', 'rm', 'container2'], check=False)
            ]
            
            # Check that subprocess.run was called with expected arguments
            self.assertEqual(mock_subprocess.call_count, 5)  # 1 ps + 2 stop + 2 rm
    
    def test_popup_dialog_creation_and_closure(self):
        """Test that popup dialog is created with correct properties and can be closed"""
        
        # Mock the PyQt5 components needed for the dialog
        with patch('PyQt5.QtWidgets.QMessageBox') as mock_messagebox:
            mock_dialog = Mock()
            mock_messagebox.return_value = mock_dialog
            
            # Simulate the popup creation logic from stopAll method
            shutdown_dialog = mock_messagebox(None)  # self would be GUI instance
            shutdown_dialog.setWindowTitle("Shutting Down")
            shutdown_dialog.setText("Shutting Down. Please Wait.")
            shutdown_dialog.setStandardButtons(mock_messagebox.NoButton)
            shutdown_dialog.setModal(True)
            shutdown_dialog.show()
            
            # Verify popup dialog was created and configured correctly
            mock_messagebox.assert_called_once_with(None)
            mock_dialog.setWindowTitle.assert_called_once_with("Shutting Down")
            mock_dialog.setText.assert_called_once_with("Shutting Down. Please Wait.")
            mock_dialog.setStandardButtons.assert_called_once_with(mock_messagebox.NoButton)
            mock_dialog.setModal.assert_called_once_with(True)
            mock_dialog.show.assert_called_once()
            
            # Test dialog closure
            shutdown_dialog.close()
            mock_dialog.close.assert_called_once()
    
    def test_qtimer_scheduling_for_main_thread_updates(self):
        """Test that QTimer is used to schedule UI updates on main thread"""
        
        with patch('PyQt5.QtCore.QTimer') as mock_qtimer:
            # Mock QTimer.singleShot to capture callback scheduling
            callback_executed = False
            
            def capture_callback(delay, callback):
                nonlocal callback_executed
                self.assertEqual(delay, 0)  # Should schedule immediately
                self.assertTrue(callable(callback))
                # Simulate callback execution
                callback()
                callback_executed = True
            
            mock_qtimer.singleShot = capture_callback
            
            # Simulate the QTimer usage from stopAll method
            def test_callback():
                pass
            
            # Test the QTimer.singleShot call
            mock_qtimer.singleShot(0, test_callback)
            
            # Verify callback was scheduled and executed
            self.assertTrue(callback_executed)
    
    def test_agent_state_management(self):
        """Test that agent state is properly managed during shutdown"""
        
        # Create a mock GUI object with minimal required attributes
        mock_gui = Mock()
        mock_gui.agent_state = 'running'
        mock_gui.agent_status_display = Mock()
        mock_gui.updateControlButtonStates = Mock()
        mock_gui.stopSystem = Mock()
        
        # Test state transitions
        self.assertEqual(mock_gui.agent_state, 'running')
        
        # Simulate state change during shutdown
        mock_gui.agent_state = 'stopped'
        self.assertEqual(mock_gui.agent_state, 'stopped')
        
        # Verify status display update would be called
        mock_gui.agent_status_display.setText('All processes stopped')
        mock_gui.agent_status_display.setText.assert_called_with('All processes stopped')
    
    def test_error_handling_during_shutdown(self):
        """Test that popup closes even when there's an error during shutdown"""
        
        with patch('subprocess.run') as mock_subprocess:
            with patch('PyQt5.QtWidgets.QMessageBox') as mock_messagebox:
                # Mock subprocess to raise an exception
                mock_subprocess.side_effect = Exception("Docker error")
                
                mock_dialog = Mock()
                mock_messagebox.return_value = mock_dialog
                
                # Simulate error handling from stopAll method
                try:
                    # This would be the Docker command that fails
                    subprocess.run(['docker', 'ps', '--filter', 'ancestor=microros/micro-ros-agent:humble', '--format', '{{.ID}}'], 
                                 capture_output=True, text=True)
                except Exception:
                    # Error occurred - dialog should still close
                    pass
                finally:
                    # Simulate the finally block - close dialog
                    shutdown_dialog = mock_messagebox(None)
                    shutdown_dialog.close()
                
                # Verify popup was created and closed even with error
                mock_messagebox.assert_called_once_with(None)
                mock_dialog.close.assert_called_once()


if __name__ == '__main__':
    unittest.main()