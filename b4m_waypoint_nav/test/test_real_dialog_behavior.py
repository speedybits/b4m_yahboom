#!/usr/bin/env python3

import unittest
import time
import threading
from unittest.mock import patch

try:
    from PyQt5.QtWidgets import QApplication, QMessageBox
    from PyQt5.QtCore import QTimer, Qt
    PYQT5_AVAILABLE = True
except ImportError:
    PYQT5_AVAILABLE = False


@unittest.skipUnless(PYQT5_AVAILABLE, "PyQt5 not available")
class TestRealDialogBehavior(unittest.TestCase):
    """Test that exposes real GUI dialog behavior issues"""
    
    @classmethod
    def setUpClass(cls):
        """Set up QApplication for the entire test class"""
        if not QApplication.instance():
            cls.app = QApplication([])
        else:
            cls.app = QApplication.instance()
    
    def test_modal_dialog_blocks_qtimer(self):
        """Test that demonstrates modal dialog blocking QTimer callbacks"""
        
        # This test should FAIL with modal dialogs and PASS with non-modal dialogs
        callback_executed = False
        
        def timer_callback():
            nonlocal callback_executed
            callback_executed = True
        
        # Create a modal dialog
        dialog = QMessageBox()
        dialog.setWindowTitle("Test Modal Dialog")
        dialog.setText("This should block QTimer")
        dialog.setStandardButtons(QMessageBox.NoButton)
        dialog.setModal(True)  # This blocks the event loop
        dialog.show()
        
        # Schedule a QTimer callback
        QTimer.singleShot(50, timer_callback)
        
        # Process events for a short time
        start_time = time.time()
        while time.time() - start_time < 0.2:  # 200ms
            QApplication.processEvents()
            if callback_executed:
                break
            time.sleep(0.01)
        
        # Clean up
        dialog.close()
        
        # With modal dialog, this should FAIL (callback not executed)
        # With non-modal dialog, this should PASS (callback executed)
        self.assertTrue(callback_executed, 
                       "MODAL DIALOG BUG: QTimer callback was blocked by modal dialog!")
    
    def test_non_modal_dialog_allows_qtimer(self):
        """Test that non-modal dialog allows QTimer callbacks to execute"""
        
        callback_executed = False
        
        def timer_callback():
            nonlocal callback_executed
            callback_executed = True
        
        # Create a NON-modal dialog
        dialog = QMessageBox()
        dialog.setWindowTitle("Test Non-Modal Dialog")
        dialog.setText("This should NOT block QTimer")
        dialog.setStandardButtons(QMessageBox.NoButton)
        dialog.setModal(False)  # This does NOT block the event loop
        dialog.setWindowFlags(dialog.windowFlags() | Qt.WindowStaysOnTopHint)
        dialog.show()
        
        # Schedule a QTimer callback
        QTimer.singleShot(50, timer_callback)
        
        # Process events for a short time
        start_time = time.time()
        while time.time() - start_time < 0.2:  # 200ms
            QApplication.processEvents()
            if callback_executed:
                break
            time.sleep(0.01)
        
        # Clean up
        dialog.close()
        
        # This should PASS (callback executed)
        self.assertTrue(callback_executed, 
                       "Non-modal dialog should allow QTimer callbacks to execute")
    
    def test_dialog_close_from_qtimer_callback(self):
        """Test that dialog can be closed from QTimer callback"""
        
        dialog = QMessageBox()
        dialog.setWindowTitle("Test Auto-Close Dialog")
        dialog.setText("This dialog should auto-close")
        dialog.setStandardButtons(QMessageBox.NoButton)
        dialog.setModal(False)  # Non-modal to avoid blocking
        dialog.show()
        
        # Verify dialog is visible
        self.assertTrue(dialog.isVisible(), "Dialog should be visible initially")
        
        # Schedule dialog to close via QTimer
        def close_dialog():
            dialog.close()
        
        QTimer.singleShot(100, close_dialog)
        
        # Process events and wait for close
        start_time = time.time()
        while time.time() - start_time < 0.3:  # 300ms
            QApplication.processEvents()
            if not dialog.isVisible():
                break
            time.sleep(0.01)
        
        # Dialog should be closed by now
        self.assertFalse(dialog.isVisible(), 
                        "Dialog should have been closed by QTimer callback")
    
    def test_threading_with_qtimer_dialog_close(self):
        """Test closing dialog from background thread using QTimer"""
        
        dialog = QMessageBox()
        dialog.setWindowTitle("Test Threading Dialog")
        dialog.setText("This dialog should be closed by background thread")
        dialog.setStandardButtons(QMessageBox.NoButton)
        dialog.setModal(False)  # Non-modal to avoid blocking
        dialog.show()
        
        # Verify dialog is visible
        self.assertTrue(dialog.isVisible(), "Dialog should be visible initially")
        
        # Close dialog from background thread using QTimer
        def background_task():
            time.sleep(0.1)  # Simulate some work
            QTimer.singleShot(0, lambda: dialog.close())
        
        thread = threading.Thread(target=background_task)
        thread.start()
        
        # Process events and wait for close
        start_time = time.time()
        while time.time() - start_time < 0.5:  # 500ms
            QApplication.processEvents()
            if not dialog.isVisible():
                break
            time.sleep(0.01)
        
        thread.join()  # Ensure thread completes
        
        # Dialog should be closed by now
        self.assertFalse(dialog.isVisible(), 
                        "Dialog should have been closed by background thread via QTimer")


if __name__ == '__main__':
    unittest.main()