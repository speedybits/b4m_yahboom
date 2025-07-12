#!/usr/bin/env python3

import unittest
import time
import threading

try:
    from PyQt5.QtWidgets import QApplication, QDialog, QVBoxLayout, QLabel
    from PyQt5.QtCore import QTimer, Qt
    PYQT5_AVAILABLE = True
except ImportError:
    PYQT5_AVAILABLE = False


@unittest.skipUnless(PYQT5_AVAILABLE, "PyQt5 not available")
class TestCustomDialogClosing(unittest.TestCase):
    """Test that custom QDialog closes properly unlike QMessageBox"""
    
    @classmethod
    def setUpClass(cls):
        """Set up QApplication for the entire test class"""
        if not QApplication.instance():
            cls.app = QApplication([])
        else:
            cls.app = QApplication.instance()
    
    def test_custom_dialog_closes_from_qtimer(self):
        """Test that custom QDialog closes properly from QTimer callback"""
        
        # Create custom dialog (like our fixed implementation)
        dialog = QDialog()
        dialog.setWindowTitle("Test Custom Dialog")
        dialog.setModal(False)
        dialog.setWindowFlags(dialog.windowFlags() | Qt.WindowStaysOnTopHint)
        dialog.resize(250, 100)
        
        layout = QVBoxLayout(dialog)
        label = QLabel("This should auto-close")
        label.setAlignment(Qt.AlignCenter)
        layout.addWidget(label)
        
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
                        "Custom QDialog should close properly from QTimer callback")
    
    def test_custom_dialog_with_threading(self):
        """Test custom QDialog closing from background thread via QTimer"""
        
        # Create custom dialog
        dialog = QDialog()
        dialog.setWindowTitle("Test Threading Custom Dialog")
        dialog.setModal(False)
        dialog.resize(250, 100)
        
        layout = QVBoxLayout(dialog)
        label = QLabel("Background thread should close this")
        label.setAlignment(Qt.AlignCenter)
        layout.addWidget(label)
        
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
                        "Custom QDialog should be closed by background thread via QTimer")


if __name__ == '__main__':
    unittest.main()