#!/usr/bin/env python3
"""
RViz Screenshot Capture and Analysis Tool

This tool captures screenshots of RViz and provides them for analysis.
It can be integrated into ROS2 tests to capture visual state.
"""

import subprocess
import os
import time
import sys
from pathlib import Path

class RVizCapture:
    def __init__(self, output_dir="/tmp"):
        self.output_dir = Path(output_dir)
        self.output_dir.mkdir(exist_ok=True)
        
        # Find the capture script
        self.script_dir = Path(__file__).parent
        self.capture_script = self.script_dir / "capture_rviz.sh"
        
        if not self.capture_script.exists():
            raise FileNotFoundError(f"Capture script not found: {self.capture_script}")
    
    def capture(self, name_prefix="rviz", method=None):
        """
        Capture a screenshot of RViz
        
        Args:
            name_prefix: Prefix for the output filename
            method: Screenshot method (scrot, import, xwd, or None for auto)
            
        Returns:
            Path to captured screenshot file
        """
        timestamp = time.strftime("%Y%m%d_%H%M%S")
        output_path = self.output_dir / f"{name_prefix}_{timestamp}.png"
        
        # Build command
        cmd = [str(self.capture_script), str(output_path)]
        if method:
            cmd.append(method)
        
        try:
            print(f"🔍 Capturing RViz screenshot: {name_prefix}")
            result = subprocess.run(cmd, capture_output=True, text=True, timeout=30)
            
            if result.returncode == 0:
                # Script outputs the actual path used
                actual_path = result.stdout.strip().split('\n')[-1]
                if os.path.exists(actual_path):
                    print(f"✅ Screenshot saved: {actual_path}")
                    return Path(actual_path)
                else:
                    print(f"⚠️  Script succeeded but file not found: {actual_path}")
                    return None
            else:
                print(f"❌ Screenshot capture failed:")
                print(f"   stdout: {result.stdout}")
                print(f"   stderr: {result.stderr}")
                return None
                
        except subprocess.TimeoutExpired:
            print("❌ Screenshot capture timed out")
            return None
        except Exception as e:
            print(f"❌ Screenshot capture error: {e}")
            return None
    
    def capture_test_sequence(self, test_name, num_captures=3, interval=2.0):
        """
        Capture a sequence of screenshots during a test
        
        Args:
            test_name: Name of the test for filename prefix
            num_captures: Number of screenshots to take
            interval: Time interval between captures (seconds)
            
        Returns:
            List of Path objects to captured screenshots
        """
        print(f"📸 Starting screenshot sequence for test: {test_name}")
        screenshots = []
        
        for i in range(num_captures):
            name = f"{test_name}_step{i+1:02d}"
            screenshot = self.capture(name)
            
            if screenshot:
                screenshots.append(screenshot)
                print(f"   Captured {i+1}/{num_captures}: {screenshot.name}")
            else:
                print(f"   Failed to capture {i+1}/{num_captures}")
            
            # Wait before next capture (except for last one)
            if i < num_captures - 1:
                time.sleep(interval)
        
        print(f"📸 Screenshot sequence complete: {len(screenshots)}/{num_captures} successful")
        return screenshots
    
    def wait_for_rviz(self, timeout=30):
        """
        Wait for RViz window to appear
        
        Args:
            timeout: Maximum time to wait (seconds)
            
        Returns:
            True if RViz window found, False if timeout
        """
        print("⏳ Waiting for RViz window to appear...")
        start_time = time.time()
        
        while time.time() - start_time < timeout:
            try:
                # Check if RViz process is running
                result = subprocess.run(
                    ["pgrep", "-f", "rviz"], 
                    capture_output=True, 
                    text=True
                )
                
                if result.returncode == 0:
                    # Give RViz a moment to fully load
                    time.sleep(2)
                    print("✅ RViz window detected")
                    return True
                    
            except Exception:
                pass
                
            time.sleep(1)
        
        print("❌ RViz window not detected within timeout")
        return False

def main():
    """Command-line interface for screenshot capture"""
    import argparse
    
    parser = argparse.ArgumentParser(description="Capture RViz screenshots")
    parser.add_argument("--name", default="rviz", help="Screenshot name prefix")
    parser.add_argument("--method", choices=["scrot", "import", "xwd"], help="Screenshot method")
    parser.add_argument("--output-dir", default="/tmp", help="Output directory")
    parser.add_argument("--sequence", type=int, help="Capture sequence of N screenshots")
    parser.add_argument("--interval", type=float, default=2.0, help="Interval between sequence captures")
    parser.add_argument("--wait-for-rviz", action="store_true", help="Wait for RViz before capturing")
    
    args = parser.parse_args()
    
    try:
        capturer = RVizCapture(args.output_dir)
        
        if args.wait_for_rviz:
            if not capturer.wait_for_rviz():
                print("❌ RViz not found, aborting capture")
                return 1
        
        if args.sequence:
            screenshots = capturer.capture_test_sequence(
                args.name, 
                args.sequence, 
                args.interval
            )
            
            if screenshots:
                print(f"\n📋 Captured screenshots:")
                for i, path in enumerate(screenshots, 1):
                    print(f"   {i}. {path}")
                return 0
            else:
                return 1
        else:
            screenshot = capturer.capture(args.name, args.method)
            if screenshot:
                print(f"\n📁 Screenshot: {screenshot}")
                return 0
            else:
                return 1
                
    except Exception as e:
        print(f"❌ Error: {e}")
        return 1

if __name__ == "__main__":
    sys.exit(main())