#!/usr/bin/env python3

import requests
import json
import random
import time
import sys
import select
import termios
import tty
import os

class B4MPingTest:
    def __init__(self):
        # Get API key from environment variable
        self.api_key = os.environ.get('B4M_API_KEY')
        if not self.api_key:
            print("ERROR: B4M_API_KEY environment variable not set!")
            print("Please set it: export B4M_API_KEY='your_key_here'")
            sys.exit(1)
            
        self.base_url = "https://app.bike4mind.com/api"
        self.headers = {
            "X-API-Key": self.api_key,
            "Content-Type": "application/json"
        }
        
        # Obstacle detection parameters
        self.directions = ["left", "front", "right"]
        self.min_distance_inches = 4
        self.max_distance_feet = 4
        
        # Session ID for maintaining context
        self.session_id = None
        
    def generate_random_obstacle_message(self):
        """Generate a random obstacle detection message"""
        # 30% chance of no obstacles
        if random.random() < 0.3:
            base_message = "No obstacles detected in any direction. Path is clear for navigation."
        else:
            # Generate random obstacles
            obstacles = []
            num_obstacles = random.randint(1, 3)
            detected_directions = random.sample(self.directions, num_obstacles)
            
            for direction in detected_directions:
                # Random distance between 4 inches and 4 feet
                if random.random() < 0.5:
                    # Distance in inches
                    distance = random.randint(self.min_distance_inches, 12)
                    unit = "inches"
                else:
                    # Distance in feet
                    distance = random.randint(1, self.max_distance_feet)
                    unit = "feet"
                
                obstacles.append(f"Obstacle detected {direction}: {distance} {unit} away")
            
            base_message = "Robot obstacle detection report: " + "; ".join(obstacles)
        
        # Add the decision instruction at the end
        message = base_message + '. Please decide what you would like to do. Respond only with "Stop", "Go Left", "Go Right" or "Go Straight", or "Back-up".'
        return message
    
    def create_session(self):
        """Create a new session/notebook for the robot"""
        try:
            print("📓 Creating new session...")
            response = requests.post(
                f"{self.base_url}/sessions/create",
                headers=self.headers,
                json={"name": "B4M_Robot_Session"},
                timeout=10
            )
            
            if response.status_code == 200:
                response_data = response.json()
                # Session ID might be in different fields
                self.session_id = response_data.get('id') or response_data.get('sessionId') or response_data.get('session_id')
                print(f"✅ Session created: {self.session_id}")
                return True
            else:
                print(f"❌ Failed to create session: {response.status_code}")
                print(f"   {response.text}")
                return False
                
        except Exception as e:
            print(f"❌ Error creating session: {e}")
            return False
    
    def send_chat_message(self, message):
        """Send a message to the bike4mind chat API"""
        data = {
            "message": message,
            "model": "gpt-4o-mini",
            "temperature": 0.7,
            "max_tokens": 500
        }
        
        # Add sessionID if we have one
        if self.session_id:
            data["sessionID"] = self.session_id
        
        try:
            print(f"\n🤖 Sending obstacle report:")
            print(f"   {message}")
            print("\n📡 Waiting for bike4mind response...")
            
            response = requests.post(
                f"{self.base_url}/chat",
                headers=self.headers,
                json=data,
                timeout=30
            )
            
            if response.status_code == 200:
                response_data = response.json()
                
                # Since it's async, we'll just acknowledge the message was sent
                if response_data.get('status') == 'queued':
                    quest_id = response_data.get('id') or response_data.get('tracking_info', {}).get('quest_id')
                    print(f"\n✅ Message sent successfully!")
                    print(f"🆔 Quest ID: {quest_id}")
                    print(f"📝 Status: Processing asynchronously")
                    print(f"\n💡 Note: Since the API is async, responses will be processed")
                    print(f"   in the background and may be available via the web interface.")
                    
                    if self.session_id:
                        print(f"📓 Session ID: {self.session_id}")
                elif 'response' in response_data:
                    # In case we get a direct response
                    print(f"\n🤖 AI Response: {response_data['response']}")
                elif 'completion' in response_data:
                    print(f"\n🤖 AI Response: {response_data['completion']}")
                else:
                    print(f"\n📦 Response received:")
                    print(f"   {json.dumps(response_data, indent=2)}")
                    
            else:
                print(f"\n❌ Error {response.status_code}: {response.text}")
                
        except requests.exceptions.Timeout:
            print("\n⏰ Request timed out after 30 seconds")
        except requests.exceptions.RequestException as e:
            print(f"\n❌ Request failed: {e}")
        except Exception as e:
            print(f"\n❌ Unexpected error: {e}")
    
    def wait_for_keypress(self):
        """Wait for any keypress or return True for CTRL+C"""
        print("\n⌨️  Press any key to send another message, or CTRL+C to stop...")
        
        try:
            # Check if stdin is a terminal
            if not sys.stdin.isatty():
                print("⚠️  Non-interactive mode detected. Exiting...")
                return False
            
            # Save current terminal settings
            old_settings = termios.tcgetattr(sys.stdin)
            try:
                # Set terminal to raw mode to capture single keypress
                tty.setraw(sys.stdin.fileno())
                
                # Wait for a single character
                char = sys.stdin.read(1)
                
                # Check for CTRL+C (ASCII 3)
                if ord(char) == 3:
                    return False
                    
                return True
                
            except KeyboardInterrupt:
                return False
            finally:
                # Restore terminal settings
                termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)
                
        except (termios.error, OSError):
            print("⚠️  Terminal not available for interactive input. Exiting...")
            return False
    
    def test_connection(self):
        """Test connection to bike4mind API"""
        try:
            print("🔍 Testing connection to bike4mind API...")
            response = requests.get(
                f"{self.base_url}/sessions",
                headers=self.headers,
                timeout=10
            )
            
            if response.status_code == 200:
                print("✅ Connection successful!")
                return True
            else:
                print(f"❌ Connection failed: {response.status_code}")
                return False
                
        except Exception as e:
            print(f"❌ Connection test failed: {e}")
            return False
    
    def run(self):
        """Main test loop"""
        print("🚀 B4M Ping Test - Obstacle Detection API Communication")
        print("=" * 60)
        
        # Test connection first
        if not self.test_connection():
            print("\n❌ Cannot connect to bike4mind API. Exiting.")
            return
        
        # Create a session for this test run
        if not self.create_session():
            print("\n⚠️  Could not create session. Continuing without session ID...")
            print("   Messages will still be sent but may not maintain context.")
        
        print("\n🎯 Starting interactive obstacle detection test...")
        
        try:
            while True:
                # Generate and send random obstacle message
                obstacle_message = self.generate_random_obstacle_message()
                self.send_chat_message(obstacle_message)
                
                # Wait for keypress to continue or CTRL+C to exit
                if not self.wait_for_keypress():
                    break
                    
                print("\n" + "─" * 60)
                
        except KeyboardInterrupt:
            pass
        
        print("\n\n👋 B4M Ping Test completed. Goodbye!")

def main():
    test = B4MPingTest()
    test.run()

if __name__ == "__main__":
    main()