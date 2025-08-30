#!/usr/bin/env python3

"""
B4M API Ping Test Script
Tests the B4M API endpoint with robot navigation messages
"""

import requests
import json
import random
import time
import sys
import os
from datetime import datetime

class B4MPingTest:
    def __init__(self):
        # Get API key from environment variable
        self.api_key = os.environ.get('B4M_API_KEY')
        if not self.api_key:
            print("ERROR: B4M_API_KEY environment variable not set!")
            print("Please set it: export B4M_API_KEY='your_key_here'")
            sys.exit(1)
        
        # Get user ID from environment variable or use default
        self.user_id = os.environ.get('B4M_USER_ID', '65563f622213b120cd1d9592')
        if self.user_id != '65563f622213b120cd1d9592':
            print(f"Using custom user ID: {self.user_id}")
            
        # B4M API endpoint (as documented in B4M_API.md)
        self.api_url = "https://app.bike4mind.com/api/ai/llm"
        
        # Use the sessionId from B4M_API.md (must be an existing session)
        self.session_id = os.environ.get('B4M_SESSION_ID', '68b1e0fcac3f77504fce09b5')
        
        # Obstacle detection parameters
        self.directions = ["left", "front", "right", "behind"]
        
    def generate_random_obstacle_message(self):
        """Generate a random obstacle detection message for robot navigation"""
        # 30% chance of clear path
        if random.random() < 0.3:
            return self.generate_clear_path_message()
        else:
            return self.generate_obstacle_message()
    
    def generate_clear_path_message(self):
        """Generate a message for clear path scenario"""
        return ("You are a navigation AI for a robot. The path is completely clear.\n\n"
                "CURRENT SITUATION:\n"
                "FRONT: CLEAR - Open space for at least 2m\n"
                "LEFT: CLEAR - Open space for at least 2m\n"
                "RIGHT: CLEAR - Open space for at least 2m\n"
                "BEHIND: CLEAR - Open space for at least 2m\n\n"
                "What navigation action should the robot take?")
    
    def generate_obstacle_message(self):
        """Generate a message with random obstacles"""
        obstacles = {}
        
        for direction in self.directions:
            rand = random.random()
            if rand < 0.3:  # 30% chance of blocked
                distance = round(random.uniform(0.15, 0.30), 2)
                obstacles[direction] = f"BLOCKED - Wall at {distance}m ({int(distance*39.37)} inches)"
            elif rand < 0.6:  # 30% chance of narrow
                distance = round(random.uniform(0.35, 0.60), 2)
                obstacles[direction] = f"NARROW - Wall at {distance}m ({int(distance*39.37)} inches)"
            else:  # 40% chance of clear
                distance = round(random.uniform(1.0, 3.0), 2)
                obstacles[direction] = f"CLEAR - Open space, nearest obstacle at {distance}m"
        
        message = ("You are a navigation AI for a robot. Based on the following spatial description, "
                  "decide the best action for the robot to take.\n\n"
                  "CURRENT SITUATION:\n")
        
        for direction in ["FRONT", "LEFT", "RIGHT", "BEHIND"]:
            key = direction.lower()
            message += f"{direction}: {obstacles[key]}\n"
        
        message += ("\nAVAILABLE ACTIONS:\n"
                   "- \"turn_left\": Rotate 90 degrees to the left\n"
                   "- \"turn_right\": Rotate 90 degrees to the right\n"
                   "- \"go_straight\": Continue moving forward\n"
                   "- \"turn_around\": Rotate 180 degrees\n\n"
                   "Respond with a JSON object containing:\n"
                   "- \"action\": one of the available actions\n"
                   "- \"reason\": brief explanation for the decision\n"
                   "- \"confidence\": confidence level (0.0 to 1.0)\n\n"
                   "Example: {\"action\": \"turn_left\", \"reason\": \"Front blocked, left side clear\", \"confidence\": 0.95}")
        
        return message
    
    def send_ping_request(self, message):
        """Send a ping request to B4M API with polling support"""
        headers = {
            "X-API-Key": self.api_key,
            "Content-Type": "application/json"
        }
        
        payload = {
            "sessionId": self.session_id,
            "message": message,
            "historyCount": 10,
            "fabFileIds": [],
            "messageFileIds": [],
            "params": {
                "model": "gpt-4o-mini",
                "temperature": 0.3,  # Low temperature for consistent navigation decisions
                "max_tokens": 100,
                "stream": False
            },
            "promptMeta": {
                "session": {
                    "id": self.session_id,  # Must match sessionId above
                    "userId": self.user_id
                }
            }
        }
        
        try:
            print(f"\n📤 Sending request to B4M API...")
            print(f"   Session ID: {self.session_id}")
            print(f"   User ID: {self.user_id}")
            print(f"   Model: gpt-4o-mini")
            print(f"   Message length: {len(message)} characters")
            
            start_time = time.time()
            response = requests.post(
                self.api_url,
                headers=headers,
                json=payload,
                timeout=10.0  # 10 second timeout
            )
            elapsed_time = time.time() - start_time
            
            if response.status_code == 200:
                initial_result = response.json()
                print(f"\n✅ Initial response received in {elapsed_time:.2f} seconds")
                
                # Check if we need to poll for the result
                # B4M API returns status "running" when processing, empty replies when not ready
                if (initial_result.get('status') == 'running' or 
                    (initial_result.get('replies') is not None and len(initial_result.get('replies', [])) == 0)):
                    print("   Status: Processing... need to poll for result")
                    return self.poll_for_response(initial_result, headers)
                elif 'replies' in initial_result and len(initial_result.get('replies', [])) > 0:
                    # Direct response with content
                    print("   Status: Complete with replies")
                    return initial_result
                else:
                    # Unclear status, try polling anyway
                    print("   Status: Unclear, attempting to poll...")
                    return self.poll_for_response(initial_result, headers)
            elif response.status_code == 202:
                # Accepted - need to poll
                initial_result = response.json()
                print(f"\n⏳ Request accepted, polling for response...")
                return self.poll_for_response(initial_result, headers)
            else:
                print(f"\n❌ API Error: Status {response.status_code}")
                print(f"   Response: {response.text}")
                return None
                
        except requests.Timeout:
            print(f"\n⏰ Request timed out after 10 seconds")
            return None
        except requests.RequestException as e:
            print(f"\n❌ Request failed: {str(e)}")
            return None
        except json.JSONDecodeError as e:
            print(f"\n❌ Failed to parse response JSON: {str(e)}")
            return None
    
    def poll_for_response(self, initial_response, headers):
        """Poll for the final response if needed"""
        # Try to get quest ID or polling URL from initial response
        quest_id = initial_response.get('questId') or initial_response.get('id')
        
        if not quest_id:
            print("⚠️ No quest ID found in response, returning initial response")
            return initial_response
        
        # Try different polling endpoints based on the API structure
        poll_endpoints = [
            f"https://app.bike4mind.com/api/quests/{quest_id}",  # Quest-specific endpoint
            f"https://app.bike4mind.com/api/ai/llm/{quest_id}",  # LLM with quest ID
            f"https://app.bike4mind.com/api/ai/llm?questId={quest_id}",  # Query param
            f"https://app.bike4mind.com/api/ai/llm?sessionId={self.session_id}&questId={quest_id}"  # Both params
        ]
        
        print(f"📊 Polling for quest ID: {quest_id}")
        
        max_polls = 30  # Maximum 30 seconds of polling
        poll_interval = 1.0  # Poll every 1 second
        
        for i in range(max_polls):
            time.sleep(poll_interval)
            
            # Try each polling endpoint
            for endpoint_idx, poll_url in enumerate(poll_endpoints):
                try:
                    poll_response = requests.get(
                        poll_url,
                        headers=headers,
                        timeout=10.0  # Increased timeout
                    )
                    
                    if poll_response.status_code == 200:
                        result = poll_response.json()
                        
                        # Check if response is complete (has replies with content)
                        if result.get('replies') and len(result.get('replies', [])) > 0:
                            print(f"\n✅ Final response received after {i+1} polls from endpoint {endpoint_idx+1}")
                            print(f"   Successful endpoint: {poll_url}")
                            return result
                        elif result.get('status') == 'done':
                            print(f"\n✅ Final response received after {i+1} polls (status=done)")
                            return result
                        elif result.get('status') == 'error':
                            print(f"\n❌ Error in processing: {result.get('error', 'Unknown error')}")
                            return None
                        else:
                            # Still processing
                            status = result.get('status', 'processing')
                            if endpoint_idx == 0:  # Only print status from first endpoint to avoid spam
                                print(f"   Poll {i+1}: Status = {status}, trying endpoint {endpoint_idx+1}/4", end='\r')
                            continue  # Try next endpoint
                            
                    elif poll_response.status_code == 404:
                        # This endpoint doesn't exist, try next one
                        continue
                        
                except requests.Timeout:
                    if endpoint_idx == 0:  # Only print timeout from first endpoint
                        print(f"\n⚠️ Poll {i+1}: Timeout on endpoint {endpoint_idx+1}, trying next...")
                    continue  # Try next endpoint
                except Exception as e:
                    if endpoint_idx == 0:  # Only print error from first endpoint
                        print(f"\n⚠️ Poll {i+1}: Error on endpoint {endpoint_idx+1}: {str(e)}")
                    continue  # Try next endpoint
            
            # If we get here, all endpoints failed for this poll iteration
            # Continue to next poll cycle
        
        print(f"\n⏰ Polling timeout after {max_polls} attempts")
        return None
    
    def display_response(self, response):
        """Display the B4M API response"""
        if not response:
            return
            
        print("\n📥 B4M API Response:")
        print("=" * 60)
        
        # Extract and display the AI response
        if 'replies' in response and len(response['replies']) > 0:
            ai_response = response['replies'][0]
            print(f"AI Response: {ai_response}")
            
            # Try to parse as navigation command if it looks like JSON
            try:
                if ai_response.strip().startswith('{'):
                    nav_command = json.loads(ai_response)
                    print("\n🤖 Navigation Decision:")
                    print(f"   Action: {nav_command.get('action', 'unknown')}")
                    print(f"   Reason: {nav_command.get('reason', 'no reason provided')}")
                    print(f"   Confidence: {nav_command.get('confidence', 0.0)}")
            except json.JSONDecodeError:
                pass  # Not a JSON response, that's okay
        
        # Display metadata
        if 'promptMeta' in response:
            meta = response['promptMeta']
            if 'tokenUsage' in meta:
                tokens = meta['tokenUsage']
                print(f"\n📊 Token Usage:")
                print(f"   Input: {tokens.get('inputTokens', 0)}")
                print(f"   Output: {tokens.get('outputTokens', 0)}")
                print(f"   Total: {tokens.get('totalTokens', 0)}")
            
            if 'performance' in meta:
                perf = meta['performance']
                print(f"\n⚡ Performance:")
                print(f"   Total Response Time: {perf.get('totalResponseTime', 0)}ms")
                print(f"   Model Inference Time: {perf.get('modelInferenceTime', 0)}ms")
        
        if 'creditsUsed' in response:
            print(f"\n💳 Credits Used: {response['creditsUsed']}")
        
        print("=" * 60)
    
    def run_interactive_test(self):
        """Run interactive ping test"""
        print("\n🤖 B4M API PING TEST")
        print("=" * 60)
        print(f"Endpoint: {self.api_url}")
        print(f"Session ID: {self.session_id}")
        print(f"User ID: {self.user_id}")
        print("API Key: [CONFIGURED VIA ENVIRONMENT]")
        print("=" * 60)
        if self.session_id != '68b1e0fcac3f77504fce09b5':
            print("\n⚠️  Using custom session ID from B4M_SESSION_ID environment variable")
            print("   Make sure this is a valid session for your account!")
        print("\nPress ENTER to send a test message, or CTRL+C to exit\n")
        
        test_count = 0
        
        try:
            while True:
                input("Press ENTER to send next test message...")
                test_count += 1
                
                print(f"\n📝 Test #{test_count}")
                print("-" * 60)
                
                # Generate random message
                message = self.generate_random_obstacle_message()
                
                # Display the message being sent
                print("Message preview (first 200 chars):")
                preview = message[:200] + "..." if len(message) > 200 else message
                print(f"   {preview}")
                
                # Send request and display response
                response = self.send_ping_request(message)
                self.display_response(response)
                
                print("\n")
                
        except KeyboardInterrupt:
            print(f"\n\n👋 Test completed. Sent {test_count} test messages.")
            print(f"   Session ID: {self.session_id}")
            return

def main():
    """Main entry point"""
    tester = B4MPingTest()
    tester.run_interactive_test()

if __name__ == "__main__":
    main()