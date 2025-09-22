#!/usr/bin/env python3
"""
HA_converse - Whisper-based Speech-to-Text with Rolling Buffer
Continuously captures speech and maintains a 500-word rolling buffer
Using faster-whisper for improved performance and smaller footprint
"""

import os
import sys
import time
import signal
import queue
import threading
import argparse
import requests
import json
from datetime import datetime
from pathlib import Path
import numpy as np
import sounddevice as sd
from typing import List, Optional

try:
    from faster_whisper import WhisperModel
    USE_FASTER_WHISPER = True
except ImportError:
    try:
        import whisper
        USE_FASTER_WHISPER = False
    except ImportError:
        print("Error: Neither faster-whisper nor openai-whisper is installed")
        print("Install with: pip3 install faster-whisper")
        print("Or: pip3 install openai-whisper")
        sys.exit(1)


class WhisperSTT:
    """Whisper-based speech-to-text with rolling buffer management"""

    def __init__(self, model_name: str = "base", buffer_size: int = 100, trigger_word: str = "rosie", b4m_enabled: bool = False):
        self.model_name = model_name
        self.buffer_size = buffer_size
        self.trigger_word = trigger_word.lower()  # Case-insensitive
        self.word_buffer: List[str] = []
        self.last_write_time = time.time()
        self.last_word_count = 0
        self.running = False
        self.audio_queue = queue.Queue()
        self.trigger_active = False  # Flag for archiving when buffer is full

        # B4M API integration
        self.b4m_enabled = b4m_enabled
        self.debug_mode = False  # Can be enabled for troubleshooting
        if self.b4m_enabled:
            self._init_b4m()

        # File paths
        self.conversation_file = Path("conversation.txt")
        self.prompts_dir = Path("prompts")

        # Audio settings
        self.sample_rate = 16000  # Whisper expects 16kHz
        self.chunk_duration = 10.0  # 10-second chunks
        self.overlap_duration = 0.5  # 0.5-second overlap

        # Initialize
        self._setup_directories()
        self._load_model()

    def _init_b4m(self):
        """Initialize B4M API configuration"""
        self.b4m_api_key = os.environ.get('B4M_API_KEY')
        if not self.b4m_api_key:
            print("\n⚠️  WARNING: B4M_API_KEY environment variable not set!")
            print("   B4M integration disabled. Set it with: export B4M_API_KEY='your_key_here'")
            self.b4m_enabled = False
            return

        self.b4m_api_url = "https://app.bike4mind.com/api/ai/llm"
        self.b4m_session_id = os.environ.get('B4M_ROSIE_ID', '68b1e0fcac3f77504fce09b5')
        self.b4m_user_id = os.environ.get('B4M_USER_ID', '65563f622213b120cd1d9592')

        print(f"✅ B4M API integration enabled")
        print(f"   Rosie ID: {self.b4m_session_id}")
        print(f"   User ID: {self.b4m_user_id}")

    def _setup_directories(self):
        """Create necessary directories and files"""
        # Create prompts directory if it doesn't exist
        self.prompts_dir.mkdir(exist_ok=True)

        # Start with empty conversation file
        self.conversation_file.write_text("")
        print(f"Initialized empty {self.conversation_file}")

    def _load_model(self):
        """Load Whisper model"""
        print(f"Loading Whisper {self.model_name} model...")
        if USE_FASTER_WHISPER:
            print("Using faster-whisper (optimized)")
            self.model = WhisperModel(self.model_name, device="cpu", compute_type="int8")
        else:
            print("Using openai-whisper")
            self.model = whisper.load_model(self.model_name)
        print(f"Model loaded successfully")

    def _archive_conversation(self):
        """Archive current conversation to timestamped file"""
        if not self.trigger_active:
            return False

        timestamp = datetime.now().strftime("%Y-%m-%d-%H-%M-%S")
        archive_file = self.prompts_dir / f"prompt_{timestamp}.txt"

        # Copy current conversation to archive
        content = self.conversation_file.read_text()
        archive_file.write_text(content)

        print(f"\nArchive created: {archive_file}")

        # Send to B4M API if enabled
        if self.b4m_enabled and content:
            self._send_to_b4m(content)

        # Clear conversation file and buffer
        self.conversation_file.write_text("")
        self.word_buffer.clear()

        # Reset trigger flag
        self.trigger_active = False
        print("Trigger reset - listening for 'Rosie' again")

        return True

    def _send_to_b4m(self, text: str):
        """Send archived text to B4M API and display response"""
        if not self.b4m_api_key:
            return

        print("\n🤖 Sending to B4M AI service...")

        headers = {
            "X-API-Key": self.b4m_api_key,
            "Content-Type": "application/json"
        }

        # Prepare the message for B4M
        message = f"Please respond to the following conversation or query:\n\n{text}"

        payload = {
            "sessionId": self.b4m_session_id,
            "message": message,
            "historyCount": 10,
            "fabFileIds": [],
            "messageFileIds": [],
            "params": {
                "model": "gpt-4o-mini",
                "temperature": 0.7,  # Conversational temperature
                "max_tokens": 500,
                "stream": False
            },
            "promptMeta": {
                "session": {
                    "id": self.b4m_session_id,
                    "userId": self.b4m_user_id
                }
            }
        }

        try:
            start_time = time.time()
            response = requests.post(
                self.b4m_api_url,
                headers=headers,
                json=payload,
                timeout=10.0
            )
            elapsed_time = time.time() - start_time

            if response.status_code == 200:
                result = response.json()
                print(f"✅ B4M response received in {elapsed_time:.2f} seconds")

                # Check if we need to poll for the result
                if (result.get('status') == 'running' or
                    (result.get('replies') is not None and len(result.get('replies', [])) == 0)):
                    print("   Processing... polling for result")
                    result = self._poll_b4m_response(result, headers)

                # Extract and display the response
                self._display_b4m_response(result)

            else:
                print(f"❌ B4M API Error: Status {response.status_code}")
                print(f"   Response: {response.text[:200]}")

        except requests.Timeout:
            print("⏰ B4M request timed out")
        except Exception as e:
            print(f"❌ B4M request failed: {str(e)}")

    def _poll_b4m_response(self, initial_response, headers):
        """Poll B4M API for final response"""
        quest_id = initial_response.get('id')
        if not quest_id:
            return initial_response

        poll_url = f"https://app.bike4mind.com/api/sessions/{self.b4m_session_id}/chat/{quest_id}"
        max_polls = 15
        poll_interval = 7.0

        for i in range(max_polls):
            time.sleep(poll_interval)

            try:
                poll_response = requests.get(poll_url, headers=headers, timeout=10.0)

                if poll_response.status_code == 200:
                    result = poll_response.json()
                    status = result.get('status', 'unknown')

                    # Check status first (like b4m_ping_test.py)
                    if status == 'done':
                        ai_response = self._extract_ai_response(result)
                        if ai_response:
                            print(f"   Response ready after {i+1} polls")
                            return result
                        else:
                            print(f"   Quest done but no AI response found")
                            return result
                    elif status == 'stopped':
                        print(f"   Quest was stopped")
                        return result
                    elif status == 'running':
                        print(f"   Poll {i+1}/{max_polls}: Status = '{status}'")
                    else:
                        # Check if response is ready by content
                        if result.get('replies') and len(result.get('replies', [])) > 0:
                            print(f"   Response ready after {i+1} polls")
                            return result
                        print(f"   Poll {i+1}/{max_polls}: Status = '{status}'")

                else:
                    print(f"   Poll {i+1}: HTTP {poll_response.status_code}")

            except Exception as e:
                print(f"   Poll error: {str(e)}")

        print("   Max polling attempts reached")
        return initial_response

    def _extract_ai_response(self, quest_data):
        """Extract AI response using multiple fallback methods (from b4m_ping_test.py)"""
        # Primary: check replies array (current B4M structure)
        if (quest_data.get('replies') and
            isinstance(quest_data['replies'], list) and
            len(quest_data['replies']) > 0):
            return '\n'.join(quest_data['replies'])

        # Fallback 1: check single reply field (legacy)
        elif quest_data.get('reply'):
            return quest_data['reply']

        # Fallback 2: check questMasterReply
        elif quest_data.get('questMasterReply'):
            return quest_data['questMasterReply']

        # Fallback 3: check Research Mode results
        elif (quest_data.get('researchModeResults') and
              isinstance(quest_data['researchModeResults'], list)):
            results = [r['response'] for r in quest_data['researchModeResults']
                      if r.get('response')]
            if results:
                return '\n\n'.join(results)

        # Fallback 4: check messages array
        elif quest_data.get('messages'):
            for msg in quest_data.get('messages', []):
                if msg.get('role') == 'assistant' and msg.get('content'):
                    return msg['content']

        return None

    def _display_b4m_response(self, response_data):
        """Extract and display B4M AI response"""
        print("\n" + "="*50)
        print("📝 B4M AI Response:")
        print("="*50)

        # Use the comprehensive extraction method
        response_text = self._extract_ai_response(response_data)

        if response_text:
            # Clean up and display the response text
            response_text = response_text.strip()
            print(response_text)
        else:
            print("⚠️ No AI response found in the quest data")
            # Always show debug info when response not found
            print("\n🔍 Debug - Response structure:")
            debug_data = {
                'status': response_data.get('status'),
                'replies': response_data.get('replies', []),
                'reply': response_data.get('reply'),
                'questMasterReply': response_data.get('questMasterReply'),
                'messages': response_data.get('messages', []),
                'available_keys': list(response_data.keys())
            }
            print(json.dumps(debug_data, indent=2)[:800] + "...")

        # Display metadata if available
        if response_data.get('promptMeta'):
            meta = response_data['promptMeta']
            if meta.get('tokenUsage'):
                tokens = meta['tokenUsage']
                print(f"\n📊 Token Usage: {tokens.get('totalTokens', 0)} total")
            if meta.get('performance'):
                perf = meta['performance']
                print(f"⚡ Response Time: {perf.get('totalResponseTime', 0)}ms")

        print("="*50 + "\n")

    def _update_conversation_file(self):
        """Write buffer to conversation file"""
        content = " ".join(self.word_buffer)
        self.conversation_file.write_text(content)

    def _should_write_to_file(self) -> bool:
        """Determine if we should write to file based on time or word count"""
        time_elapsed = time.time() - self.last_write_time >= 10.0
        words_added = len(self.word_buffer) - self.last_word_count >= 50
        return time_elapsed or words_added

    def _remove_consecutive_duplicates(self, words: List[str]) -> List[str]:
        """Remove exact consecutive duplicate phrases"""
        if not words or not self.word_buffer:
            return words

        # Check for overlap duplicates at the boundary
        result = []
        last_word = self.word_buffer[-1] if self.word_buffer else None

        for word in words:
            if word != last_word:
                result.append(word)
            last_word = word

        return result

    def _check_for_trigger(self, text: str) -> bool:
        """Check if trigger word is in the text"""
        return self.trigger_word in text.lower()

    def _process_transcription(self, text: str):
        """Process transcribed text and update buffer"""
        if not text or text.strip() == "":
            return

        # Check for trigger word
        if not self.trigger_active and self._check_for_trigger(text):
            self.trigger_active = True
            print(f"\n🎯 Trigger word 'Rosie' detected - will archive when buffer full")

        # Split into words and clean
        words = text.strip().split()
        words = self._remove_consecutive_duplicates(words)

        if not words:
            return

        # Add to buffer
        if len(self.word_buffer) + len(words) > self.buffer_size:
            # Buffer will overflow
            if self.trigger_active:
                # Archive before adding new words
                words_to_add = self.buffer_size - len(self.word_buffer)
                self.word_buffer.extend(words[:words_to_add])
                self._update_conversation_file()
                self._archive_conversation()
                # Add remaining words to new buffer
                self.word_buffer = words[words_to_add:]
            else:
                # Circular buffer - keep only last buffer_size words
                old_size = len(self.word_buffer)
                self.word_buffer.extend(words)
                self.word_buffer = self.word_buffer[-self.buffer_size:]
                # Show rollover indicator
                words_dropped = (old_size + len(words)) - self.buffer_size
                if words_dropped > 0:
                    print(f"\n↻ Buffer rollover: {words_dropped} oldest word(s) dropped")
        else:
            # Normal addition
            self.word_buffer.extend(words)

        # Check if we should archive (buffer full with trigger)
        if self.trigger_active and len(self.word_buffer) >= self.buffer_size:
            self._update_conversation_file()
            self._archive_conversation()

        # Update display
        word_count = len(self.word_buffer)
        status = " [TRIGGER ACTIVE]" if self.trigger_active else ""
        print(f"\rBuffer: {word_count}/{self.buffer_size} words{status}", end="", flush=True)

        # Write to file if needed
        if self._should_write_to_file():
            self._update_conversation_file()
            self.last_write_time = time.time()
            self.last_word_count = word_count

    def _audio_callback(self, indata, frames, time_info, status):
        """Callback for audio input"""
        if status:
            print(f"\nAudio error: {status}")

        # Add audio to queue for processing
        audio_data = indata.copy().flatten()
        self.audio_queue.put(audio_data)

    def _transcription_worker(self):
        """Worker thread for processing audio chunks"""
        audio_buffer = []
        chunk_samples = int(self.chunk_duration * self.sample_rate)
        overlap_samples = int(self.overlap_duration * self.sample_rate)

        while self.running:
            try:
                # Collect audio data
                audio_chunk = self.audio_queue.get(timeout=0.1)
                audio_buffer.extend(audio_chunk)

                # Process when we have enough audio
                if len(audio_buffer) >= chunk_samples:
                    # Extract chunk with overlap
                    chunk = np.array(audio_buffer[:chunk_samples], dtype=np.float32)

                    # Keep overlap for next chunk
                    audio_buffer = audio_buffer[chunk_samples - overlap_samples:]

                    # Transcribe
                    try:
                        if USE_FASTER_WHISPER:
                            # faster-whisper transcription
                            segments, info = self.model.transcribe(
                                chunk,
                                language='en',
                                beam_size=5,
                                vad_filter=True
                            )
                            text = " ".join([segment.text for segment in segments]).strip()
                        else:
                            # openai-whisper transcription
                            result = self.model.transcribe(
                                chunk,
                                language='en',
                                fp16=False,
                                verbose=False
                            )
                            text = result.get('text', '').strip()

                        if text:
                            self._process_transcription(text)

                    except Exception as e:
                        print(f"\nTranscription error: {e}")
                        self._process_transcription("[inaudible]")

            except queue.Empty:
                continue
            except Exception as e:
                print(f"\nWorker error: {e}")

    def start(self):
        """Start the speech-to-text system"""
        print("Starting HA_converse Speech-to-Text System")
        print(f"Using Whisper {self.model_name} model")
        print(f"Trigger word: 'Rosie' (say it to enable archiving)")
        print(f"Buffer size: {self.buffer_size} words")
        if self.b4m_enabled:
            print(f"B4M API: ENABLED - responses will be sent to AI")
        print(f"Listening on default microphone...")
        print(f"Press Ctrl+C to stop\n")

        self.running = True

        # Start transcription worker thread
        worker_thread = threading.Thread(target=self._transcription_worker, daemon=True)
        worker_thread.start()

        # Start audio stream
        try:
            with sd.InputStream(
                samplerate=self.sample_rate,
                channels=1,
                dtype=np.float32,
                callback=self._audio_callback,
                blocksize=int(self.sample_rate * 0.1)  # 100ms blocks
            ):
                # Keep main thread alive
                while self.running:
                    time.sleep(0.1)

        except KeyboardInterrupt:
            self.stop()
        except Exception as e:
            print(f"\nAudio stream error: {e}")
            self.stop()

    def stop(self):
        """Stop the system and save current buffer"""
        print("\n\nShutting down...")
        self.running = False

        # Process remaining audio
        time.sleep(1)

        # Save current buffer
        if self.word_buffer:
            self._update_conversation_file()
            print(f"Saved {len(self.word_buffer)} words to {self.conversation_file}")

        print("HA_converse stopped")


def signal_handler(signum, frame):
    """Handle Ctrl+C gracefully"""
    sys.exit(0)


def main():
    """Main entry point"""
    # Setup signal handler
    signal.signal(signal.SIGINT, signal_handler)

    # Parse command line arguments
    parser = argparse.ArgumentParser(
        description="HA_converse Speech-to-Text with rolling buffer",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
  %(prog)s                    # Basic mode
  %(prog)s --b4m             # With B4M AI integration

Environment variables for B4M:
  B4M_API_KEY     Required API key for B4M service
  B4M_ROSIE_ID    Optional Rosie session ID (default provided)
  B4M_USER_ID     Optional user ID (default provided)
        """
    )

    parser.add_argument(
        '--b4m',
        action='store_true',
        help='Enable B4M API integration - sends archived prompts to AI service'
    )

    parser.add_argument(
        '--buffer-size',
        type=int,
        default=100,
        help='Buffer size in words (default: 100)'
    )

    parser.add_argument(
        '--model',
        type=str,
        default='base',
        choices=['tiny', 'base', 'small', 'medium', 'large'],
        help='Whisper model size (default: base)'
    )

    args = parser.parse_args()

    # Check for required dependencies
    try:
        import sounddevice
        import numpy
        # Check if we have either whisper implementation
        if not USE_FASTER_WHISPER:
            import whisper
    except ImportError as e:
        print(f"Missing dependency: {e}")
        if USE_FASTER_WHISPER:
            print("Please install requirements: pip install faster-whisper sounddevice numpy")
        else:
            print("Please install requirements: pip install openai-whisper sounddevice numpy")
        sys.exit(1)

    # Check for requests if B4M is enabled
    if args.b4m:
        try:
            import requests
        except ImportError:
            print("Missing dependency for B4M integration: requests")
            print("Please install: pip install requests")
            sys.exit(1)

    # Start the system
    stt = WhisperSTT(
        model_name=args.model,
        buffer_size=args.buffer_size,
        trigger_word="rosie",
        b4m_enabled=args.b4m
    )

    try:
        stt.start()
    except Exception as e:
        print(f"Fatal error: {e}")
        sys.exit(1)


if __name__ == "__main__":
    main()