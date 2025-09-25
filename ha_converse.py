#!/usr/bin/env python3
"""
HA_converse - Speech-to-Text Application with B4M AI Integration
A continuous speech recognition system that processes 20-word chunks through B4M AI
and provides voice responses via Piper TTS.
"""

import argparse
import os
import signal
import sys
import threading
import time
import json
import glob
import re
from datetime import datetime
from pathlib import Path
from typing import Optional, List, Dict, Any

# Audio and speech recognition
try:
    from faster_whisper import WhisperModel
    WHISPER_BACKEND = "faster-whisper"
except ImportError:
    try:
        import whisper
        WHISPER_BACKEND = "openai-whisper"
    except ImportError:
        print("❌ Error: Neither faster-whisper nor openai-whisper is installed")
        print("Please install one of them: pip install faster-whisper")
        sys.exit(1)

import sounddevice as sd
import numpy as np

# HTTP requests for B4M API
import requests

# Audio processing and TTS
try:
    import piper
    PIPER_AVAILABLE = True
except ImportError:
    PIPER_AVAILABLE = False


class HAConverseApp:
    """Main application class for HA_converse speech-to-text system"""

    def __init__(self, interactive_mode: bool = False, test_mode: bool = False):
        # Configuration
        self.interactive_mode = interactive_mode
        self.test_mode = test_mode
        self.buffer_size = 20
        self.trigger_word = "rosie"
        self.silence_timeout = 3.0
        self.chunk_duration = 10  # seconds
        self.sample_rate = 16000
        self.test_sentence_interval = 3.0

        # Threading and synchronization
        self.shutdown_event = threading.Event()
        self.voice_activity_flag = threading.Lock()
        self.tts_interrupt_flag = threading.Event()
        self.response_queue_lock = threading.Lock()
        self.b4m_processing_flag = threading.Lock()

        # Speech recognition
        self.whisper_model = None
        self.word_buffer = []
        self.last_speech_time = time.time()
        self.silence_triggered = False

        # File counter for 1:1 mapping
        self.file_counter = 1
        self.file_counter_lock = threading.Lock()

        # Test mode
        self.test_sentences = []
        self.test_sentence_index = 0

        # B4M API configuration
        self.b4m_enabled = True  # Always enabled as per spec
        self.b4m_api_key = os.getenv('B4M_API_KEY')
        self.b4m_rosie_id = os.getenv('B4M_ROSIE_ID')
        self.b4m_user_id = os.getenv('B4M_USER_ID', 'test_user')

        # Piper TTS configuration
        self.piper_enabled = True  # Always enabled as per spec
        self.piper_model = None
        self.piper_model_path = os.getenv('PIPER_MODEL_PATH')
        self.piper_config_path = os.getenv('PIPER_CONFIG_PATH')

        # Threads
        self.speech_thread = None
        self.tts_thread = None
        self.b4m_thread = None

        # Setup signal handlers
        self.setup_signal_handlers()

    def setup_signal_handlers(self):
        """Setup graceful shutdown handlers for SIGINT and SIGTERM"""
        def signal_handler(signum, frame):
            print(f"\n🛑 Received signal {signum} - initiating graceful shutdown...")
            self.shutdown_event.set()

        signal.signal(signal.SIGINT, signal_handler)
        signal.signal(signal.SIGTERM, signal_handler)

    def cleanup_files(self):
        """Delete all existing conversation and response files"""
        # Get files but exclude the test file
        conversation_files = [f for f in glob.glob("conversation_*.txt") if f != "conversation_test.txt"]
        response_files = glob.glob("response_*.txt")

        for file_path in conversation_files + response_files:
            try:
                os.remove(file_path)
                print(f"🗑️ Cleaned up {file_path}")
            except OSError as e:
                print(f"⚠️ Could not remove {file_path}: {e}")

    def load_whisper_model(self):
        """Load the Whisper model for speech recognition"""
        print("🎤 Initializing Whisper base model...")

        try:
            if WHISPER_BACKEND == "faster-whisper":
                self.whisper_model = WhisperModel(
                    "base",
                    compute_type="int8",
                    device="cpu"
                )
            else:  # openai-whisper
                self.whisper_model = whisper.load_model("base")

            print("✅ Whisper model loaded successfully")
            return True

        except Exception as e:
            print(f"❌ Failed to load Whisper model: {e}")
            return False

    def initialize_piper(self):
        """Initialize Piper TTS system"""
        if not PIPER_AVAILABLE:
            print("⚠️ Piper TTS not available - continuing without voice output")
            return False

        try:
            if self.piper_model_path and self.piper_config_path:
                if os.path.exists(self.piper_model_path) and os.path.exists(self.piper_config_path):
                    # Load real Piper model
                    self.piper_model = piper.PiperVoice.load(
                        self.piper_model_path,
                        config_path=self.piper_config_path
                    )
                    print("🔊 Piper TTS initialized with real voice model")
                else:
                    print("⚠️ Piper voice model files not found - using simulation mode")
                    self.piper_model = "simulation"
            else:
                print("⚠️ Piper model paths not configured - using simulation mode")
                self.piper_model = "simulation"

            # Test TTS with startup message
            self.speak_text("Hello World! Piper text-to-speech is working correctly.")
            return True

        except Exception as e:
            print(f"❌ Failed to initialize Piper TTS: {e}")
            return False

    def load_test_sentences(self):
        """Load test sentences from conversation_test.txt"""
        try:
            with open("conversation_test.txt", "r", encoding="utf-8") as f:
                self.test_sentences = [line.strip() for line in f if line.strip()]

            print(f"📄 Loaded {len(self.test_sentences)} test sentences")
            return len(self.test_sentences) > 0

        except FileNotFoundError:
            print("❌ conversation_test.txt not found - test mode unavailable")
            return False
        except Exception as e:
            print(f"❌ Error loading test sentences: {e}")
            return False

    def speak_text(self, text: str):
        """Speak text using Piper TTS"""
        if not self.piper_enabled or not self.piper_model:
            return

        try:
            if self.piper_model == "simulation":
                # Simulation mode
                print(f"🔊 [SIMULATION] Speaking: \"{text}\"")
                # Simulate speaking time
                words = len(text.split())
                speak_duration = max(1.0, words * 0.3)  # ~0.3 seconds per word

                # Interruptible sleep
                start_time = time.time()
                while time.time() - start_time < speak_duration:
                    if self.shutdown_event.is_set() or self.tts_interrupt_flag.is_set():
                        print("⏹️ Speech simulation interrupted")
                        return
                    time.sleep(0.1)
            else:
                # Real Piper TTS
                print(f"🔊 Speaking: \"{text}\"")
                audio_chunks = []

                for audio_chunk in self.piper_model.synthesize(text):
                    if self.shutdown_event.is_set() or self.tts_interrupt_flag.is_set():
                        print("⏹️ Speech interrupted during synthesis")
                        return

                    # Convert AudioChunk to numpy array
                    if hasattr(audio_chunk, 'audio_float_array'):
                        # Piper AudioChunk has audio_float_array property
                        audio_data = np.array(audio_chunk.audio_float_array, dtype=np.float32)
                    elif hasattr(audio_chunk, 'audio'):
                        # Fallback: audio property
                        audio_data = np.array(audio_chunk.audio, dtype=np.float32)
                    else:
                        # Direct audio array
                        audio_data = np.array(audio_chunk, dtype=np.float32)

                    if audio_data.size > 0:
                        audio_chunks.append(audio_data)

                # Only concatenate if we have audio chunks
                if audio_chunks:
                    audio_array = np.concatenate(audio_chunks)

                    # Play audio with interruption checking
                    sd.play(audio_array, samplerate=22050)
                    while sd.get_stream().active:
                        if self.shutdown_event.is_set() or self.tts_interrupt_flag.is_set():
                            sd.stop()
                            print("⏹️ Speech interrupted - stopping playback")
                            return
                        time.sleep(0.1)
                else:
                    print("⚠️ No audio data generated for text")

        except Exception as e:
            print(f"❌ [TTS Thread] Error during speech synthesis: {e}")

    def create_conversation_file(self, content: str) -> tuple[str, int]:
        """Create a timestamped conversation file with integer counter"""
        timestamp = datetime.now().strftime("%Y-%m-%d_%H-%M-%S")

        # Get next counter value thread-safely
        with self.file_counter_lock:
            current_counter = self.file_counter
            self.file_counter += 1

        # Create filename with integer counter
        filename = f"conversation_{timestamp}__{current_counter:03d}.txt"

        try:
            with open(filename, "w", encoding="utf-8") as f:
                f.write(content)

            print(f"💾 Conversation saved to {filename}")
            return filename, current_counter

        except Exception as e:
            print(f"❌ Error creating conversation file: {e}")
            return "", 0

    def create_response_file(self, content: str, conversation_counter: int) -> str:
        """Create a timestamped response file with matching counter"""
        timestamp = datetime.now().strftime("%Y-%m-%d_%H-%M-%S")

        # Create filename with matching integer counter from conversation file
        filename = f"response_{timestamp}__{conversation_counter:03d}.txt"

        try:
            with open(filename, "w", encoding="utf-8") as f:
                f.write(content)

            print(f"💾 AI response saved to {filename}")
            return filename

        except Exception as e:
            print(f"❌ Error creating response file: {e}")
            return ""

    def extract_counter_from_filename(self, filename: str) -> int:
        """Extract integer counter from filename"""
        try:
            # Extract counter from filename like conversation_2025-01-01_12-00-00__001.txt
            if '__' in filename:
                counter_part = filename.split('__')[1].split('.')[0]
                return int(counter_part)
            return 0
        except (IndexError, ValueError):
            return 0

    def get_oldest_conversation_file(self) -> Optional[str]:
        """Get the oldest conversation file by timestamp"""
        conversation_files = [f for f in glob.glob("conversation_*.txt") if f != "conversation_test.txt"]
        if not conversation_files:
            return None

        # Sort by timestamp in filename
        conversation_files.sort(key=lambda x: os.path.getctime(x))
        return conversation_files[0]

    def get_oldest_response_file(self) -> Optional[str]:
        """Get the oldest response file by timestamp"""
        response_files = glob.glob("response_*.txt")
        if not response_files:
            return None

        # Sort by timestamp in filename
        response_files.sort(key=lambda x: os.path.getctime(x))
        return response_files[0]

    def process_b4m_api(self, conversation_file: str, retry_count: int = 0):
        """Process conversation file through B4M API with proper quest-based polling"""
        if not self.b4m_enabled or not self.b4m_api_key or not self.b4m_rosie_id:
            return

        try:
            # Extract counter from conversation filename for 1:1 mapping
            conversation_counter = self.extract_counter_from_filename(conversation_file)

            # Read conversation file
            with open(conversation_file, "r", encoding="utf-8") as f:
                message_content = f.read().strip()

            print("🤖 Processing conversation file with B4M AI...")

            # Prepare API request
            headers = {
                "X-API-Key": self.b4m_api_key,
                "Content-Type": "application/json"
            }

            # Add Rosie-style response instruction
            rosie_prompt = f"You are Rosie, a helpful robot assistant. Respond to the following in a single sentence like Rosie the robot would, being helpful and friendly: {message_content}"

            payload = {
                "sessionId": self.b4m_rosie_id,
                "message": rosie_prompt,
                "historyCount": 10,
                "fabFileIds": [],
                "messageFileIds": [],
                "params": {
                    "model": "gpt-4o-mini",
                    "temperature": 0.7,
                    "max_tokens": 1000
                },
                "promptMeta": {
                    "userId": self.b4m_user_id,
                    "sessionId": self.b4m_rosie_id,
                    "timestamp": datetime.now().isoformat()
                }
            }

            # Make initial API request to create quest
            response = requests.post(
                "https://app.bike4mind.com/api/ai/llm",
                headers=headers,
                json=payload,
                timeout=10
            )

            if response.status_code == 200:
                data = response.json()
                quest_id = data.get('id') or data.get('questId')

                if quest_id:
                    # Always poll for quest-based system
                    ai_response = self.poll_b4m_response(quest_id)

                    if ai_response:
                        # Save response to file
                        self.create_response_file(ai_response, conversation_counter)

                        # Delete processed conversation file
                        os.remove(conversation_file)
                        print(f"🗑️ Deleted {conversation_file} after processing")
                    else:
                        print(f"⚠️ B4M API: Quest {quest_id} failed to complete or extract response")
                else:
                    print("⚠️ B4M API: No quest ID returned from initial request")

            elif response.status_code == 429:
                # Handle rate limiting
                self.handle_rate_limit_error(response, conversation_file, retry_count)

            else:
                print(f"⚠️ B4M API failed: {response.status_code} - {response.text}")
                if retry_count < 3:
                    print(f"⚠️ Retrying {conversation_file} (attempt {retry_count + 1}/3)")
                    time.sleep(5)  # Brief delay before retry
                    self.process_b4m_api(conversation_file, retry_count + 1)
                else:
                    print(f"❌ B4M API failed after 3 attempts - keeping {conversation_file} for manual retry")

        except requests.Timeout:
            print(f"⚠️ B4M API timeout for {conversation_file}")
            if retry_count < 3:
                print(f"⚠️ Retrying {conversation_file} (attempt {retry_count + 1}/3) after timeout")
                time.sleep(5)
                self.process_b4m_api(conversation_file, retry_count + 1)
        except Exception as e:
            print(f"⚠️ B4M API error for {conversation_file}: {e}")
            if retry_count < 3:
                print(f"⚠️ Retrying {conversation_file} (attempt {retry_count + 1}/3) after error")
                time.sleep(5)
                self.process_b4m_api(conversation_file, retry_count + 1)

    def handle_rate_limit_error(self, response, conversation_file: str, retry_count: int):
        """Handle B4M API rate limiting (HTTP 429) with proper retry-after parsing"""
        if retry_count >= 3:
            print(f"❌ B4M API failed after 3 attempts - keeping {conversation_file} for manual retry")
            return

        # Parse retry-after time from response
        retry_after = 60  # Default fallback
        try:
            response_data = response.json()
            if 'error' in response_data:
                error_msg = response_data['error']
                # Try to extract retry time from error message like "Try again in 59.165 seconds"
                import re
                match = re.search(r'Try again in (\d+(?:\.\d+)?) seconds', error_msg)
                if match:
                    retry_after = int(float(match.group(1)))
        except:
            # Check headers for Retry-After
            retry_after_header = response.headers.get('Retry-After')
            if retry_after_header and retry_after_header.isdigit():
                retry_after = int(retry_after_header)

        # Apply exponential backoff for repeated 429s
        if retry_count > 0:
            retry_after = retry_after * (2 ** retry_count)
            retry_after = min(retry_after, 240)  # Cap at 240 seconds

        print(f"⚠️ B4M API: Rate limit exceeded. Try again in {retry_after}s")
        print(f"⏱️ Rate limited - waiting {retry_after}s before retry (attempt {retry_count + 1}/3)")

        # Wait for rate limit period
        for i in range(retry_after):
            if self.shutdown_event.is_set():
                return
            time.sleep(1)

        print("✅ Rate limit wait complete - retrying B4M request")
        self.process_b4m_api(conversation_file, retry_count + 1)

    def poll_b4m_response(self, quest_id: str) -> Optional[str]:
        """Poll B4M API for quest completion with proper status messages"""
        headers = {"X-API-Key": self.b4m_api_key}
        polling_url = f"https://app.bike4mind.com/api/sessions/{self.b4m_rosie_id}/chat/{quest_id}"

        for attempt in range(15):  # Max 15 attempts (105 seconds)
            if self.shutdown_event.is_set():
                return None

            try:
                # Wait 7 seconds between polls (standard B4M interval)
                time.sleep(7)

                print(f"📡 Polling B4M quest status (attempt {attempt + 1}/15)...")
                response = requests.get(polling_url, headers=headers, timeout=5)

                if response.status_code == 200:
                    data = response.json()
                    status = data.get('status')

                    if status == 'done':
                        print("✅ Quest complete - extracting AI response")
                        return self.extract_b4m_response(data)
                    elif status == 'running':
                        print("⏳ Quest still running, polling again in 7s...")
                        continue
                    elif status == 'stopped':
                        print("⚠️ B4M quest was stopped/cancelled")
                        return None
                    else:
                        print(f"⚠️ B4M quest status unknown: {status}")
                elif response.status_code == 429:
                    print("⚠️ B4M polling rate limited - continuing with exponential backoff")
                    time.sleep(7)  # Additional delay for rate limiting
                    continue
                else:
                    print(f"⚠️ B4M polling HTTP {response.status_code} (attempt {attempt + 1})")

            except requests.Timeout:
                print(f"⚠️ B4M polling timeout (attempt {attempt + 1})")
            except Exception as e:
                print(f"⚠️ B4M polling error (attempt {attempt + 1}): {e}")

        print("⚠️ B4M API polling timeout after 15 attempts (105 seconds)")
        return None

    def extract_b4m_response(self, data: Dict[str, Any]) -> Optional[str]:
        """Extract AI response from B4M API response using fallback methods (per B4M docs)"""
        # Priority order for response extraction based on B4M API documentation

        # Primary: check replies array (current B4M structure)
        if (data.get('replies') and
            isinstance(data['replies'], list) and
            len(data['replies']) > 0):
            # Join multiple replies with newlines
            replies = [reply for reply in data['replies'] if reply and reply.strip()]
            if replies:
                return '\n'.join(replies).strip()

        # Fallback 1: check single reply field (legacy)
        elif data.get('reply'):
            return data['reply'].strip()

        # Fallback 2: check questMasterReply
        elif data.get('questMasterReply'):
            return data['questMasterReply'].strip()

        # Fallback 3: check Research Mode results
        elif (data.get('researchModeResults') and
              isinstance(data['researchModeResults'], list)):
            results = [r.get('response', '') for r in data['researchModeResults']
                      if r.get('response')]
            if results:
                return '\n\n'.join(results).strip()

        # Fallback 4: check messages array with content field
        elif (data.get('messages') and
              isinstance(data['messages'], list) and
              len(data['messages']) > 0):
            messages = [msg.get('content', '') for msg in data['messages']
                       if msg.get('content')]
            if messages:
                return '\n'.join(messages).strip()

        print("⚠️ B4M response extraction failed - no valid response found in any format")
        print(f"⚠️ Available keys: {list(data.keys())}")
        return None

    def b4m_processing_thread(self):
        """Sequential B4M processing thread - processes one conversation file at a time"""
        print("🤖 B4M Sequential Processing Thread started")

        while not self.shutdown_event.is_set():
            try:
                if not self.b4m_enabled:
                    time.sleep(1)
                    continue

                # Get oldest conversation file
                conversation_file = self.get_oldest_conversation_file()

                if conversation_file:
                    # Process this file and WAIT for completion
                    with self.b4m_processing_flag:
                        print(f"🔄 Processing {conversation_file} sequentially...")
                        self.process_b4m_api(conversation_file)
                else:
                    # No files to process, brief sleep
                    time.sleep(0.5)

            except Exception as e:
                print(f"❌ [B4M Thread] Error: {e}")
                time.sleep(1)

    def count_words(self, text: str) -> int:
        """Count words in text according to spec"""
        if not text or not text.strip():
            return 0
        return len(text.split())

    def remove_consecutive_duplicates(self, new_words: List[str]) -> List[str]:
        """Remove exact consecutive duplicate phrases"""
        if not self.word_buffer:
            return new_words

        # Simple duplicate removal - if new words exactly match end of buffer, skip them
        buffer_text = " ".join(self.word_buffer)
        new_text = " ".join(new_words)

        if buffer_text.endswith(new_text):
            return []  # Skip duplicates

        return new_words

    def update_buffer(self, new_text: str):
        """Update word buffer and create conversation files when full"""
        if not new_text.strip():
            return

        new_words = new_text.split()
        new_words = self.remove_consecutive_duplicates(new_words)

        if not new_words:
            return

        self.word_buffer.extend(new_words)
        current_count = len(self.word_buffer)

        print(f"Buffer: {current_count}/{self.buffer_size} words")

        if current_count >= self.buffer_size:
            # Create conversation file
            conversation_text = " ".join(self.word_buffer[:self.buffer_size])
            conversation_file, _ = self.create_conversation_file(conversation_text)

            # Reset buffer with remaining words
            self.word_buffer = self.word_buffer[self.buffer_size:]

            # Note: B4M processing will be handled sequentially by dedicated thread

    def transcribe_audio(self, audio_data: np.ndarray) -> str:
        """Transcribe audio data using Whisper"""
        try:
            if WHISPER_BACKEND == "faster-whisper":
                segments, _ = self.whisper_model.transcribe(
                    audio_data.astype(np.float32),
                    language="en",
                    temperature=0.0
                )

                text_segments = []
                for segment in segments:
                    text_segments.append(segment.text.strip())

                return " ".join(text_segments).strip()

            else:  # openai-whisper
                result = self.whisper_model.transcribe(
                    audio_data.astype(np.float32),
                    language="en",
                    temperature=0.0
                )
                return result["text"].strip()

        except Exception as e:
            print(f"❌ [Speech Recognition Thread] Transcription error: {e}")
            return "[inaudible]"

    def speech_recognition_thread(self):
        """Main speech recognition thread"""
        print(f"🎙️ Starting speech recognition{'(Interactive Mode)' if self.interactive_mode else ''}...")

        if self.test_mode:
            self.test_mode_loop()
        else:
            self.live_audio_loop()

    def test_mode_loop(self):
        """Test mode loop - process sentences from file"""
        print("🧪 Test Mode: Reading from conversation_test.txt")

        while not self.shutdown_event.is_set():
            if self.test_sentence_index >= len(self.test_sentences):
                self.test_sentence_index = 0  # Cycle back to start

            sentence = self.test_sentences[self.test_sentence_index]
            print(f"🎤 Processing sentence {self.test_sentence_index + 1}/{len(self.test_sentences)}: \"{sentence[:50]}...\"")

            # Update speech timing for interactive mode
            self.last_speech_time = time.time()
            self.silence_triggered = False

            # Process sentence
            self.update_buffer(sentence)

            self.test_sentence_index += 1

            # Wait with interruption checking
            for _ in range(int(self.test_sentence_interval * 10)):
                if self.shutdown_event.is_set():
                    return
                time.sleep(0.1)

            # Simulate silence for interactive mode testing
            if self.interactive_mode:
                time.sleep(0.5)  # Add extra pause for silence detection

    def live_audio_loop(self):
        """Live audio capture and processing loop"""
        chunk_samples = int(self.sample_rate * self.chunk_duration)

        while not self.shutdown_event.is_set():
            try:
                # Record audio in interruptible chunks
                audio_data = np.array([], dtype=np.float32)

                for _ in range(self.chunk_duration):
                    if self.shutdown_event.is_set():
                        return

                    # Record 1-second chunk
                    chunk = sd.rec(
                        self.sample_rate,
                        samplerate=self.sample_rate,
                        channels=1,
                        dtype=np.float32
                    )
                    sd.wait()  # Wait for recording to complete

                    audio_data = np.concatenate([audio_data, chunk.flatten()])

                # Check for voice activity (simple energy-based VAD)
                energy = np.mean(audio_data ** 2)
                if energy > 0.001:  # Voice activity threshold
                    self.last_speech_time = time.time()
                    self.silence_triggered = False

                    # Transcribe audio
                    transcription = self.transcribe_audio(audio_data)
                    if transcription and transcription != "[inaudible]":
                        self.update_buffer(transcription)

            except Exception as e:
                print(f"❌ [Speech Recognition Thread] Audio processing error: {e}")
                time.sleep(1)  # Brief pause before retry

    def tts_thread_main(self):
        """Main TTS thread for handling voice responses"""
        print("🔊 TTS Thread started")

        while not self.shutdown_event.is_set():
            try:
                # Check for trigger conditions
                if self.check_trigger_condition():
                    self.handle_voice_response()

                # Brief sleep to prevent busy waiting
                time.sleep(0.1)

            except Exception as e:
                print(f"❌ [TTS Thread] Error: {e}")
                time.sleep(1)

    def check_trigger_condition(self) -> bool:
        """Check if voice response should be triggered"""
        if self.interactive_mode:
            return self.check_silence_trigger()
        else:
            return self.check_keyword_trigger()

    def check_keyword_trigger(self) -> bool:
        """Check for keyword trigger in recent buffer content"""
        if len(self.word_buffer) == 0:
            return False

        # Check last few words for trigger
        recent_words = " ".join(self.word_buffer[-10:]).lower()
        return self.trigger_word in recent_words

    def check_silence_trigger(self) -> bool:
        """Check for silence-based trigger in interactive mode"""
        current_time = time.time()
        silence_duration = current_time - self.last_speech_time

        if silence_duration >= 0.5 and not self.silence_triggered:
            # Show timer after 0.5 seconds of silence
            if silence_duration < self.silence_timeout:
                print(f"⏳ Silence timer: {silence_duration:.1f}s / {self.silence_timeout:.1f}s", end="\r")
                return False
            else:
                print(f"⏳ Silence timer: {self.silence_timeout:.1f}s / {self.silence_timeout:.1f}s")
                print("🤫 3 seconds of silence detected - speaking AI response")
                self.silence_triggered = True
                return True

        return False

    def handle_voice_response(self):
        """Handle voice response when triggered"""
        with self.response_queue_lock:
            response_file = self.get_oldest_response_file()

            if response_file:
                try:
                    # Read response content
                    with open(response_file, "r", encoding="utf-8") as f:
                        response_text = f.read().strip()

                    if response_text:
                        if not self.interactive_mode:
                            print("🎯 Trigger word 'Rosie' detected - speaking AI response")

                        print(f"🔊 Speaking AI response from {response_file}")

                        # Clear interrupt flag before speaking
                        self.tts_interrupt_flag.clear()

                        # Speak the response
                        self.speak_text(response_text)

                        # Clean up response file if not interrupted
                        if not self.tts_interrupt_flag.is_set():
                            os.remove(response_file)
                            print(f"💾 Cleared {response_file} after speaking")
                        else:
                            os.remove(response_file)
                            print(f"💾 Cleared {response_file} due to voice detection")

                except Exception as e:
                    print(f"❌ [TTS Thread] Error processing response file: {e}")

            else:
                # No response file available
                if self.interactive_mode:
                    print("🤫 3 seconds of silence detected")  # Silent mode
                else:
                    print("ℹ️ response.txt not found (no AI response to speak)")

    def run(self):
        """Main application entry point"""
        print("🎤 HA_converse - Speech-to-Text Application Starting...")

        # Cleanup old files
        self.cleanup_files()

        # Load Whisper model
        if not self.load_whisper_model():
            return False

        # Load test sentences if in test mode
        if self.test_mode and not self.load_test_sentences():
            return False

        # Initialize Piper TTS
        if not self.initialize_piper():
            return False

        # Start speech recognition thread
        self.speech_thread = threading.Thread(
            target=self.speech_recognition_thread,
            daemon=True
        )
        self.speech_thread.start()

        # Start TTS thread
        self.tts_thread = threading.Thread(
            target=self.tts_thread_main,
            daemon=True
        )
        self.tts_thread.start()

        # Start B4M sequential processing thread
        if self.b4m_enabled:
            self.b4m_thread = threading.Thread(
                target=self.b4m_processing_thread,
                daemon=True
            )
            self.b4m_thread.start()

        # Main loop - wait for shutdown
        try:
            while not self.shutdown_event.is_set():
                time.sleep(0.1)
        except KeyboardInterrupt:
            print("\n🛑 Keyboard interrupt received")

        # Graceful shutdown
        print("🔄 Shutting down gracefully...")

        # Signal all threads to stop
        self.shutdown_event.set()
        self.tts_interrupt_flag.set()

        # Wait for threads to finish
        if self.speech_thread and self.speech_thread.is_alive():
            self.speech_thread.join(timeout=2)

        if self.tts_thread and self.tts_thread.is_alive():
            self.tts_thread.join(timeout=2)

        if self.b4m_thread and self.b4m_thread.is_alive():
            self.b4m_thread.join(timeout=2)

        # Save current buffer if not empty
        if self.word_buffer:
            buffer_text = " ".join(self.word_buffer)
            _, _ = self.create_conversation_file(buffer_text)
            print(f"💾 Saved current buffer ({len(self.word_buffer)} words)")

        # Final cleanup
        self.cleanup_files()

        print(f"👋 HA_converse shutdown complete. Final word count: {len(self.word_buffer)}")
        return True


def main():
    """Main entry point"""
    parser = argparse.ArgumentParser(
        description="HA_converse - Speech-to-Text Application with B4M AI Integration"
    )
    parser.add_argument(
        "--interactive",
        action="store_true",
        help="Use silence-based triggering instead of keyword detection"
    )
    parser.add_argument(
        "--test",
        action="store_true",
        help="Use test mode with conversation_test.txt instead of microphone"
    )

    args = parser.parse_args()

    # Check required environment variables
    if not os.getenv('B4M_API_KEY'):
        print("⚠️ Warning: B4M_API_KEY environment variable not set")
        print("B4M API integration will be disabled")

    if not os.getenv('B4M_ROSIE_ID'):
        print("⚠️ Warning: B4M_ROSIE_ID environment variable not set")
        print("B4M API integration will be disabled")

    # Create and run application
    app = HAConverseApp(
        interactive_mode=args.interactive,
        test_mode=args.test
    )

    success = app.run()
    sys.exit(0 if success else 1)


if __name__ == "__main__":
    main()