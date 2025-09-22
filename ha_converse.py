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
import wave
import tempfile
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

# Optional Piper TTS support
try:
    from piper.voice import PiperVoice
    PIPER_AVAILABLE = True
except ImportError:
    PIPER_AVAILABLE = False


class WhisperSTT:
    """Whisper-based speech-to-text with rolling buffer management"""

    def __init__(self, model_name: str = "base", buffer_size: int = 20, trigger_word: str = "rosie",
                 b4m_enabled: bool = False, piper_enabled: bool = False, interactive_mode: bool = False):
        self.model_name = model_name
        self.buffer_size = buffer_size
        self.trigger_word = trigger_word.lower()  # Case-insensitive
        self.word_buffer: List[str] = []
        self.last_write_time = time.time()
        self.last_word_count = 0
        self.running = False
        self.audio_queue = queue.Queue()

        # B4M API integration
        self.b4m_enabled = b4m_enabled
        self.debug_mode = False  # Can be enabled for troubleshooting
        if self.b4m_enabled:
            self._init_b4m()

        # Piper TTS integration
        self.piper_enabled = piper_enabled
        self.piper_voice = None
        if self.piper_enabled:
            self._init_piper()

        # Interactive mode settings
        self.interactive_mode = interactive_mode
        self.silence_timeout = 3.0  # 3 seconds of silence triggers response
        self.silence_start_time = None
        self.last_speech_time = time.time()
        self.silence_triggered = False  # Prevent repeated triggers

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

    def _init_piper(self):
        """Initialize Piper TTS configuration"""
        if not PIPER_AVAILABLE:
            print("\n⚠️  WARNING: piper-tts is not installed!")
            print("   Piper TTS disabled. Install with: pip install piper-tts")
            self.piper_enabled = False
            return

        # Get Piper model path from environment or use default
        self.piper_model_path = os.environ.get('PIPER_MODEL_PATH')
        self.piper_config_path = os.environ.get('PIPER_CONFIG_PATH')

        if not self.piper_model_path:
            print("\n⚠️  WARNING: PIPER_MODEL_PATH environment variable not set!")
            print("   Using default Piper voice (if available)")
            print("   Set custom model: export PIPER_MODEL_PATH='/path/to/model.onnx'")
            # Try to use a default model if available
            self.piper_model_path = None
        else:
            # Verify model file exists
            if not os.path.exists(self.piper_model_path):
                print(f"\n⚠️  WARNING: Piper model not found at {self.piper_model_path}")
                print("   Piper TTS disabled. Check PIPER_MODEL_PATH")
                self.piper_enabled = False
                return

            # Verify config file exists if specified
            if self.piper_config_path and not os.path.exists(self.piper_config_path):
                print(f"\n⚠️  WARNING: Piper config not found at {self.piper_config_path}")
                print("   Piper TTS disabled. Check PIPER_CONFIG_PATH")
                self.piper_enabled = False
                return

        try:
            # Load Piper voice model
            if self.piper_model_path:
                print(f"🔊 Loading Piper voice model: {os.path.basename(self.piper_model_path)}")
                if self.piper_config_path:
                    self.piper_voice = PiperVoice.load(self.piper_model_path, self.piper_config_path)
                else:
                    self.piper_voice = PiperVoice.load(self.piper_model_path)
            else:
                # Try to use default model (this might fail, which is fine)
                print("🔊 Attempting to load default Piper voice...")
                # We'll defer loading until first use
                self.piper_voice = None

            print("✅ Piper TTS integration enabled")

        except Exception as e:
            print(f"\n⚠️  WARNING: Failed to load Piper voice: {str(e)}")
            print("   Piper TTS disabled")
            self.piper_enabled = False

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
        if not self.word_buffer:
            print("No content to archive")
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

        print("Ready for new conversation - listening for 'Rosie' again")

        return True

    def _send_buffer_to_b4m(self):
        """Send current buffer to B4M API and save response to response.txt"""
        if not self.b4m_enabled or not self.word_buffer:
            return

        # Get current buffer content
        buffer_content = " ".join(self.word_buffer)

        # Send to B4M API
        try:
            response_text = self._send_to_b4m_and_get_response(buffer_content)
            if response_text:
                # Save response to response.txt
                response_file = Path("response.txt")
                response_file.write_text(response_text, encoding='utf-8')
                print(f"💾 AI response saved to response.txt")
            else:
                print("⚠️  No response received from B4M API")
        except Exception as e:
            print(f"❌ B4M API error: {str(e)}")

    def _send_to_b4m_and_get_response(self, text: str) -> str:
        """Send text to B4M API and return the response text"""
        if not self.b4m_api_key:
            print("❌ B4M_API_KEY environment variable not set")
            return ""

        try:
            print("🤖 Sending to B4M AI service...")
            start_time = time.time()

            # Send initial request
            response = self._make_b4m_request(text)
            if not response:
                return ""

            # Poll for completion
            final_response = self._poll_b4m_response(response, self._get_b4m_headers())

            # Extract AI response
            ai_response = self._extract_ai_response(final_response)

            if ai_response:
                elapsed_time = time.time() - start_time
                print(f"✅ B4M response received in {elapsed_time:.2f} seconds")
                return ai_response
            else:
                print("❌ Failed to extract AI response from B4M")
                return ""

        except Exception as e:
            print(f"❌ B4M API error: {str(e)}")
            return ""

    def _speak_response_file(self):
        """Read and speak the contents of response.txt using Piper TTS"""
        if not self.piper_enabled:
            return  # Silently return if Piper not enabled

        response_file = Path("response.txt")

        try:
            if response_file.exists() and response_file.stat().st_size > 0:
                response_text = response_file.read_text(encoding='utf-8').strip()
                if response_text:
                    print("🔊 Speaking AI response from response.txt")
                    self._speak_text(response_text)
                    # Clear response.txt after speaking to prevent repeat
                    response_file.write_text("", encoding='utf-8')
                    print("💾 Cleared response.txt after speaking")
                else:
                    # response.txt is empty - don't speak anything
                    if not self.interactive_mode:
                        print("ℹ️  response.txt is empty (no AI response to speak)")
            else:
                # response.txt doesn't exist - don't speak anything
                if not self.interactive_mode:
                    print("ℹ️  response.txt not found (no AI response to speak)")
        except Exception as e:
            print(f"❌ Error reading response.txt: {str(e)}")

    def _get_b4m_headers(self):
        """Get headers for B4M API requests"""
        return {
            "X-API-Key": self.b4m_api_key,
            "Content-Type": "application/json"
        }

    def _make_b4m_request(self, text: str):
        """Make initial B4M API request and return response"""
        headers = self._get_b4m_headers()

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
            response = requests.post(
                self.b4m_api_url,
                headers=headers,
                json=payload,
                timeout=10.0
            )

            if response.status_code == 200:
                return response.json()
            else:
                print(f"❌ B4M API Error: Status {response.status_code}")
                return None

        except Exception as e:
            print(f"❌ B4M request error: {str(e)}")
            return None

    def _send_to_b4m(self, text: str):
        """Send archived text to B4M API and display response"""
        if not self.b4m_api_key:
            return

        print("\n🤖 Sending to B4M AI service...")

        try:
            start_time = time.time()

            # Send initial request
            result = self._make_b4m_request(text)
            if not result:
                return

            elapsed_time = time.time() - start_time
            print(f"✅ B4M response received in {elapsed_time:.2f} seconds")

            # Check if we need to poll for the result
            if (result.get('status') == 'running' or
                (result.get('replies') is not None and len(result.get('replies', [])) == 0)):
                print("   Processing... polling for result")
                result = self._poll_b4m_response(result, self._get_b4m_headers())

            # Extract and display the response
            self._display_b4m_response(result)

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

        # Speak the response if Piper is enabled
        if self.piper_enabled and response_text:
            self._speak_text(response_text)

    def _speak_text(self, text: str):
        """Convert text to speech using Piper TTS"""
        if not self.piper_enabled or not text:
            return

        try:
            # Load voice model if not already loaded
            if not self.piper_voice:
                if self.piper_model_path:
                    if self.piper_config_path:
                        self.piper_voice = PiperVoice.load(self.piper_model_path, self.piper_config_path)
                    else:
                        self.piper_voice = PiperVoice.load(self.piper_model_path)
                else:
                    print("🔊 No Piper voice model available")
                    return

            print("🔊 Converting text to speech...")

            # Create temporary WAV file for audio
            with tempfile.NamedTemporaryFile(suffix='.wav', delete=False) as temp_file:
                temp_path = temp_file.name

            # Generate speech using correct Piper API
            audio_chunks = list(self.piper_voice.synthesize(text))

            # Save to temporary WAV file
            with wave.open(temp_path, 'wb') as wav_file:
                # Configure WAV file based on first chunk
                if audio_chunks:
                    first_chunk = audio_chunks[0]
                    wav_file.setnchannels(first_chunk.sample_channels)
                    wav_file.setsampwidth(first_chunk.sample_width)
                    wav_file.setframerate(first_chunk.sample_rate)

                    # Write all audio chunks
                    for chunk in audio_chunks:
                        wav_file.writeframes(chunk.audio_int16_bytes)

            # Play the audio file
            self._play_audio_file(temp_path)

            # Clean up temporary file
            try:
                os.unlink(temp_path)
            except:
                pass  # Ignore cleanup errors

        except Exception as e:
            print(f"🔊 TTS Error: {str(e)}")
            print("   Continuing without speech synthesis")

    def _play_audio_file(self, file_path: str):
        """Play audio file using sounddevice"""
        try:
            # Read WAV file
            with wave.open(file_path, 'rb') as wav_file:
                frames = wav_file.readframes(wav_file.getnframes())
                sample_rate = wav_file.getframerate()
                channels = wav_file.getnchannels()
                sample_width = wav_file.getsampwidth()

            # Convert to numpy array
            if sample_width == 2:  # 16-bit
                audio_data = np.frombuffer(frames, dtype=np.int16)
            else:
                print(f"🔊 Unsupported audio format: {sample_width} bytes")
                return

            # Reshape for channels
            if channels == 2:
                audio_data = audio_data.reshape(-1, 2)

            # Convert to float32 for sounddevice
            audio_data = audio_data.astype(np.float32) / 32768.0

            # Play audio
            sd.play(audio_data, sample_rate)
            sd.wait()  # Wait until playback is finished

        except Exception as e:
            print(f"🔊 Audio playback error: {str(e)}")

    def _update_conversation_file(self):
        """Write buffer to conversation file"""
        content = " ".join(self.word_buffer)
        self.conversation_file.write_text(content)

    def _should_write_to_file(self) -> bool:
        """Determine if we should write to file based on time or word count"""
        time_elapsed = time.time() - self.last_write_time >= 10.0
        words_added = len(self.word_buffer) - self.last_word_count >= 10
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
        # In interactive mode, we don't use keyword trigger
        if self.interactive_mode:
            return False
        return self.trigger_word in text.lower()

    def _check_silence_trigger(self) -> bool:
        """Check if silence duration has exceeded threshold in interactive mode"""
        if not self.interactive_mode:
            return False

        current_time = time.time()
        silence_duration = current_time - self.last_speech_time

        # If we've already triggered, don't show timer or check again until speech resets it
        if self.silence_triggered:
            return False

        # Show silence timer if we have some silence
        if silence_duration >= 0.5:  # Only show after 0.5 seconds
            print(f"\r⏳ Silence timer: {silence_duration:.1f}s / {self.silence_timeout}s", end="", flush=True)

        # Check if we've reached the silence threshold
        if silence_duration >= self.silence_timeout:
            if not self.silence_triggered:
                # Check if we have a response to speak before showing the message
                response_file = Path("response.txt")
                if response_file.exists() and response_file.stat().st_size > 0:
                    print(f"\n🤫 {self.silence_timeout} seconds of silence detected - speaking AI response")
                else:
                    print(f"\n🤫 {self.silence_timeout} seconds of silence detected")
                self.silence_triggered = True
                # Reset the timer after triggering
                self.last_speech_time = current_time
                return True

        return False

    def _process_transcription(self, text: str):
        """Process transcribed text and update buffer"""
        if not text or text.strip() == "":
            # In interactive mode, check for silence trigger during empty transcriptions
            if self.interactive_mode:
                if self._check_silence_trigger():
                    self._speak_response_file()
                    # Don't immediately reset the flag - wait for speech to reset it
            return

        # We have speech - reset silence tracking in interactive mode
        if self.interactive_mode:
            self.last_speech_time = time.time()
            self.silence_triggered = False
            # Clear the silence timer line
            print("\r" + " " * 50 + "\r", end="", flush=True)

        # Check for trigger word - speak response.txt (only in non-interactive mode)
        if self._check_for_trigger(text):
            print(f"\n🎯 Trigger word 'Rosie' detected - speaking AI response")
            self._speak_response_file()
            # Continue processing the text normally (don't return early)

        # Split into words and clean
        words = text.strip().split()
        words = self._remove_consecutive_duplicates(words)

        if not words:
            return

        # Add to buffer and check if we need to send to B4M
        old_buffer_size = len(self.word_buffer)

        if len(self.word_buffer) + len(words) > self.buffer_size:
            # Buffer will overflow - send to B4M before rollover
            if old_buffer_size >= self.buffer_size and self.b4m_enabled:
                print(f"\n🤖 Buffer full ({self.buffer_size} words) - sending to B4M AI...")
                self._send_buffer_to_b4m()

            # Circular buffer - keep only last buffer_size words
            self.word_buffer.extend(words)
            self.word_buffer = self.word_buffer[-self.buffer_size:]
            # Show rollover indicator
            words_dropped = (old_buffer_size + len(words)) - self.buffer_size
            if words_dropped > 0:
                print(f"↻ Buffer rollover: {words_dropped} oldest word(s) dropped")
        else:
            # Normal addition
            self.word_buffer.extend(words)

            # Check if we just reached buffer_size
            if len(self.word_buffer) == self.buffer_size and self.b4m_enabled:
                print(f"\n🤖 Buffer full ({self.buffer_size} words) - sending to B4M AI...")
                self._send_buffer_to_b4m()

        # Update display
        word_count = len(self.word_buffer)
        print(f"\rBuffer: {word_count}/{self.buffer_size} words", end="", flush=True)

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
                        else:
                            # Even with no text, check for silence in interactive mode
                            self._process_transcription("")

                    except Exception as e:
                        print(f"\nTranscription error: {e}")
                        self._process_transcription("[inaudible]")

            except queue.Empty:
                # In interactive mode, still check for silence trigger
                if self.interactive_mode:
                    self._process_transcription("")
                continue
            except Exception as e:
                print(f"\nWorker error: {e}")

    def start(self):
        """Start the speech-to-text system"""
        print("Starting HA_converse Speech-to-Text System")
        print(f"Using Whisper {self.model_name} model")
        if self.interactive_mode:
            print(f"Interactive mode: {self.silence_timeout}s silence triggers voice response")
        else:
            print(f"Trigger word: 'Rosie' (say it to speak AI response)")
        print(f"Buffer size: {self.buffer_size} words")
        if self.b4m_enabled:
            print(f"B4M API: ENABLED - responses will be sent to AI")
        if self.piper_enabled:
            print(f"Piper TTS: ENABLED - AI responses will be spoken")
            # Test Piper TTS on startup
            print("🔊 Testing Piper TTS...")
            self._speak_text("Hello World! Piper text-to-speech is working correctly.")
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
  %(prog)s                         # Basic mode with "Rosie" trigger
  %(prog)s --b4m                  # With B4M AI integration
  %(prog)s --b4m --piper          # With voice output
  %(prog)s --b4m --piper --interactive  # Interactive mode (3s silence trigger)

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
        '--piper',
        action='store_true',
        help='Enable Piper TTS - speaks B4M AI responses aloud (requires --b4m)'
    )

    parser.add_argument(
        '--interactive',
        action='store_true',
        help='Enable interactive mode - speaks response after 3 seconds of silence (replaces "Rosie" trigger)'
    )

    parser.add_argument(
        '--buffer-size',
        type=int,
        default=20,
        help='Buffer size in words (default: 20)'
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

    # Check for piper-tts if Piper is enabled
    if args.piper:
        if not args.b4m:
            print("Error: --piper requires --b4m to be enabled")
            print("Piper TTS is used to speak B4M AI responses")
            sys.exit(1)

        if not PIPER_AVAILABLE:
            print("Missing dependency for Piper TTS: piper-tts")
            print("Please install: pip install piper-tts")
            sys.exit(1)

    # Start the system
    stt = WhisperSTT(
        model_name=args.model,
        buffer_size=args.buffer_size,
        trigger_word="rosie",
        b4m_enabled=args.b4m,
        piper_enabled=args.piper,
        interactive_mode=args.interactive
    )

    try:
        stt.start()
    except Exception as e:
        print(f"Fatal error: {e}")
        sys.exit(1)


if __name__ == "__main__":
    main()