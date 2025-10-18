#!/usr/bin/env python3
"""
ROSIE Conversational AI System
Implements CONVERSE_B4M_OLLAMA_HYBRID specification

A voice-controlled conversational AI that combines:
- Local Ollama for immediate responses (<1 second)
- bike4mind API for background intelligence (5-10 seconds)
- Whisper for speech-to-text
- Piper for text-to-speech

Architecture: State machine with three states (LISTENING, RESPONDING, SPEAKING)
Wake word: "Rosie"
"""

import os
import sys
import signal
import threading
import time
import json
import re
from enum import Enum
from pathlib import Path
import requests
import whisper
import numpy as np
import sounddevice as sd
from dotenv import load_dotenv

# Load environment configuration (environment variables take precedence over .env file)
load_dotenv('/home/mike/projects/b4m_yahboom/.env.rosie.example', override=False)


class ConversationState(Enum):
    """State machine states"""
    LISTENING = 1
    RESPONDING = 2
    SPEAKING = 3


class RosieConversation:
    """
    Main ROSIE Conversational AI System

    Implements a state machine architecture with asynchronous background intelligence.
    """

    def __init__(self):
        """Initialize ROSIE system with configuration from environment"""

        # Load configuration from environment
        self.whisper_model_name = os.getenv('WHISPER_MODEL', 'base')
        self.whisper_chunk_duration = int(os.getenv('WHISPER_CHUNK_DURATION', '3'))

        self.ollama_model = os.getenv('OLLAMA_MODEL', 'qwen2.5:0.5b')
        self.ollama_temperature = float(os.getenv('OLLAMA_TEMPERATURE', '0.7'))
        self.ollama_max_tokens = int(os.getenv('OLLAMA_MAX_TOKENS', '100'))
        self.ollama_url = 'http://localhost:11434/api/generate'

        # bike4mind API configuration (from .bashrc)
        self.b4m_api_key = os.getenv('B4M_API_KEY')
        self.b4m_conversation_id = os.getenv('B4M_OLLAMA_CONVERSATION_ID')
        self.b4m_user_id = os.getenv('B4M_USER_ID', '65563f622213b120cd1d9592')
        self.b4m_url = 'https://www.bike4mind.com/api/v1/conversation'

        # Piper TTS configuration (from .bashrc)
        self.piper_model_path = os.getenv('PIPER_MODEL_PATH')
        self.piper_config_path = os.getenv('PIPER_CONFIG_PATH')

        # File paths
        self.listen_file = Path(os.getenv('LISTEN_FILE', '/tmp/listen.txt'))
        self.summary_file = Path(os.getenv('SUMMARY_FILE', '/tmp/summary.txt'))
        self.speak_file = Path(os.getenv('SPEAK_FILE', '/tmp/speak.txt'))

        # Conversation settings
        self.max_words = int(os.getenv('MAX_WORDS', '100'))
        self.debug = int(os.getenv('DEBUG', '0')) == 1

        # State machine
        self.state = ConversationState.LISTENING
        self.state_lock = threading.Lock()

        # Conversation activation flag (for edge detection)
        self.conversation_active = False
        self.conversation_lock = threading.Lock()

        # Shutdown event for graceful termination
        self.shutdown_event = threading.Event()

        # Threads
        self.whisper_thread = None
        self.wake_word_thread = None
        self.b4m_worker_thread = None

        # Audio configuration for continuous streaming
        self.sample_rate = 16000

        # Continuous audio buffer - collects audio while Whisper processes
        self.audio_queue = []  # List of numpy arrays (chunks)
        self.audio_lock = threading.Lock()
        self.audio_stream = None  # sounddevice InputStream

        # Whisper model (loaded lazily)
        self.whisper_model = None

        # Initialize files
        self._initialize_files()

        # Validate configuration
        self._validate_configuration()

        self._log("ROSIE Conversational AI System initialized")

    def _log(self, message):
        """Log messages with timestamp"""
        if self.debug:
            print(f"[{time.strftime('%H:%M:%S')}] {message}")

    def _initialize_files(self):
        """Initialize conversation files - clear all temp files at startup"""
        # Clear all conversation files for fresh start
        for file_path in [self.listen_file, self.summary_file, self.speak_file]:
            file_path.write_text('')

        print("Conversation files cleared and ready for new session.")

    def _validate_configuration(self):
        """Validate required configuration"""
        if not self.b4m_api_key:
            print("WARNING: B4M_API_KEY not set in environment. bike4mind features will be disabled.")

        if not self.b4m_conversation_id:
            print("WARNING: B4M_OLLAMA_CONVERSATION_ID not set in environment. bike4mind features will be disabled.")

        if not self.piper_model_path or not Path(self.piper_model_path).exists():
            print(f"ERROR: PIPER_MODEL_PATH not found: {self.piper_model_path}")
            print("Please set PIPER_MODEL_PATH in ~/.bashrc")
            sys.exit(1)

    def _get_state(self):
        """Thread-safe state getter"""
        with self.state_lock:
            return self.state

    def _set_state(self, new_state):
        """Thread-safe state setter"""
        with self.state_lock:
            old_state = self.state
            self.state = new_state
            # Always print state transitions to console
            print(f"\n[STATE] {old_state.name} → {new_state.name}")
            self._log(f"State transition: {old_state.name} -> {new_state.name}")

    def _get_conversation_active(self):
        """Thread-safe conversation_active getter"""
        with self.conversation_lock:
            return self.conversation_active

    def _set_conversation_active(self, value):
        """Thread-safe conversation_active setter with edge detection"""
        with self.conversation_lock:
            old_value = self.conversation_active
            self.conversation_active = value
            edge_detected = (old_value != value)
            self._log(f"Conversation active: {old_value} -> {value} (edge: {edge_detected})")
            return edge_detected, old_value, value

    # =====================================================================
    # STATE 1: LISTENING - Whisper STT and Wake Word Detection
    # =====================================================================

    def _load_whisper_model(self):
        """Load Whisper model (lazy loading)"""
        if self.whisper_model is None:
            self._log(f"Loading Whisper model: {self.whisper_model_name}")
            self.whisper_model = whisper.load_model(self.whisper_model_name)
            self._log("Whisper model loaded")

    def _audio_callback(self, indata, frames, time_info, status):
        """
        Audio input callback - continuously captures audio

        This runs in a separate thread managed by sounddevice.
        Captures ALL audio without gaps.
        """
        if status:
            self._log(f"Audio callback status: {status}")

        # Only capture during LISTENING state
        if self._get_state() == ConversationState.LISTENING:
            # Append audio to queue (thread-safe)
            with self.audio_lock:
                self.audio_queue.append(indata.copy().flatten())

    def _whisper_worker(self):
        """
        Whisper speech-to-text worker thread

        Processes audio from continuous stream and appends to listen.txt.
        Audio capture continues in background via callback - NO GAPS.
        """
        self._log("Whisper worker started")
        self._load_whisper_model()

        # Start continuous audio stream
        self._start_audio_stream()

        # Process audio buffer every 2 seconds
        process_interval = 2.0  # seconds

        while not self.shutdown_event.is_set():
            current_state = self._get_state()

            # Only process in LISTENING state
            if current_state == ConversationState.LISTENING:
                try:
                    # Wait for audio to accumulate
                    time.sleep(process_interval)

                    # Get accumulated audio from queue
                    with self.audio_lock:
                        if len(self.audio_queue) == 0:
                            continue

                        # Concatenate all queued audio chunks
                        audio_data = np.concatenate(self.audio_queue)

                        # Clear queue (we've grabbed it all)
                        self.audio_queue.clear()

                    # Check if there's actual speech
                    audio_level = np.abs(audio_data).max()
                    if audio_level < 0.01:
                        continue  # Skip silence

                    # Transcribe with Whisper
                    result = self.whisper_model.transcribe(
                        audio_data,
                        fp16=False,
                        language='en'
                    )

                    transcription = result['text'].strip()

                    if transcription:
                        # Append to listen.txt with "Human said:" prefix
                        self._append_to_listen_file(f"Human said: {transcription}")
                        # Always print transcriptions to console
                        print(f"[WHISPER] Human said: {transcription}")
                        self._log(f"Transcribed: {transcription}")

                except Exception as e:
                    self._log(f"Whisper error: {e}")

            else:
                # Clear queue during RESPONDING/SPEAKING (don't transcribe robot speech)
                with self.audio_lock:
                    self.audio_queue.clear()
                time.sleep(0.1)

        # Stop audio stream
        self._stop_audio_stream()
        self._log("Whisper worker stopped")

    def _start_audio_stream(self):
        """Start continuous audio capture stream"""
        if self.audio_stream is None:
            self._log("Starting continuous audio stream")
            self.audio_stream = sd.InputStream(
                samplerate=self.sample_rate,
                channels=1,
                dtype='float32',
                callback=self._audio_callback,
                blocksize=int(self.sample_rate * 0.1)  # 100ms blocks
            )
            self.audio_stream.start()
            self._log("Audio stream started")

    def _stop_audio_stream(self):
        """Stop continuous audio capture stream"""
        if self.audio_stream is not None:
            self._log("Stopping audio stream")
            self.audio_stream.stop()
            self.audio_stream.close()
            self.audio_stream = None
            self._log("Audio stream stopped")

    def _append_to_listen_file(self, text):
        """Append text to listen.txt (thread-safe)"""
        try:
            with open(self.listen_file, 'a') as f:
                f.write(text + ' ')
        except Exception as e:
            self._log(f"Error writing to listen.txt: {e}")

    def _wake_word_detector(self):
        """
        Wake word detection worker thread

        Monitors listen.txt for "Rosie" and activates conversation mode.
        Active only during LISTENING state.
        """
        self._log("Wake word detector started")

        while not self.shutdown_event.is_set():
            current_state = self._get_state()

            # Only active in LISTENING state
            if current_state == ConversationState.LISTENING:
                try:
                    # Read listen.txt
                    content = self.listen_file.read_text()

                    # Check if "Rosie" appears (case-insensitive)
                    if re.search(r'\brosie\b', content, re.IGNORECASE):
                        # Always print wake word detection to console
                        print(f"\n[WAKE WORD] 'Rosie' detected! Activating conversation...")
                        self._log("Wake word 'Rosie' detected!")

                        # Activate conversation (edge detection)
                        edge_detected, _, _ = self._set_conversation_active(True)

                        # Transition to RESPONDING state
                        self._set_state(ConversationState.RESPONDING)

                        # Remove "Rosie" from listen.txt
                        cleaned_content = re.sub(r'\brosie\b', '', content, flags=re.IGNORECASE)
                        self.listen_file.write_text(cleaned_content)

                        # Trigger Ollama response (in new thread to avoid blocking)
                        threading.Thread(target=self._ollama_response, daemon=True).start()

                except Exception as e:
                    self._log(f"Wake word detection error: {e}")

            # Check every 100ms
            time.sleep(0.1)

        self._log("Wake word detector stopped")

    # =====================================================================
    # STATE 2: RESPONDING - Ollama Immediate Response
    # =====================================================================

    def _ollama_response(self):
        """
        Ollama immediate response generation

        Generates response in <1 second using local Ollama.
        Primary goal: Keep the human engaged and talking.
        """
        self._log("Ollama processing started")

        try:
            # Read listen.txt (conversation history)
            listen_content = self.listen_file.read_text()

            # Read summary.txt if it exists (may be stale)
            summary_content = ""
            if self.summary_file.exists():
                summary_content = self.summary_file.read_text()

            # Combine context
            context = f"Conversation history:\n{listen_content}\n\n"
            if summary_content:
                context += f"Intelligence summary:\n{summary_content}\n\n"

            # Ollama prompt focused on engagement
            prompt = (
                f"{context}"
                "Please respond to this conversation. Your primary goal is to keep the human "
                "engaged and talking. Ask follow-up questions, express curiosity, and maintain "
                "natural dialogue. If you don't have complete information, acknowledge what was "
                "asked and encourage them to tell you more about it. Keep the conversation "
                "flowingbike4mind will provide deeper insights soon."
            )

            # Call Ollama API
            response = requests.post(
                self.ollama_url,
                json={
                    'model': self.ollama_model,
                    'prompt': prompt,
                    'temperature': self.ollama_temperature,
                    'max_tokens': self.ollama_max_tokens,
                    'stream': False
                },
                timeout=5
            )

            if response.status_code == 200:
                result = response.json()
                ollama_text = result.get('response', '').strip()

                if ollama_text:
                    # Write to speak.txt
                    self.speak_file.write_text(ollama_text)
                    # Always print Ollama response to console
                    print(f"[OLLAMA] Robot will say: {ollama_text}")
                    self._log(f"Ollama response: {ollama_text}")

                    # Transition to SPEAKING state
                    self._set_state(ConversationState.SPEAKING)

                    # Trigger speech output
                    threading.Thread(target=self._piper_speak, daemon=True).start()
                else:
                    self._log("Ollama returned empty response")
                    self._set_state(ConversationState.LISTENING)
            else:
                self._log(f"Ollama API error: {response.status_code}")
                self._set_state(ConversationState.LISTENING)

        except Exception as e:
            self._log(f"Ollama error: {e}")
            self._set_state(ConversationState.LISTENING)

    # =====================================================================
    # BACKGROUND: bike4mind Intelligent Analysis
    # =====================================================================

    def _b4m_worker(self):
        """
        bike4mind background worker with edge detection

        Monitors conversation_active flag for state changes.
        Processes conversation once per activation (False -> True edge).
        Operates independently, never blocks main conversation loop.
        """
        self._log("bike4mind worker started")

        # Check if bike4mind is configured
        if not self.b4m_api_key or not self.b4m_conversation_id:
            self._log("bike4mind not configured, worker disabled")
            return

        last_state = False

        while not self.shutdown_event.is_set():
            current_state = self._get_conversation_active()

            # Edge detection: False -> True transition
            if not last_state and current_state:
                # Always print bike4mind activation to console
                print(f"\n[BIKE4MIND] Background analysis triggered...")
                self._log("bike4mind activation edge detected!")

                # Process conversation (one-shot per activation)
                threading.Thread(target=self._b4m_process, daemon=True).start()

            last_state = current_state

            # Check every 100ms
            time.sleep(0.1)

        self._log("bike4mind worker stopped")

    def _b4m_process(self):
        """
        bike4mind API processing (background, asynchronous)

        Leverages powerful LLM with real-time internet access.
        Updates summary.txt for future Ollama responses.
        """
        self._log("bike4mind processing started")

        try:
            # Read current listen.txt content
            listen_content = self.listen_file.read_text()

            if not listen_content.strip():
                self._log("bike4mind: Empty listen.txt, skipping")
                return

            # Always print what's being sent to bike4mind
            print(f"[BIKE4MIND] Analyzing conversation: {listen_content[:80]}{'...' if len(listen_content) > 80 else ''}")

            # Prepare bike4mind API request
            headers = {
                'X-API-Key': self.b4m_api_key,
                'Content-Type': 'application/json'
            }

            payload = {
                'conversationId': self.b4m_conversation_id,
                'userId': self.b4m_user_id,
                'message': (
                    f"Please summarize this conversation, including intelligent insights:\n\n"
                    f"{listen_content}"
                )
            }

            # Call bike4mind API (5-10 seconds latency expected)
            response = requests.post(
                self.b4m_url,
                headers=headers,
                json=payload,
                timeout=15
            )

            if response.status_code == 200:
                result = response.json()

                # Extract response (API structure may vary)
                b4m_response = result.get('response', result.get('message', ''))

                if b4m_response:
                    # Write enriched summary to summary.txt (atomic overwrite)
                    self.summary_file.write_text(b4m_response)
                    # Always print bike4mind completion to console
                    print(f"[BIKE4MIND] Analysis complete! Summary updated ({len(b4m_response)} chars)")
                    print(f"[BIKE4MIND] Preview: {b4m_response[:100]}{'...' if len(b4m_response) > 100 else ''}")
                    self._log(f"bike4mind summary updated: {len(b4m_response)} chars")
                else:
                    print(f"[BIKE4MIND] Warning: Empty response received")
                    self._log("bike4mind returned empty response")
            else:
                print(f"[BIKE4MIND] Error: API returned status {response.status_code}")
                self._log(f"bike4mind API error: {response.status_code}")

        except Exception as e:
            print(f"[BIKE4MIND] Error: {e}")
            self._log(f"bike4mind error: {e}")

    # =====================================================================
    # STATE 3: SPEAKING - Piper TTS Output
    # =====================================================================

    def _piper_speak(self):
        """
        Piper text-to-speech output

        Converts speak.txt to speech, plays audio, and deactivates conversation.
        """
        self._log("Piper TTS started")

        try:
            # Read speak.txt
            text = self.speak_file.read_text().strip()

            if not text:
                self._log("speak.txt is empty, skipping TTS")
                self._set_state(ConversationState.LISTENING)
                return

            # Generate audio with Piper
            import subprocess

            # Piper command: echo text | piper --model path --output-raw | aplay
            piper_cmd = f'echo "{text}" | piper --model {self.piper_model_path} --output-raw | aplay -r 22050 -f S16_LE -t raw -'

            subprocess.run(piper_cmd, shell=True, check=True)

            self._log(f"Piper spoke: {text}")

            # Append to listen.txt with "Robot said:" prefix
            self._append_to_listen_file(f"Robot said: {text}")

            # Clear speak.txt
            self.speak_file.write_text('')

            # Prune listen.txt if needed
            self._prune_listen_file()

            # Deactivate conversation (edge: True -> False)
            self._set_conversation_active(False)

            # Transition back to LISTENING state
            self._set_state(ConversationState.LISTENING)

        except Exception as e:
            self._log(f"Piper error: {e}")
            self._set_state(ConversationState.LISTENING)

    def _prune_listen_file(self):
        """Prune listen.txt to keep last MAX_WORDS words"""
        try:
            content = self.listen_file.read_text()
            words = content.split()

            if len(words) > self.max_words:
                pruned = ' '.join(words[-self.max_words:])
                self.listen_file.write_text(pruned)
                self._log(f"Pruned listen.txt to {self.max_words} words")

        except Exception as e:
            self._log(f"Pruning error: {e}")

    # =====================================================================
    # System Control
    # =====================================================================

    def start(self):
        """Start ROSIE system (all threads)"""
        self._log("Starting ROSIE Conversational AI System...")

        # Start Whisper worker
        self.whisper_thread = threading.Thread(target=self._whisper_worker, daemon=True)
        self.whisper_thread.start()

        # Start wake word detector
        self.wake_word_thread = threading.Thread(target=self._wake_word_detector, daemon=True)
        self.wake_word_thread.start()

        # Start bike4mind background worker
        self.b4m_worker_thread = threading.Thread(target=self._b4m_worker, daemon=True)
        self.b4m_worker_thread.start()

        # Always print startup message and instructions
        print("\n" + "="*70)
        print("ROSIE Conversational AI System - READY")
        print("="*70)
        print(f"Current State: {self._get_state().name}")
        print("\nSay 'Rosie' followed by your question to start a conversation.")
        print("Press CTRL+C to exit.")
        print("="*70 + "\n")

        self._log("ROSIE system started. Say 'Rosie' to activate conversation.")

        # Main loop (keep alive)
        try:
            while not self.shutdown_event.is_set():
                time.sleep(0.5)
        except KeyboardInterrupt:
            self.shutdown()

    def shutdown(self):
        """Graceful shutdown with SIGINT handling"""
        self._log("Shutting down ROSIE system...")

        # Signal all threads to stop
        self.shutdown_event.set()

        # Wait for threads to finish (with timeout)
        threads = [self.whisper_thread, self.wake_word_thread, self.b4m_worker_thread]
        for thread in threads:
            if thread and thread.is_alive():
                thread.join(timeout=2)

        self._log("ROSIE system stopped")
        sys.exit(0)


def signal_handler(sig, frame):
    """Handle CTRL+C for graceful shutdown"""
    print("\nReceived shutdown signal (CTRL+C)")
    if hasattr(signal_handler, 'rosie_instance'):
        signal_handler.rosie_instance.shutdown()
    else:
        sys.exit(0)


def main():
    """Main entry point"""
    # Register signal handler for graceful shutdown
    signal.signal(signal.SIGINT, signal_handler)

    # Create ROSIE instance
    rosie = RosieConversation()

    # Store instance for signal handler
    signal_handler.rosie_instance = rosie

    # Start system
    rosie.start()


if __name__ == '__main__':
    main()
