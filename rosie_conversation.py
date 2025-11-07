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
import argparse
from enum import Enum
from pathlib import Path
import requests
import whisper
import numpy as np
import sounddevice as sd
from dotenv import load_dotenv

# Load optional configuration from .env file (only if variables not already set)
# Environment variables from .bashrc always take precedence
env_file = Path(__file__).parent / '.env.rosie'
if env_file.exists():
    load_dotenv(env_file, override=False)


class ConversationState(Enum):
    """State machine states"""
    LISTENING = 1
    RESPONDING = 2
    SPEAKING = 3


class RosieConversation:
    """
    Main ROSIE Conversational AI System

    Simplified architecture with plain text conversation history.
    """

    def __init__(self):
        """Initialize ROSIE system with configuration from environment"""

        # Load configuration from environment
        self.whisper_model_name = os.getenv('WHISPER_MODEL', 'base')
        self.whisper_chunk_duration = int(os.getenv('WHISPER_CHUNK_DURATION', '3'))

        self.ollama_model = os.getenv('OLLAMA_MODEL', 'llama3.1:8b')
        self.ollama_temperature = float(os.getenv('OLLAMA_TEMPERATURE', '0.7'))
        self.ollama_max_tokens = int(os.getenv('OLLAMA_MAX_TOKENS', '100'))
        self.ollama_url = 'http://localhost:11434/api/generate'
        self.context_limit = int(os.getenv('CONTEXT_LIMIT', '6000'))  # Token limit before summarization

        # Piper TTS configuration (from .bashrc)
        self.piper_model_path = os.getenv('PIPER_MODEL_PATH')
        self.piper_config_path = os.getenv('PIPER_CONFIG_PATH')

        # File paths (default to script directory)
        script_dir = Path(__file__).parent
        self.history_file = Path(os.getenv('HISTORY_FILE', script_dir / 'conversation_history.txt'))
        self.speak_file = Path(os.getenv('SPEAK_FILE', script_dir / 'speak.txt'))

        # Conversation settings
        self.debug = int(os.getenv('DEBUG', '0')) == 1

        # State machine
        self.state = ConversationState.LISTENING
        self.state_lock = threading.Lock()

        # Shutdown event for graceful termination
        self.shutdown_event = threading.Event()

        # Threads
        self.whisper_thread = None
        self.wake_word_thread = None

        # Audio configuration for continuous streaming
        self.sample_rate = 16000

        # Continuous audio buffer - collects audio while Whisper processes
        self.audio_queue = []  # List of numpy arrays (chunks)
        self.audio_lock = threading.Lock()
        self.audio_stream = None  # sounddevice InputStream

        # Whisper model (loaded lazily)
        self.whisper_model = None

        # Wake word detection flag (set by Whisper when wake word detected)
        self.wake_word_detected = False
        self.wake_word_lock = threading.Lock()

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
        """Initialize conversation files - ensure they exist"""
        # Create files if they don't exist (but don't clear existing content)
        if not self.history_file.exists():
            self.history_file.write_text('')
            print("Created new conversation_history.txt")
        else:
            # Load existing conversation
            existing_content = self.history_file.read_text()
            if existing_content:
                print(f"Loaded existing conversation history ({len(existing_content)} chars)")
            else:
                print("conversation_history.txt exists but is empty")

        # Always clear speak file (temporary buffer)
        self.speak_file.write_text('')

    def _estimate_token_count(self, text):
        """Estimate token count (rough approximation: 1 token ≈ 0.75 words)"""
        words = len(text.split())
        return int(words * 1.3)  # Conservative estimate

    def _validate_configuration(self):
        """Validate required configuration"""
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

    # =====================================================================
    # STATE 1: LISTENING - Whisper STT and Wake Word Detection
    # =====================================================================

    def _is_hallucination(self, text):
        """
        Detect common Whisper hallucinations

        Returns True if text appears to be a hallucination
        """
        hallucination_patterns = [
            r'subscribe',
            r'youtube',
            r'channel',
            r'like.*comment',
            r'bell icon',
            r'thank(s| you) for watching',
            r'幕',  # Chinese characters (common in training data)
            r'字幕',
            r'살아',  # Korean characters
            r'CC',
            r'\[.*?\]',  # Brackets like [Music], [Applause]
            r'♪',  # Music notes
            r'www\.',
            r'\.com',
            r'http',
        ]

        text_lower = text.lower()

        # Check for hallucination patterns
        for pattern in hallucination_patterns:
            if re.search(pattern, text_lower):
                return True

        # Check for very short transcriptions (single punctuation or short nonsense)
        if len(text.strip()) <= 2:
            return True

        # Check for repeated punctuation (often hallucination)
        if re.search(r'[.!?]{3,}', text):
            return True

        # Check for gibberish - high ratio of non-ASCII or mixed scripts
        non_ascii_count = sum(1 for c in text if ord(c) > 127)
        if len(text) > 0 and (non_ascii_count / len(text)) > 0.3:
            return True

        # Check for very low ratio of common English words
        common_words = ['the', 'is', 'are', 'was', 'were', 'a', 'an', 'to', 'of', 'in', 'on', 'for', 'with', 'you', 'i', 'me', 'my', 'your']
        words = text_lower.split()
        if len(words) > 5:  # Only check longer phrases
            common_word_count = sum(1 for word in words if word in common_words)
            if common_word_count == 0:
                return True

        return False

    def _load_whisper_model(self):
        """Load Whisper model (lazy loading) with GPU auto-detection and CPU fallback"""
        if self.whisper_model is None:
            self._log(f"Loading Whisper model: {self.whisper_model_name}")

            try:
                import torch

                # Auto-detect best available device
                device = self._detect_best_device()
                self._log(f"Using device: {device}")

                # Load model with selected device
                self.whisper_model = whisper.load_model(
                    self.whisper_model_name,
                    device=device
                )

                if device == "cuda":
                    print(f"[WHISPER] ✓ Model loaded on GPU (CUDA)")
                else:
                    print(f"[WHISPER] ℹ Model loaded on CPU (GPU not available)")

                self._log(f"Whisper model loaded successfully on {device}")

            except Exception as e:
                print(f"\n[ERROR] Failed to load Whisper model: {e}")
                print(f"[ERROR] Please check Whisper installation and model availability")
                self._log(f"Whisper model loading failed: {e}")
                raise

    def _detect_best_device(self):
        """Detect best available device for Whisper (GPU with fallback to CPU)"""
        try:
            import torch

            # Check if CUDA is available and working
            if torch.cuda.is_available():
                try:
                    # Test if CUDA actually works by creating a small tensor
                    test_tensor = torch.zeros(1).cuda()
                    del test_tensor
                    self._log("CUDA detected and functional")
                    return "cuda"
                except Exception as e:
                    self._log(f"CUDA available but not functional: {e}")
                    print(f"[WHISPER] ⚠ GPU detected but CUDA initialization failed")
                    print(f"[WHISPER] → Falling back to CPU")
                    return "cpu"
            else:
                self._log("CUDA not available, using CPU")
                return "cpu"

        except Exception as e:
            self._log(f"Device detection error: {e}, defaulting to CPU")
            return "cpu"

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

                    # Filter out common Whisper hallucinations
                    if transcription and not self._is_hallucination(transcription):
                        # Check for wake word BEFORE storing
                        wake_word_pattern = r'\b(rosie|rose|rosy|rosee)\b'
                        wake_word_match = re.search(wake_word_pattern, transcription, re.IGNORECASE)

                        if wake_word_match:
                            # Wake word detected! Set flag and remove it from transcription
                            with self.wake_word_lock:
                                self.wake_word_detected = True

                            # Remove wake word from transcription before storing
                            cleaned_transcription = re.sub(wake_word_pattern, '', transcription, flags=re.IGNORECASE).strip()

                            print(f"[WHISPER] Human: {transcription} (wake word detected and removed)")
                            self._log(f"Wake word detected in: {transcription}")

                            # Store cleaned version (without wake word)
                            if cleaned_transcription:  # Only store if there's content left
                                self._append_to_history(f"Human: {cleaned_transcription}\n")
                        else:
                            # No wake word, store as-is
                            self._append_to_history(f"Human: {transcription}\n")
                            print(f"[WHISPER] Human: {transcription}")
                            self._log(f"Transcribed: {transcription}")
                    elif transcription:
                        # Log filtered hallucinations in debug mode
                        self._log(f"Filtered hallucination: {transcription}")

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
            try:
                self.audio_stream.stop()
            except Exception as e:
                self._log(f"Error stopping stream: {e}")

            try:
                self.audio_stream.close()
            except Exception as e:
                self._log(f"Error closing stream: {e}")

            self.audio_stream = None
            self._log("Audio stream stopped")

    def _append_to_history(self, text):
        """Append text to conversation_history.txt (thread-safe)"""
        try:
            with open(self.history_file, 'a') as f:
                f.write(text)
        except Exception as e:
            self._log(f"Error writing to conversation_history.txt: {e}")

    def _wake_word_detector(self):
        """
        Wake word detection worker thread

        Monitors wake_word_detected flag and triggers response.
        Flag is set by Whisper when wake word detected.
        Active only during LISTENING state.
        """
        self._log("Wake word detector started")

        while not self.shutdown_event.is_set():
            current_state = self._get_state()

            # Only active in LISTENING state
            if current_state == ConversationState.LISTENING:
                try:
                    # Check if wake word flag was set by Whisper
                    with self.wake_word_lock:
                        if self.wake_word_detected:
                            # Reset flag
                            self.wake_word_detected = False

                            # Always print wake word detection to console
                            print(f"\n[WAKE WORD] Detected! Processing...")
                            self._log(f"Wake word flag detected!")

                            # Check for memory reset command in recent history
                            content = self.history_file.read_text()
                            forget_pattern = r'\b(forget everything|clear memory|reset memory)\b'
                            forget_match = re.search(forget_pattern, content, re.IGNORECASE)

                            if forget_match:
                                print(f"\n[MEMORY] Reset command detected! Clearing conversation history...")
                                self.history_file.write_text('')
                                print(f"[MEMORY] Conversation history cleared. Starting fresh.")
                                continue

                            # Transition to RESPONDING state
                            self._set_state(ConversationState.RESPONDING)

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
        Ollama response generation using full conversation context

        Automatically summarizes context if it exceeds token limit.
        """
        print(f"[OLLAMA] Processing started...")
        self._log("Ollama processing started")

        try:
            # Read conversation history
            conversation_content = self.history_file.read_text()
            print(f"[OLLAMA] Read conversation_history.txt: {len(conversation_content)} chars")

            # Check if context exceeds limit - if so, summarize first
            token_count = self._estimate_token_count(conversation_content)
            print(f"[OLLAMA] Estimated tokens: {token_count} (limit: {self.context_limit})")

            if token_count > self.context_limit:
                print(f"[OLLAMA] Context exceeds limit! Summarizing...")
                self._speak_immediately("Let me think")

                # Create summary
                summary = self._summarize_conversation(conversation_content)
                if summary:
                    # Replace history with summary
                    self.history_file.write_text(f"Summary of previous conversation:\n{summary}\n\n")
                    print(f"[OLLAMA] History replaced with summary ({len(summary)} chars)")
                    conversation_content = self.history_file.read_text()
                else:
                    print(f"[OLLAMA] Warning: Summarization failed, using full context")

            # Build prompt
            prompt = (
                f"Conversation history:\n{conversation_content}\n\n"
                "You are Rosie, a friendly conversational robot. "
                "Respond naturally to what the human just said in 2-3 short sentences. "
                "Be conversational and use 'I' statements. Keep it brief and natural."
            )

            print(f"[OLLAMA] Total prompt length: {len(prompt)} chars")
            print(f"\n[OLLAMA] ===== PROMPT BEING SENT =====")
            print(prompt)
            print(f"[OLLAMA] ===== END PROMPT =====\n")
            print(f"[OLLAMA] Sending request to Ollama...")

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
                timeout=30  # Increased from 5 to 30 seconds for longer contexts
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
                print(f"[OLLAMA] Error: API returned status {response.status_code}")
                self._log(f"Ollama API error: {response.status_code}")
                self._set_state(ConversationState.LISTENING)

        except Exception as e:
            print(f"[OLLAMA] Error: {e}")
            import traceback
            traceback.print_exc()
            self._log(f"Ollama error: {e}")
            self._set_state(ConversationState.LISTENING)

    def _summarize_conversation(self, conversation_text):
        """
        Summarize conversation using Ollama

        Returns summary string or None on failure
        """
        try:
            print(f"[OLLAMA] Creating summary of {len(conversation_text)} chars...")

            summary_prompt = (
                f"Please summarize this conversation, keeping important information only:\n\n"
                f"{conversation_text}\n\n"
                f"Summary:"
            )

            response = requests.post(
                self.ollama_url,
                json={
                    'model': self.ollama_model,
                    'prompt': summary_prompt,
                    'temperature': 0.7,
                    'max_tokens': 500,
                    'stream': False
                },
                timeout=30
            )

            if response.status_code == 200:
                result = response.json()
                summary = result.get('response', '').strip()
                print(f"[OLLAMA] Summary created: {len(summary)} chars")
                return summary
            else:
                print(f"[OLLAMA] Summarization failed: HTTP {response.status_code}")
                return None

        except Exception as e:
            print(f"[OLLAMA] Summarization error: {e}")
            return None

    # =====================================================================
    # STATE 3: SPEAKING - Piper TTS Output
    # =====================================================================

    def _speak_immediately(self, text):
        """
        Speak text immediately without changing state (for system messages)

        Used for "Let me think" message during summarization.
        """
        try:
            import subprocess
            piper_cmd = f'echo "{text}" | piper --model {self.piper_model_path} --output-raw | aplay -r 22050 -f S16_LE -t raw -'
            subprocess.run(piper_cmd, shell=True, check=True, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
            print(f"[PIPER] {text}")
        except Exception as e:
            self._log(f"Piper immediate speech error: {e}")

    def _piper_speak(self):
        """
        Piper text-to-speech output

        Converts speak.txt to speech and returns to LISTENING state.
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

            # Append to conversation history with "Robot:" prefix
            self._append_to_history(f"Robot: {text}\n")

            # Clear speak.txt
            self.speak_file.write_text('')

            # Transition back to LISTENING state
            self._set_state(ConversationState.LISTENING)

        except Exception as e:
            self._log(f"Piper error: {e}")
            self._set_state(ConversationState.LISTENING)


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

        # Always print startup message and instructions
        print("\n" + "="*70)
        print("ROSIE Conversational AI System - READY")
        print("="*70)
        print(f"Current State: {self._get_state().name}")
        print("\nSpeak naturally - everything is transcribed.")
        print("Say 'Rosie' to get a response.")
        print("Say 'Rosie, forget everything' to reset conversation history.")
        print("Press CTRL+C to exit.")
        print("="*70 + "\n")

        self._log("ROSIE system started. Say 'Rosie' to get a response.")

        # Main loop (keep alive)
        try:
            while not self.shutdown_event.is_set():
                time.sleep(0.5)
        except KeyboardInterrupt:
            self.shutdown()

    def shutdown(self):
        """Graceful shutdown with SIGINT handling"""
        print("\n[SHUTDOWN] Stopping ROSIE system...")
        self._log("Shutting down ROSIE system...")

        # Signal all threads to stop
        self.shutdown_event.set()

        # Stop audio stream first (critical for clean shutdown)
        try:
            if self.audio_stream is not None:
                print("[SHUTDOWN] Stopping audio stream...")
                self._stop_audio_stream()
                print("[SHUTDOWN] Audio stream stopped")
        except Exception as e:
            print(f"[SHUTDOWN] Error stopping audio stream: {e}")

        # Wait for threads to finish (with timeout)
        print("[SHUTDOWN] Waiting for threads to finish...")
        threads = [self.whisper_thread, self.wake_word_thread]
        for thread in threads:
            if thread and thread.is_alive():
                thread.join(timeout=2)

        print("[SHUTDOWN] ROSIE system stopped cleanly")
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
    # Parse command-line arguments
    parser = argparse.ArgumentParser(
        description='ROSIE Conversational AI System - Fully Local Voice Assistant',
        epilog='Example: ./rosie_conversation.py'
    )
    args = parser.parse_args()

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
