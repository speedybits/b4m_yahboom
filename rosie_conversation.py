#!/usr/bin/env python3
"""
ROSIE Conversational AI System
Based on CONVERSE_B4M_OLLAMA_HYBRID specification

Combines:
- Whisper (speech-to-text)
- Ollama (local LLM for immediate responses)
- bike4mind API (background intelligence with internet access)
- Piper (text-to-speech)

Architecture: Non-blocking with progressive intelligence enhancement
"""

import os
import sys
import time
import signal
import threading
import json
import requests
from enum import Enum
from pathlib import Path
from typing import Optional
import subprocess
import numpy as np
import sounddevice as sd
from faster_whisper import WhisperModel


# File paths
LISTEN_FILE = "/tmp/listen.txt"
SUMMARY_FILE = "/tmp/summary.txt"
SPEAK_FILE = "/tmp/speak.txt"


class State(Enum):
    """State machine states"""
    LISTENING = "listening"
    RESPONDING = "responding"
    SPEAKING = "speaking"


class StateMachine:
    """Thread-safe state machine for conversation flow"""

    def __init__(self):
        self.state = State.LISTENING
        self.lock = threading.Lock()
        self.shutdown_event = threading.Event()

    def get_state(self) -> State:
        """Get current state (thread-safe)"""
        with self.lock:
            return self.state

    def set_state(self, new_state: State):
        """Set new state (thread-safe)"""
        with self.lock:
            print(f"[STATE] {self.state.value} → {new_state.value}")
            self.state = new_state

    def is_shutdown(self) -> bool:
        """Check if shutdown requested"""
        return self.shutdown_event.is_set()

    def request_shutdown(self):
        """Request graceful shutdown"""
        print("\n[SHUTDOWN] Shutdown requested...")
        self.shutdown_event.set()


class WhisperWorker:
    """Handles speech-to-text using faster-whisper"""

    def __init__(self, state_machine: StateMachine):
        self.state_machine = state_machine
        self.thread = None
        self.model = None
        self.sample_rate = 16000
        self.chunk_duration = 3  # Record 3 second chunks
        self.is_recording = False

    def start(self):
        """Start Whisper worker thread"""
        print("[WHISPER] Loading model...")
        # Use base model for good balance of speed and accuracy
        self.model = WhisperModel("base", device="cpu", compute_type="int8")
        print("[WHISPER] Model loaded")

        self.thread = threading.Thread(target=self._run, daemon=True)
        self.thread.start()
        print("[WHISPER] Worker started")

    def _run(self):
        """Main Whisper worker loop"""
        while not self.state_machine.is_shutdown():
            # Only transcribe when in LISTENING state
            if self.state_machine.get_state() == State.LISTENING:
                self._transcribe()
            else:
                # Pause during RESPONDING and SPEAKING states
                time.sleep(0.1)

    def _transcribe(self):
        """
        Capture audio and transcribe to listen.txt with 'Human said:' prefix
        """
        try:
            # Record audio chunk
            duration = self.chunk_duration

            audio_data = sd.rec(
                int(duration * self.sample_rate),
                samplerate=self.sample_rate,
                channels=1,
                dtype='float32'
            )
            sd.wait()  # Wait for recording to complete

            # Convert to format expected by faster-whisper
            audio_np = audio_data.flatten()

            # Transcribe with error handling for empty segments
            try:
                segments, info = self.model.transcribe(
                    audio_np,
                    beam_size=5,
                    vad_filter=True,  # Use voice activity detection
                    vad_parameters=dict(min_silence_duration_ms=500)
                )

                # Extract text from segments
                transcribed_text = ""
                segment_count = 0
                for segment in segments:
                    transcribed_text += segment.text + " "
                    segment_count += 1

                transcribed_text = transcribed_text.strip()

                if transcribed_text:
                    print(f"[WHISPER] Transcribed: {transcribed_text}")

                    # Append to listen.txt with "Human said:" prefix
                    human_said = f"Human said: {transcribed_text}"

                    if os.path.exists(LISTEN_FILE):
                        with open(LISTEN_FILE, 'a') as f:
                            f.write(f" {human_said}")
                    else:
                        with open(LISTEN_FILE, 'w') as f:
                            f.write(human_said)
                # else: silence, no transcription needed

            except ValueError as e:
                # Handle "max() arg is an empty sequence" - means no speech detected
                # This is normal when there's silence, just skip this chunk
                pass
            except Exception as e:
                print(f"[WHISPER] Transcription error: {e}")

        except Exception as e:
            print(f"[WHISPER] Recording error: {e}")
            time.sleep(0.5)

    def stop(self):
        """Stop Whisper worker"""
        print("[WHISPER] Worker stopped")


class WakeWordDetector:
    """Detects 'Rosie' wake word and triggers response"""

    def __init__(self, state_machine: StateMachine):
        self.state_machine = state_machine
        self.thread = None
        self.last_check_time = 0

    def start(self):
        """Start wake word detector thread"""
        self.thread = threading.Thread(target=self._run, daemon=True)
        self.thread.start()
        print("[WAKEWORD] Detector started")

    def _run(self):
        """Monitor listen.txt for 'Rosie' wake word"""
        while not self.state_machine.is_shutdown():
            # Only detect wake word when LISTENING
            if self.state_machine.get_state() == State.LISTENING:
                self._check_for_wake_word()
            time.sleep(0.1)

    def _check_for_wake_word(self):
        """Check if 'Rosie' appears in listen.txt"""
        if not os.path.exists(LISTEN_FILE):
            return

        try:
            # Check if file was modified since last check
            mtime = os.path.getmtime(LISTEN_FILE)
            if mtime <= self.last_check_time:
                return
            self.last_check_time = mtime

            with open(LISTEN_FILE, 'r') as f:
                content = f.read()

            # Check if "Rosie" appears (case-insensitive)
            if "rosie" in content.lower():
                print("[WAKEWORD] Detected 'Rosie'!")

                # Remove "Rosie" from the text (keep "Human said:" prefix)
                # Remove case-insensitive, preserve formatting
                import re
                updated_content = re.sub(r'\bRosie\b', '', content, flags=re.IGNORECASE)
                updated_content = re.sub(r'\s+', ' ', updated_content).strip()

                # Write back to listen.txt
                with open(LISTEN_FILE, 'w') as f:
                    f.write(updated_content)

                # Transition to RESPONDING state
                self.state_machine.set_state(State.RESPONDING)

        except Exception as e:
            print(f"[WAKEWORD] Error: {e}")


class OllamaResponder:
    """Generates immediate responses using local Ollama"""

    def __init__(self, state_machine: StateMachine):
        self.state_machine = state_machine
        self.thread = None

    def start(self):
        """Start Ollama responder thread"""
        self.thread = threading.Thread(target=self._run, daemon=True)
        self.thread.start()
        print("[OLLAMA] Responder started")

    def _run(self):
        """Monitor for RESPONDING state and generate response"""
        while not self.state_machine.is_shutdown():
            if self.state_machine.get_state() == State.RESPONDING:
                self._generate_response()
            time.sleep(0.1)

    def _generate_response(self):
        """Generate Ollama response from listen.txt and summary.txt"""
        try:
            # Read conversation history
            conversation = ""
            if os.path.exists(LISTEN_FILE):
                with open(LISTEN_FILE, 'r') as f:
                    conversation = f.read().strip()

            # Read summary if available
            summary = ""
            if os.path.exists(SUMMARY_FILE):
                with open(SUMMARY_FILE, 'r') as f:
                    summary = f.read().strip()

            # Build prompt for Ollama
            context = f"Conversation:\n{conversation}\n"
            if summary:
                context += f"\nContext and insights:\n{summary}\n"

            prompt = f"{context}\nPlease respond to this question in only words (no symbols or emojis). If you don't know the answer, please tell them that you are thinking about it."

            # Call Ollama API
            response_text = self._call_ollama(prompt)

            # Write to speak.txt
            with open(SPEAK_FILE, 'w') as f:
                f.write(response_text)

            print(f"[OLLAMA] Response generated: {response_text[:50]}...")

            # Transition to SPEAKING state
            self.state_machine.set_state(State.SPEAKING)

        except Exception as e:
            print(f"[OLLAMA] Error: {e}")
            # Fallback response
            with open(SPEAK_FILE, 'w') as f:
                f.write("I'm having trouble processing that right now.")
            self.state_machine.set_state(State.SPEAKING)

    def _call_ollama(self, prompt: str) -> str:
        """
        Call local Ollama API
        """
        try:
            url = "http://localhost:11434/api/generate"
            payload = {
                "model": "qwen2.5:0.5b",  # Use available model
                "prompt": prompt,
                "stream": False,
                "options": {
                    "temperature": 0.7,
                    "num_predict": 100  # Limit tokens for quick response
                }
            }

            response = requests.post(url, json=payload, timeout=10.0)

            if response.status_code == 200:
                result = response.json()
                return result.get('response', 'I am thinking about it.').strip()
            else:
                print(f"[OLLAMA] API error: {response.status_code}")
                return "I am thinking about it."

        except requests.Timeout:
            print("[OLLAMA] Request timeout")
            return "I am thinking about it."
        except Exception as e:
            print(f"[OLLAMA] Error: {e}")
            return "I am thinking about it."


class PiperSpeaker:
    """Handles text-to-speech using Piper"""

    def __init__(self, state_machine: StateMachine):
        self.state_machine = state_machine
        self.thread = None

    def start(self):
        """Start Piper speaker thread"""
        self.thread = threading.Thread(target=self._run, daemon=True)
        self.thread.start()
        print("[PIPER] Speaker started")

    def _run(self):
        """Monitor for SPEAKING state and play audio"""
        while not self.state_machine.is_shutdown():
            if self.state_machine.get_state() == State.SPEAKING:
                self._speak()
            time.sleep(0.1)

    def _speak(self):
        """Read speak.txt, play audio, append to listen.txt"""
        try:
            if not os.path.exists(SPEAK_FILE):
                self.state_machine.set_state(State.LISTENING)
                return

            # Read text to speak
            with open(SPEAK_FILE, 'r') as f:
                text = f.read().strip()

            if not text:
                self.state_machine.set_state(State.LISTENING)
                return

            print(f"[PIPER] Speaking: {text}")

            # Call Piper TTS
            self._call_piper(text)

            # Append to listen.txt with "Robot said:" prefix
            robot_said = f"Robot said: {text}"
            if os.path.exists(LISTEN_FILE):
                with open(LISTEN_FILE, 'a') as f:
                    f.write(f" {robot_said}")
            else:
                with open(LISTEN_FILE, 'w') as f:
                    f.write(robot_said)

            # Clear speak.txt
            if os.path.exists(SPEAK_FILE):
                os.remove(SPEAK_FILE)

            print("[PIPER] Finished speaking")

            # Transition back to LISTENING
            self.state_machine.set_state(State.LISTENING)

        except Exception as e:
            print(f"[PIPER] Error: {e}")
            self.state_machine.set_state(State.LISTENING)

    def _call_piper(self, text: str):
        """
        Call Piper TTS to generate and play audio
        """
        piper_process = None
        play_process = None

        try:
            # Use environment variables from .bashrc, with fallback to jenny voice
            piper_cmd = "/home/mike/.local/bin/piper"
            voice_model = os.environ.get('PIPER_MODEL_PATH',
                                        "/home/mike/.local/share/piper/voices/en_GB-jenny_dioco-medium.onnx")

            # Generate audio and play directly
            # Piper outputs WAV to stdout, pipe to aplay for playback
            piper_process = subprocess.Popen(
                [piper_cmd, "--model", voice_model, "--output-raw"],
                stdin=subprocess.PIPE,
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE
            )

            # Send text to Piper and capture audio
            audio_output, error = piper_process.communicate(input=text.encode('utf-8'), timeout=10)

            if error:
                error_text = error.decode('utf-8').strip()
                if error_text:  # Only print non-empty errors
                    print(f"[PIPER] stderr: {error_text}")

            # Play audio using aplay
            if audio_output:
                play_process = subprocess.Popen(
                    ["aplay", "-r", "22050", "-f", "S16_LE", "-t", "raw", "-", "-q"],  # -q for quiet
                    stdin=subprocess.PIPE,
                    stdout=subprocess.DEVNULL,
                    stderr=subprocess.PIPE
                )
                _, play_error = play_process.communicate(input=audio_output, timeout=30)

                if play_error:
                    play_error_text = play_error.decode('utf-8').strip()
                    if play_error_text and "no soundcards found" not in play_error_text.lower():
                        print(f"[PIPER] Audio playback: {play_error_text}")

        except subprocess.TimeoutExpired as e:
            print(f"[PIPER] Timeout: {e}")
            if piper_process:
                piper_process.kill()
            if play_process:
                play_process.kill()
        except FileNotFoundError as e:
            print(f"[PIPER] Command not found: {e}")
        except Exception as e:
            print(f"[PIPER] Error: {e}")
        finally:
            # Ensure processes are cleaned up
            for proc in [piper_process, play_process]:
                if proc and proc.poll() is None:
                    try:
                        proc.kill()
                    except:
                        pass


class Bike4mindWorker:
    """Background worker for bike4mind API intelligence"""

    def __init__(self, state_machine: StateMachine):
        self.state_machine = state_machine
        self.thread = None
        self.last_processed_content = ""
        self.api_key = os.environ.get('B4M_API_KEY')
        self.session_id = os.environ.get('B4M_OLLAMA_CONVERSATION_ID')
        self.user_id = os.environ.get('B4M_USER_ID', '65563f622213b120cd1d9592')

    def start(self):
        """Start bike4mind background worker"""
        if not self.api_key or not self.session_id:
            print("[BIKE4MIND] WARNING: B4M_API_KEY or B4M_OLLAMA_CONVERSATION_ID not set - worker disabled")
            return

        self.thread = threading.Thread(target=self._run, daemon=True)
        self.thread.start()
        print("[BIKE4MIND] Worker started")

    def _run(self):
        """Monitor listen.txt and update summary.txt asynchronously"""
        while not self.state_machine.is_shutdown():
            self._check_for_updates()

            # Use interruptible sleep
            for _ in range(20):  # 20 * 0.1s = 2 seconds
                if self.state_machine.is_shutdown():
                    print("[BIKE4MIND] Worker stopping...")
                    return
                time.sleep(0.1)

    def _check_for_updates(self):
        """Check if listen.txt has new content and process"""
        if not os.path.exists(LISTEN_FILE):
            return

        try:
            with open(LISTEN_FILE, 'r') as f:
                content = f.read().strip()

            # Only process if content has changed
            if content and content != self.last_processed_content:
                print("[BIKE4MIND] New conversation detected, analyzing...")
                self.last_processed_content = content

                # Process in background (don't block)
                summary = self._analyze_conversation(content)

                if summary:
                    # Write to summary.txt
                    with open(SUMMARY_FILE, 'w') as f:
                        f.write(summary)
                    print(f"[BIKE4MIND] Summary updated: {summary[:50]}...")

        except Exception as e:
            print(f"[BIKE4MIND] Error: {e}")

    def _analyze_conversation(self, conversation: str) -> Optional[str]:
        """
        Send conversation to bike4mind API for analysis
        Implements quest-based polling system from B4M_API_HOWTO.md
        """
        try:
            # Step 1: Submit quest to bike4mind API
            api_url = "https://app.bike4mind.com/api/ai/llm"
            headers = {
                "X-API-Key": self.api_key,
                "Content-Type": "application/json"
            }

            payload = {
                "sessionId": self.session_id,
                "message": f"Please summarize this conversation into 3 sentences with only words, including intelligent insights:\n\n{conversation}",
                "historyCount": 10,
                "fabFileIds": [],
                "messageFileIds": [],
                "params": {
                    "model": "gpt-4o-mini",
                    "temperature": 0.7,
                    "max_tokens": 500,
                    "stream": False
                },
                "promptMeta": {
                    "session": {
                        "id": self.session_id,
                        "userId": self.user_id
                    }
                }
            }

            print("[BIKE4MIND] Submitting quest...")
            response = requests.post(api_url, headers=headers, json=payload, timeout=10.0)

            if response.status_code != 200:
                print(f"[BIKE4MIND] API error: {response.status_code}")
                return None

            quest_data = response.json()
            quest_id = quest_data.get('id')  # Note: 'id' not 'questId'

            if not quest_id:
                print("[BIKE4MIND] No quest ID in response")
                return None

            print(f"[BIKE4MIND] Quest submitted: {quest_id}")

            # Step 2: Poll for completion (7 second intervals, max 15 attempts)
            return self._poll_for_completion(quest_id)

        except requests.Timeout:
            print("[BIKE4MIND] Request timeout")
            return None
        except Exception as e:
            print(f"[BIKE4MIND] Error: {e}")
            return None

    def _poll_for_completion(self, quest_id: str) -> Optional[str]:
        """Poll quest until completion or timeout (based on B4M_API_HOWTO.md)"""
        poll_url = f"https://app.bike4mind.com/api/sessions/{self.session_id}/chat/{quest_id}"
        headers = {"X-API-Key": self.api_key}

        for attempt in range(15):  # 15 attempts = 105 seconds max
            # Check for shutdown before sleeping
            if self.state_machine.is_shutdown():
                print("[BIKE4MIND] Shutdown detected, stopping poll")
                return None

            # Use interruptible sleep
            for _ in range(70):  # 70 * 0.1s = 7 seconds
                if self.state_machine.is_shutdown():
                    return None
                time.sleep(0.1)

            try:
                response = requests.get(poll_url, headers=headers, timeout=5.0)

                if response.status_code == 200:
                    quest_data = response.json()
                    status = quest_data.get('status')

                    if status == 'done':
                        print(f"[BIKE4MIND] Quest completed (attempt {attempt + 1})")
                        return self._extract_ai_response(quest_data)

                    elif status == 'stopped':
                        print("[BIKE4MIND] Quest was stopped")
                        return None

                    # Status is 'running', continue polling
                    print(f"[BIKE4MIND] Polling... (attempt {attempt + 1}/15)")

            except requests.Timeout:
                continue  # Continue polling on timeout

        print("[BIKE4MIND] Timeout: No response after 105 seconds")
        return None

    def _extract_ai_response(self, quest_data: dict) -> Optional[str]:
        """
        Extract AI response with multiple fallback methods
        Based on B4M_API_HOWTO.md
        """
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

        return None


class RosieConversationSystem:
    """Main application controller"""

    def __init__(self):
        self.state_machine = StateMachine()
        self.whisper_worker = WhisperWorker(self.state_machine)
        self.wake_word_detector = WakeWordDetector(self.state_machine)
        self.ollama_responder = OllamaResponder(self.state_machine)
        self.piper_speaker = PiperSpeaker(self.state_machine)
        self.bike4mind_worker = Bike4mindWorker(self.state_machine)

        # Setup signal handlers for graceful shutdown
        signal.signal(signal.SIGINT, self._signal_handler)
        signal.signal(signal.SIGTERM, self._signal_handler)

    def _signal_handler(self, signum, frame):
        """Handle CTRL+C and SIGTERM for graceful shutdown"""
        self.state_machine.request_shutdown()

    def start(self):
        """Start all workers"""
        print("=" * 60)
        print("ROSIE Conversational AI System")
        print("Based on CONVERSE_B4M_OLLAMA_HYBRID specification")
        print("=" * 60)
        print("\nInitializing file-based communication...")

        # Initialize files
        self._initialize_files()

        print("\nStarting workers...")
        self.whisper_worker.start()
        self.wake_word_detector.start()
        self.ollama_responder.start()
        self.piper_speaker.start()
        self.bike4mind_worker.start()

        print("\n" + "=" * 60)
        print("System ready! Say 'Rosie' to activate conversation.")
        print("Press CTRL+C to shutdown gracefully.")
        print("=" * 60 + "\n")

        # Main loop - just monitor shutdown
        try:
            while not self.state_machine.is_shutdown():
                time.sleep(0.5)
        except KeyboardInterrupt:
            self.state_machine.request_shutdown()

        self._cleanup()

    def _initialize_files(self):
        """Initialize conversation files"""
        # Clear speak.txt if exists
        if os.path.exists(SPEAK_FILE):
            os.remove(SPEAK_FILE)

        # Create listen.txt if doesn't exist
        if not os.path.exists(LISTEN_FILE):
            Path(LISTEN_FILE).touch()

        print(f"  listen.txt:  {LISTEN_FILE}")
        print(f"  summary.txt: {SUMMARY_FILE}")
        print(f"  speak.txt:   {SPEAK_FILE}")

    def _cleanup(self):
        """Cleanup on shutdown"""
        print("\n[SHUTDOWN] Cleaning up...")

        # Stop workers
        self.whisper_worker.stop()

        # Wait for threads to finish (with shorter timeout for faster shutdown)
        threads = [
            self.whisper_worker.thread,
            self.wake_word_detector.thread,
            self.ollama_responder.thread,
            self.piper_speaker.thread,
            self.bike4mind_worker.thread
        ]

        for thread in threads:
            if thread and thread.is_alive():
                thread.join(timeout=0.5)

        # Force exit if threads still running
        import sys
        print("[SHUTDOWN] Complete. Goodbye!")
        sys.exit(0)


def main():
    """Main entry point"""
    system = RosieConversationSystem()
    system.start()


if __name__ == "__main__":
    main()
