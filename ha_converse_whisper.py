#!/usr/bin/env python3
"""
HA_converse Whisper Process - Speech Recognition and B4M Integration
Handles continuous speech-to-text and sends to B4M API
"""

import time
import numpy as np
import sounddevice as sd
import threading
import queue
from pathlib import Path
from typing import List
import sys
import os
import json
import requests

# Whisper imports - try faster-whisper first, fallback to openai-whisper
try:
    import faster_whisper
    WHISPER_BACKEND = "faster"
    print("Using faster-whisper (optimized)")
except ImportError:
    try:
        import whisper
        WHISPER_BACKEND = "openai"
        print("Using openai-whisper (standard)")
    except ImportError:
        print("Error: No Whisper implementation found!")
        print("Install: pip install faster-whisper OR pip install openai-whisper")
        sys.exit(1)


class WhisperSTT:
    """Whisper-based speech-to-text with B4M integration"""

    def __init__(self, model_name: str = "base", buffer_size: int = 20):
        self.model_name = model_name
        self.buffer_size = buffer_size
        self.word_buffer: List[str] = []
        self.last_write_time = time.time()
        self.last_word_count = 0
        self.running = False
        self.audio_queue = queue.Queue()

        # B4M API configuration
        self.b4m_api_key = os.environ.get('B4M_API_KEY', '')
        self.b4m_session_id = os.environ.get('B4M_ROSIE_ID', 'rosie-default-session')
        self.b4m_user_id = os.environ.get('B4M_USER_ID', 'user-default')
        self.b4m_api_url = "https://app.bike4mind.com/api/ai/llm"

        if not self.b4m_api_key:
            print("⚠️  WARNING: B4M_API_KEY not set - B4M integration disabled")
            print("   Set: export B4M_API_KEY='your_api_key'")
        else:
            print(f"✅ B4M API integration enabled")
            print(f"   Rosie ID: {self.b4m_session_id}")
            print(f"   User ID: {self.b4m_user_id}")

        # File paths
        self.conversation_file = Path("conversation.txt")
        self.response_file = Path("response.txt")
        self.prompts_dir = Path("prompts")

        # Audio settings
        self.sample_rate = 16000  # Whisper expects 16kHz
        self.chunk_duration = 10.0  # 10-second chunks
        self.overlap_duration = 0.5  # 0.5-second overlap

        # Initialize
        self._setup_directories()
        self._load_model()

    def _setup_directories(self):
        """Create necessary directories and files"""
        self.prompts_dir.mkdir(exist_ok=True)
        # Clear conversation file on startup
        self.conversation_file.write_text("")
        print(f"Initialized empty conversation.txt")

    def _load_model(self):
        """Load the Whisper model"""
        print(f"Loading Whisper {self.model_name} model...")

        if WHISPER_BACKEND == "faster":
            # Use faster-whisper with CPU optimization
            self.model = faster_whisper.WhisperModel(
                self.model_name,
                device="cpu",
                compute_type="int8"  # Optimize for CPU
            )
        else:
            # Use openai-whisper
            self.model = whisper.load_model(self.model_name)

        print(f"Model loaded successfully")

    def _transcribe_audio(self, audio: np.ndarray) -> str:
        """Transcribe audio to text using Whisper"""
        try:
            if WHISPER_BACKEND == "faster":
                # faster-whisper API
                segments, _ = self.model.transcribe(
                    audio,
                    language="en",
                    vad_filter=True,  # Voice activity detection
                    vad_parameters=dict(
                        min_silence_duration_ms=500
                    )
                )
                # Combine all segments
                text = " ".join([seg.text.strip() for seg in segments])
            else:
                # openai-whisper API
                result = self.model.transcribe(
                    audio,
                    language="en",
                    fp16=False  # CPU compatibility
                )
                text = result["text"].strip()

            return text
        except Exception as e:
            print(f"Transcription error: {e}")
            return ""

    def _send_buffer_to_b4m(self):
        """Send current buffer to B4M API and save response"""
        if not self.b4m_api_key or not self.word_buffer:
            return

        buffer_content = " ".join(self.word_buffer)

        try:
            print(f"🤖 Sending to B4M AI ({len(self.word_buffer)} words)...")

            headers = {
                "X-API-Key": self.b4m_api_key,
                "Content-Type": "application/json"
            }

            payload = {
                "sessionId": self.b4m_session_id,
                "messages": [
                    {
                        "role": "system",
                        "content": "You are a helpful AI assistant having a natural conversation. Keep responses concise and conversational."
                    },
                    {
                        "role": "user",
                        "content": buffer_content
                    }
                ],
                "userId": self.b4m_user_id,
                "llmModel": "gpt-4o-mini",
                "temperature": 0.7,
                "forceNewQuest": True,
                "checkKnowledge": False
            }

            response = requests.post(
                self.b4m_api_url,
                headers=headers,
                json=payload,
                timeout=10.0
            )

            if response.status_code == 200:
                result = response.json()

                # Poll for completion if needed
                if result.get('status') == 'running':
                    print("   Processing... polling for result")
                    result = self._poll_b4m_response(result, headers)

                # Extract AI response
                ai_response = self._extract_ai_response(result)

                if ai_response:
                    # Save to response.txt
                    self.response_file.write_text(ai_response, encoding='utf-8')
                    print(f"💾 AI response saved to response.txt")
                    print(f"   Length: {len(ai_response)} characters")
                else:
                    print("⚠️  No response from B4M")
            else:
                print(f"❌ B4M API Error: {response.status_code}")

        except Exception as e:
            print(f"❌ B4M API error: {str(e)}")

    def _poll_b4m_response(self, initial_response, headers):
        """Poll B4M API for quest completion"""
        quest_id = initial_response.get('questId') or initial_response.get('id')
        if not quest_id:
            return initial_response

        poll_url = f"{self.b4m_api_url}/quest/{quest_id}"
        max_polls = 15

        for i in range(max_polls):
            time.sleep(7)

            try:
                response = requests.get(poll_url, headers=headers, timeout=10.0)
                if response.status_code == 200:
                    result = response.json()

                    if result.get('status') == 'done':
                        print(f"   Response ready after {i+1} polls")
                        return result
                    elif result.get('status') == 'stopped':
                        print(f"   Quest stopped after {i+1} polls")
                        return result

            except Exception as e:
                print(f"   Polling error: {e}")

        return initial_response

    def _extract_ai_response(self, response_data):
        """Extract AI response from various B4M response formats"""
        # Try different response structures
        if response_data.get('replies'):
            replies = response_data['replies']
            if isinstance(replies, list) and replies:
                return replies[0] if isinstance(replies[0], str) else replies[0].get('content', '')

        # Try other common structures
        for field in ['reply', 'questMasterReply', 'researchModeResults']:
            if response_data.get(field):
                return response_data[field]

        # Try messages array
        if response_data.get('messages'):
            for msg in response_data['messages']:
                if msg.get('role') == 'assistant':
                    return msg.get('content', '')

        return None

    def _update_conversation_file(self):
        """Write buffer to conversation file"""
        content = " ".join(self.word_buffer)
        self.conversation_file.write_text(content)

    def _should_write_to_file(self) -> bool:
        """Determine if we should write to file"""
        time_elapsed = time.time() - self.last_write_time >= 10.0
        words_added = len(self.word_buffer) - self.last_word_count >= 10
        return time_elapsed or words_added

    def _process_transcription(self, text: str):
        """Process transcribed text and update buffer"""
        if not text or text.strip() == "":
            return

        # Split into words and clean
        words = text.strip().split()

        if not words:
            return

        # Add to buffer and check if we need to send to B4M
        old_buffer_size = len(self.word_buffer)

        if len(self.word_buffer) + len(words) > self.buffer_size:
            # Buffer will overflow - send to B4M before rollover
            if old_buffer_size >= self.buffer_size:
                print(f"\n🤖 Buffer full ({self.buffer_size} words) - sending to B4M...")
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
            if len(self.word_buffer) == self.buffer_size:
                print(f"\n🤖 Buffer full ({self.buffer_size} words) - sending to B4M...")
                self._send_buffer_to_b4m()

        # Update display
        word_count = len(self.word_buffer)
        print(f"\rBuffer: {word_count}/{self.buffer_size} words", end="", flush=True)

        # Write to file if needed
        if self._should_write_to_file():
            self._update_conversation_file()
            self.last_write_time = time.time()
            self.last_word_count = len(self.word_buffer)

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
                # Get audio from queue
                audio_chunk = self.audio_queue.get(timeout=0.1)
                audio_buffer.extend(audio_chunk)

                # Process when we have enough audio
                if len(audio_buffer) >= chunk_samples:
                    # Extract chunk with overlap
                    chunk = np.array(audio_buffer[:chunk_samples], dtype=np.float32)

                    # Keep overlap for next chunk
                    audio_buffer = audio_buffer[chunk_samples - overlap_samples:]

                    # Transcribe
                    text = self._transcribe_audio(chunk)

                    if text:
                        self._process_transcription(text)

            except queue.Empty:
                continue
            except Exception as e:
                print(f"\nTranscription error: {e}")

    def run(self):
        """Start the speech-to-text system"""
        print("=" * 50)
        print("HA_converse Whisper Process - Speech Recognition")
        print("=" * 50)
        print(f"Buffer size: {self.buffer_size} words")
        print(f"B4M API: {'ENABLED' if self.b4m_api_key else 'DISABLED'}")
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

        # Save final buffer
        if self.word_buffer:
            self._update_conversation_file()
            print(f"Final buffer saved: {len(self.word_buffer)} words")

        print("Whisper process stopped.")


def main():
    """Main entry point"""
    import argparse

    parser = argparse.ArgumentParser(description="HA_converse Whisper Process")
    parser.add_argument("--buffer-size", type=int, default=20,
                       help="Buffer size in words (default: 20)")
    parser.add_argument("--model", default="base",
                       choices=["tiny", "base", "small", "medium", "large"],
                       help="Whisper model size (default: base)")

    args = parser.parse_args()

    # Create and run the STT system
    stt = WhisperSTT(
        model_name=args.model,
        buffer_size=args.buffer_size
    )

    stt.run()


if __name__ == "__main__":
    main()