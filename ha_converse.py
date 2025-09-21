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

    def __init__(self, model_name: str = "base", buffer_size: int = 100):
        self.model_name = model_name
        self.buffer_size = buffer_size
        self.word_buffer: List[str] = []
        self.last_write_time = time.time()
        self.last_word_count = 0
        self.running = False
        self.audio_queue = queue.Queue()

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
        timestamp = datetime.now().strftime("%Y-%m-%d-%H-%M-%S")
        archive_file = self.prompts_dir / f"prompt_{timestamp}.txt"

        # Copy current conversation to archive
        content = self.conversation_file.read_text()
        archive_file.write_text(content)

        print(f"Archive created: {archive_file}")

        # Clear conversation file and buffer
        self.conversation_file.write_text("")
        self.word_buffer.clear()

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

    def _process_transcription(self, text: str):
        """Process transcribed text and update buffer"""
        if not text or text.strip() == "":
            return

        # Split into words and clean
        words = text.strip().split()
        words = self._remove_consecutive_duplicates(words)

        if not words:
            return

        # Add to buffer
        self.word_buffer.extend(words)

        # Check if we need to archive
        if len(self.word_buffer) >= self.buffer_size:
            self._archive_conversation()

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

    # Start the system
    stt = WhisperSTT(model_name="base", buffer_size=100)

    try:
        stt.start()
    except Exception as e:
        print(f"Fatal error: {e}")
        sys.exit(1)


if __name__ == "__main__":
    main()