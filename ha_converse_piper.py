#!/usr/bin/env python3
"""
HA_converse Piper Process - Text-to-Speech Response Monitor
Monitors response.txt and speaks new content using Piper TTS
"""

import time
import os
import sys
import wave
import tempfile
import numpy as np
import sounddevice as sd
from pathlib import Path
from typing import Optional

# Check for Piper
try:
    from piper import PiperVoice
    PIPER_AVAILABLE = True
except ImportError:
    print("Error: piper-tts is not installed!")
    print("Install: pip install piper-tts")
    sys.exit(1)


class PiperTTS:
    """Piper TTS response monitor and speaker"""

    def __init__(self, trigger_word: str = "rosie", interactive_mode: bool = False):
        self.trigger_word = trigger_word.lower()
        self.interactive_mode = interactive_mode
        self.running = False

        # Files to monitor
        self.response_file = Path("response.txt")
        self.conversation_file = Path("conversation.txt")

        # Track last response to avoid repeats
        self.last_response_hash = None
        self.last_response_time = 0

        # Voice configuration
        self.piper_voice_name = os.environ.get('PIPER_VOICE', 'en_GB-jenny_dioco-medium')
        self.piper_model_path = os.environ.get('PIPER_MODEL_PATH')
        self.piper_config_path = os.environ.get('PIPER_CONFIG_PATH')
        self.piper_voice = None

        # Silence detection for interactive mode
        self.silence_timeout = 3.0  # 3 seconds
        self.last_activity_time = time.time()
        self.silence_triggered = False

        # Initialize Piper
        self._init_piper()

        # Startup test
        self._startup_test()

    def _init_piper(self):
        """Initialize Piper TTS"""
        # If no model path is set, try to use downloaded voice model
        if not self.piper_model_path:
            default_voice_dir = os.path.expanduser(f"~/.local/share/piper/voices")
            default_model_path = os.path.join(default_voice_dir, f"{self.piper_voice_name}.onnx")
            default_config_path = os.path.join(default_voice_dir, f"{self.piper_voice_name}.onnx.json")

            if os.path.exists(default_model_path) and os.path.exists(default_config_path):
                self.piper_model_path = default_model_path
                self.piper_config_path = default_config_path
                print(f"✅ Using Piper voice: {self.piper_voice_name}")
            else:
                print(f"⚠️  Piper voice '{self.piper_voice_name}' not found")
                print(f"   Download to: {default_voice_dir}")
                sys.exit(1)

        # Load the voice model
        try:
            if self.piper_config_path:
                self.piper_voice = PiperVoice.load(self.piper_model_path, self.piper_config_path)
            else:
                self.piper_voice = PiperVoice.load(self.piper_model_path)
            print("✅ Piper voice loaded successfully")
        except Exception as e:
            print(f"❌ Failed to load Piper voice: {e}")
            sys.exit(1)

    def _startup_test(self):
        """Speak startup message"""
        print("🔊 Testing Piper TTS...")
        self._speak_text("Hello World! Piper text-to-speech is working correctly.")

    def _speak_text(self, text: str):
        """Convert text to speech using Piper"""
        if not text:
            return

        try:
            print(f"🔊 Speaking: {text[:50]}{'...' if len(text) > 50 else ''}")

            # Create temporary WAV file
            with tempfile.NamedTemporaryFile(suffix='.wav', delete=False) as temp_file:
                temp_path = temp_file.name

            # Generate speech
            audio_chunks = list(self.piper_voice.synthesize(text))

            # Save to WAV file
            with wave.open(temp_path, 'wb') as wav_file:
                if audio_chunks:
                    first_chunk = audio_chunks[0]
                    wav_file.setnchannels(first_chunk.sample_channels)
                    wav_file.setsampwidth(first_chunk.sample_width)
                    wav_file.setframerate(first_chunk.sample_rate)

                    for chunk in audio_chunks:
                        wav_file.writeframes(chunk.audio_int16_bytes)

            # Play the audio
            self._play_audio_file(temp_path)

            # Clean up
            try:
                os.unlink(temp_path)
            except:
                pass

        except Exception as e:
            print(f"❌ TTS Error: {str(e)}")

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
                print(f"⚠️  Unsupported audio format: {sample_width} bytes")
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
            print(f"❌ Audio playback error: {str(e)}")

    def _check_for_trigger(self) -> bool:
        """Check if trigger word is in conversation file"""
        if self.interactive_mode:
            return False  # Use silence detection instead

        try:
            if self.conversation_file.exists():
                content = self.conversation_file.read_text().lower()
                return self.trigger_word in content
        except:
            pass
        return False

    def _check_silence_trigger(self) -> bool:
        """Check if silence duration exceeds threshold in interactive mode"""
        if not self.interactive_mode:
            return False

        # Check if conversation file has been modified recently
        try:
            if self.conversation_file.exists():
                mtime = os.path.getmtime(self.conversation_file)
                if mtime > self.last_activity_time:
                    self.last_activity_time = mtime
                    self.silence_triggered = False
                    print("\r" + " " * 50 + "\r", end="", flush=True)  # Clear timer
                    return False
        except:
            pass

        # Calculate silence duration
        current_time = time.time()
        silence_duration = current_time - self.last_activity_time

        # Show timer after 0.5 seconds
        if silence_duration >= 0.5 and not self.silence_triggered:
            print(f"\r⏳ Silence timer: {silence_duration:.1f}s / {self.silence_timeout}s",
                  end="", flush=True)

        # Check if threshold reached
        if silence_duration >= self.silence_timeout and not self.silence_triggered:
            print(f"\n🤫 {self.silence_timeout} seconds of silence detected")
            self.silence_triggered = True
            self.last_activity_time = current_time
            return True

        return False

    def _get_response_hash(self, content: str) -> str:
        """Get a simple hash of the content to detect changes"""
        return f"{len(content)}:{hash(content)}"

    def _check_and_speak_response(self):
        """Check for new response and speak it"""
        try:
            if not self.response_file.exists():
                return

            content = self.response_file.read_text(encoding='utf-8').strip()
            if not content:
                return

            # Check if this is new content
            content_hash = self._get_response_hash(content)
            if content_hash == self.last_response_hash:
                return  # Already spoken

            # Check for trigger
            should_speak = False

            if self.interactive_mode:
                # Check silence trigger
                if self._check_silence_trigger():
                    should_speak = True
            else:
                # Check keyword trigger
                if self._check_for_trigger():
                    print(f"🎯 Trigger word '{self.trigger_word}' detected")
                    should_speak = True

            if should_speak:
                # Speak the response
                self._speak_text(content)

                # Mark as spoken
                self.last_response_hash = content_hash
                self.last_response_time = time.time()

                # Clear the response file
                self.response_file.write_text("", encoding='utf-8')
                print("💾 Cleared response.txt after speaking")

                # Reset silence trigger
                if self.interactive_mode:
                    self.silence_triggered = False
                    self.last_activity_time = time.time()

        except Exception as e:
            print(f"❌ Error checking response: {e}")

    def run(self):
        """Main monitoring loop"""
        print("=" * 50)
        print("HA_converse Piper Process - TTS Response Monitor")
        print("=" * 50)
        print(f"Voice: {self.piper_voice_name}")
        if self.interactive_mode:
            print(f"Mode: Interactive (silence trigger: {self.silence_timeout}s)")
        else:
            print(f"Mode: Keyword (trigger: '{self.trigger_word}')")
        print(f"Monitoring response.txt for new content...")
        print(f"Press Ctrl+C to stop\n")

        self.running = True

        try:
            while self.running:
                self._check_and_speak_response()
                time.sleep(0.5)  # Check every 500ms

        except KeyboardInterrupt:
            self.stop()
        except Exception as e:
            print(f"\n❌ Error: {e}")
            self.stop()

    def stop(self):
        """Stop the monitoring loop"""
        print("\n\nShutting down...")
        self.running = False
        print("Piper process stopped.")


def main():
    """Main entry point"""
    import argparse

    parser = argparse.ArgumentParser(description="HA_converse Piper Process")
    parser.add_argument("--interactive", action="store_true",
                       help="Use silence detection instead of keyword trigger")
    parser.add_argument("--trigger-word", default="rosie",
                       help="Trigger word for speaking (default: rosie)")

    args = parser.parse_args()

    # Create and run the TTS monitor
    tts = PiperTTS(
        trigger_word=args.trigger_word,
        interactive_mode=args.interactive
    )

    tts.run()


if __name__ == "__main__":
    main()