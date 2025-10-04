#!/usr/bin/env python3
"""
HA_converse - Speech-to-Text Conversation System with B4M AI Integration

A two-thread application that captures speech, sends 20-word segments to B4M API,
and speaks AI responses via Piper TTS when triggered by "Rosie" keyword.
"""

import os
import sys
import time
import signal
import threading
import argparse
import glob
from datetime import datetime
from pathlib import Path
import requests
import numpy as np

# Try importing faster-whisper, fallback to openai-whisper
try:
    from faster_whisper import WhisperModel
    USING_FASTER_WHISPER = True
except ImportError:
    import whisper
    USING_FASTER_WHISPER = False
    print("⚠️ faster-whisper not available, using openai-whisper (slower)")

try:
    import sounddevice as sd
    AUDIO_AVAILABLE = True
except ImportError:
    AUDIO_AVAILABLE = False
    print("⚠️ sounddevice not available - test mode only")

# Global configuration
BUFFER_SIZE = 20  # words
TRIGGER_WORD = "rosie"  # case-insensitive
CHUNK_DURATION = 10.0  # seconds
MINI_CHUNK_DURATION = 0.5  # seconds for interruptible recording
SAMPLE_RATE = 16000
B4M_POLLING_INTERVAL = 7  # seconds
B4M_MAX_POLLING_ATTEMPTS = 15  # 105 seconds total
B4M_RATE_LIMIT_RETRIES = 3
TEST_MODE_INTERVAL = 3.0  # seconds between test sentences

# Global state
shutdown_event = threading.Event()
response_queue_lock = threading.Lock()
conversation_queue_lock = threading.Lock()  # Prevents multiple threads from processing same file
tts_speaking_event = threading.Event()  # Set when TTS is speaking to pause recording
clear_buffer_event = threading.Event()  # Set when buffer should be cleared after TTS
conversation_counter = 0
conversation_counter_lock = threading.Lock()

# Environment variables
B4M_API_KEY = os.environ.get('B4M_API_KEY', '')
B4M_SESSION_ID = os.environ.get('B4M_ROSIE_ID', os.environ.get('B4M_SESSION_ID', ''))
B4M_USER_ID = os.environ.get('B4M_USER_ID', '65563f622213b120cd1d9592')
PIPER_MODEL_PATH = os.environ.get('PIPER_MODEL_PATH', '')
PIPER_CONFIG_PATH = os.environ.get('PIPER_CONFIG_PATH', '')


def signal_handler(sig, frame):
    """Handle SIGINT (Ctrl+C) and SIGTERM for graceful shutdown"""
    print("\n\n🛑 Shutdown signal received - stopping all threads...")
    shutdown_event.set()


def cleanup_old_files():
    """Delete all existing conversation and response files on startup"""
    conversation_files = glob.glob('conversation_*.txt')
    response_files = glob.glob('response_*.txt')

    deleted_count = 0
    for file_path in conversation_files + response_files:
        # Skip conversation_test.txt (test mode input file)
        if file_path == 'conversation_test.txt':
            continue

        try:
            os.remove(file_path)
            deleted_count += 1
        except Exception as e:
            print(f"⚠️ Could not delete {file_path}: {e}")

    if deleted_count > 0:
        print(f"🗑️ Cleaned up {deleted_count} old conversation/response files")


def get_next_conversation_counter():
    """Thread-safe counter for conversation/response file pairing"""
    global conversation_counter
    with conversation_counter_lock:
        conversation_counter += 1
        return conversation_counter


def find_oldest_file(pattern):
    """Find oldest file matching pattern by timestamp in filename"""
    files = glob.glob(pattern)
    if not files:
        return None

    # Sort by timestamp in filename (YYYY-MM-DD_HH-MM-SS format sorts lexicographically)
    files.sort()
    return files[0] if files else None


def extract_quest_id(response_data):
    """Extract quest ID from B4M API response"""
    # Try 'questId' field first
    if 'questId' in response_data:
        return response_data['questId']
    # Fallback to 'id' field (observed in actual API responses)
    if 'id' in response_data:
        return response_data['id']
    return None


def extract_ai_response(quest_data):
    """Extract AI response text from quest data using multiple fallback methods"""
    # Primary: replies array (current B4M structure)
    if 'replies' in quest_data and isinstance(quest_data['replies'], list) and len(quest_data['replies']) > 0:
        return '\n'.join(quest_data['replies'])

    # Fallback 1: reply field (legacy)
    if 'reply' in quest_data and quest_data['reply']:
        return quest_data['reply']

    # Fallback 2: questMasterReply
    if 'questMasterReply' in quest_data and quest_data['questMasterReply']:
        return quest_data['questMasterReply']

    # Fallback 3: researchModeResults
    if 'researchModeResults' in quest_data and isinstance(quest_data['researchModeResults'], list):
        results = [r.get('response', '') for r in quest_data['researchModeResults'] if r.get('response')]
        if results:
            return '\n\n'.join(results)

    # Fallback 4: messages array
    if 'messages' in quest_data and isinstance(quest_data['messages'], list):
        for msg in quest_data['messages']:
            if msg.get('content'):
                return msg['content']

    return None


def parse_rate_limit_wait_time(error_text):
    """Parse wait time from rate limit error message"""
    # Example: "Try again in 59.165 seconds"
    import re
    match = re.search(r'Try again in ([\d.]+) seconds', error_text)
    if match:
        return float(match.group(1))
    return 60.0  # Default to 60 seconds


def send_to_b4m_api(message_text, file_counter):
    """
    Send conversation to B4M API with quest-based polling
    Returns: AI response text or None on failure
    """
    if not B4M_API_KEY or not B4M_SESSION_ID:
        print("⚠️ B4M API credentials not configured - skipping API call")
        return None

    headers = {
        "X-API-Key": B4M_API_KEY,
        "Content-Type": "application/json"
    }

    payload = {
        "sessionId": B4M_SESSION_ID,
        "message": f"Please respond in a single sentence.: {message_text}",
        "historyCount": 10,
        "fabFileIds": [],
        "messageFileIds": [],
        "params": {
            "model": "gpt-4o-mini",
            "temperature": 0.3,
            "max_tokens": 100,
            "stream": False
        },
        "promptMeta": {
            "session": {
                "id": B4M_SESSION_ID,
                "userId": B4M_USER_ID
            }
        }
    }

    # Retry loop for rate limiting
    for retry_attempt in range(1, B4M_RATE_LIMIT_RETRIES + 1):
        try:
            # Initial quest creation
            response = requests.post(
                "https://app.bike4mind.com/api/ai/llm",
                headers=headers,
                json=payload,
                timeout=10.0
            )

            if response.status_code == 429:
                # Rate limited
                error_data = response.json() if response.text else {}
                wait_time = parse_rate_limit_wait_time(str(error_data))
                print(f"⚠️ B4M API: Rate limit exceeded. Try again in {int(wait_time)}s")

                if retry_attempt < B4M_RATE_LIMIT_RETRIES:
                    print(f"⏱️ Rate limited - waiting {int(wait_time)}s before retry (attempt {retry_attempt}/{B4M_RATE_LIMIT_RETRIES})")
                    time.sleep(wait_time)
                    continue
                else:
                    print(f"❌ B4M API failed after {B4M_RATE_LIMIT_RETRIES} attempts - keeping conversation file for manual retry")
                    return None

            response.raise_for_status()
            initial_data = response.json()

            # Extract quest ID
            quest_id = extract_quest_id(initial_data)
            if not quest_id:
                print(f"⚠️ No quest ID in B4M response - cannot poll for completion")
                return None

            # Quest-based polling
            for poll_attempt in range(1, B4M_MAX_POLLING_ATTEMPTS + 1):
                if shutdown_event.is_set():
                    return None

                print(f"📡 Polling B4M quest status (attempt {poll_attempt}/{B4M_MAX_POLLING_ATTEMPTS})...")

                # Poll quest endpoint
                poll_response = requests.get(
                    f"https://app.bike4mind.com/api/sessions/{B4M_SESSION_ID}/chat/{quest_id}",
                    headers=headers,
                    timeout=5.0
                )

                if poll_response.status_code == 200:
                    quest_data = poll_response.json()

                    # Check quest status
                    if quest_data.get('status') == 'done':
                        print("✅ Quest complete - extracting AI response")
                        ai_response = extract_ai_response(quest_data)

                        if ai_response:
                            return ai_response
                        else:
                            print("⚠️ Quest done but no response found - check response extraction")
                            print(f"Debug: Quest data keys: {list(quest_data.keys())}")
                            return None

                    elif quest_data.get('status') == 'stopped':
                        print("⚠️ Quest was stopped or cancelled")
                        return None

                    elif quest_data.get('status') == 'running':
                        print(f"⏳ Quest still running, polling again in {B4M_POLLING_INTERVAL}s...")

                # Wait before next poll
                if poll_attempt < B4M_MAX_POLLING_ATTEMPTS:
                    time.sleep(B4M_POLLING_INTERVAL)

            # Polling timeout
            print(f"⚠️ B4M polling timeout after {B4M_MAX_POLLING_ATTEMPTS * B4M_POLLING_INTERVAL}s")
            return None

        except requests.exceptions.RequestException as e:
            print(f"⚠️ B4M API error (attempt {retry_attempt}/{B4M_RATE_LIMIT_RETRIES}): {e}")
            if retry_attempt < B4M_RATE_LIMIT_RETRIES:
                wait_time = 60 * (2 ** (retry_attempt - 1))  # Exponential backoff
                print(f"⏱️ Waiting {wait_time}s before retry...")
                time.sleep(wait_time)
            else:
                print(f"❌ B4M API failed after {B4M_RATE_LIMIT_RETRIES} attempts")
                return None

    return None


def process_conversation_queue():
    """Process oldest conversation file with B4M API (called by STT thread)"""
    # Use lock to prevent multiple threads from processing same file
    with conversation_queue_lock:
        conversation_file = find_oldest_file('conversation_*.txt')
        if not conversation_file:
            return

        # Skip conversation_test.txt
        if conversation_file == 'conversation_test.txt':
            return

    try:
        # Read conversation content
        with open(conversation_file, 'r', encoding='utf-8') as f:
            message_text = f.read().strip()

        if not message_text:
            print(f"⚠️ Empty conversation file: {conversation_file}")
            os.remove(conversation_file)
            return

        # Extract counter from filename
        import re
        match = re.search(r'__(\d+)\.txt$', conversation_file)
        file_counter = int(match.group(1)) if match else 0

        print(f"🤖 Processing conversation file with B4M AI...")

        # Send to B4M API (blocking but doesn't stop speech recognition due to threading)
        ai_response = send_to_b4m_api(message_text, file_counter)

        if ai_response:
            # Create response file with matching counter
            timestamp = datetime.now().strftime('%Y-%m-%d_%H-%M-%S')
            response_filename = f"response_{timestamp}__{file_counter:03d}.txt"

            with response_queue_lock:
                with open(response_filename, 'w', encoding='utf-8') as f:
                    f.write(ai_response)

            print(f"💾 AI response saved to {response_filename}")

            # Delete conversation file after successful processing
            os.remove(conversation_file)
            print(f"🗑️ Deleted {conversation_file} after processing")
        else:
            print(f"⚠️ B4M API failed for {conversation_file} - will retry")

    except Exception as e:
        print(f"❌ Error processing conversation queue: {e}")


def speech_recognition_thread(args):
    """Main thread: Speech recognition + B4M API processing"""
    print("🎤 Initializing Whisper base model...")

    # Load Whisper model
    try:
        if USING_FASTER_WHISPER:
            model = WhisperModel("base", device="cpu", compute_type="int8")
        else:
            model = whisper.load_model("base")
        print("✅ Whisper model loaded successfully")
    except Exception as e:
        print(f"❌ Failed to load Whisper model: {e}")
        shutdown_event.set()
        return

    # Initialize word buffer
    word_buffer = []
    last_phrase = ""  # For duplicate detection

    if args.test:
        # Test mode: Read from conversation_test.txt
        print("🧪 Test Mode: Reading from conversation_test.txt")
        try:
            with open('conversation_test.txt', 'r', encoding='utf-8') as f:
                test_sentences = [line.strip() for line in f if line.strip()]
            print(f"📄 Loaded {len(test_sentences)} test sentences")
        except FileNotFoundError:
            print("❌ conversation_test.txt not found")
            shutdown_event.set()
            return

        sentence_index = 0
        while not shutdown_event.is_set():
            # Get next sentence (cycle through)
            sentence = test_sentences[sentence_index % len(test_sentences)]
            sentence_index += 1

            print(f"🎤 Processing sentence {sentence_index}/{len(test_sentences)}: \"{sentence[:50]}...\"")

            # Add words to buffer
            words = sentence.split()
            word_buffer.extend(words)

            # Update display
            current_count = len(word_buffer)
            print(f"Buffer: {min(current_count, BUFFER_SIZE)}/{BUFFER_SIZE} words")

            # Check if buffer is full
            if len(word_buffer) >= BUFFER_SIZE:
                # Take first 20 words
                conversation_words = word_buffer[:BUFFER_SIZE]
                word_buffer = word_buffer[BUFFER_SIZE:]

                # Create conversation file
                timestamp = datetime.now().strftime('%Y-%m-%d_%H-%M-%S')
                counter = get_next_conversation_counter()
                conversation_filename = f"conversation_{timestamp}__{counter:03d}.txt"

                conversation_text = ' '.join(conversation_words)
                with open(conversation_filename, 'w', encoding='utf-8') as f:
                    f.write(conversation_text)

                print(f"💾 Conversation saved to {conversation_filename}")

                # Process with B4M API (non-blocking - run in separate thread)
                threading.Thread(target=process_conversation_queue, daemon=True).start()

            # Wait before next sentence
            time.sleep(TEST_MODE_INTERVAL)

    else:
        # Live microphone mode
        if not AUDIO_AVAILABLE:
            print("❌ sounddevice not available - cannot use live microphone mode")
            shutdown_event.set()
            return

        print("🎙️ Starting speech recognition...")
        print(f"Buffer: 0/{BUFFER_SIZE} words")

        while not shutdown_event.is_set():
            try:
                # Check if buffer should be cleared (after TTS)
                if clear_buffer_event.is_set():
                    word_buffer = []
                    last_phrase = ""
                    print(f"🗑️ Cleared speech buffer after TTS")
                    clear_buffer_event.clear()

                # Pause recording during TTS playback
                if tts_speaking_event.is_set():
                    time.sleep(0.1)
                    continue

                # Record audio in interruptible chunks
                audio_chunks = []
                chunks_needed = int(CHUNK_DURATION / MINI_CHUNK_DURATION)

                for _ in range(chunks_needed):
                    if shutdown_event.is_set() or tts_speaking_event.is_set():
                        break

                    # Record mini-chunk
                    audio_chunk = sd.rec(
                        int(MINI_CHUNK_DURATION * SAMPLE_RATE),
                        samplerate=SAMPLE_RATE,
                        channels=1,
                        dtype='float32'
                    )

                    # Wait for recording, but check for interruptions
                    start_time = time.time()
                    while sd.get_stream().active:
                        if tts_speaking_event.is_set() or shutdown_event.is_set():
                            sd.stop()
                            break
                        time.sleep(0.01)
                        # Timeout after expected duration + 1 second
                        if time.time() - start_time > MINI_CHUNK_DURATION + 1.0:
                            break

                    # Only append if recording completed successfully
                    if not tts_speaking_event.is_set() and not shutdown_event.is_set():
                        audio_chunks.append(audio_chunk)

                if shutdown_event.is_set():
                    break

                # Skip if no audio chunks collected (interrupted by TTS)
                if not audio_chunks:
                    continue

                # Combine chunks
                audio = np.concatenate(audio_chunks).flatten()

                # Transcribe
                if USING_FASTER_WHISPER:
                    segments, _ = model.transcribe(audio, language='en', beam_size=5)
                    text = ' '.join([segment.text for segment in segments]).strip()
                else:
                    result = model.transcribe(audio, language='en')
                    text = result['text'].strip()

                if not text or text == last_phrase:
                    continue

                last_phrase = text

                # Add words to buffer
                words = text.split()
                word_buffer.extend(words)

                # Update display
                current_count = len(word_buffer)
                print(f"Buffer: {min(current_count, BUFFER_SIZE)}/{BUFFER_SIZE} words")

                # Check if buffer is full
                if len(word_buffer) >= BUFFER_SIZE:
                    # Take first 20 words
                    conversation_words = word_buffer[:BUFFER_SIZE]
                    word_buffer = word_buffer[BUFFER_SIZE:]

                    # Create conversation file
                    timestamp = datetime.now().strftime('%Y-%m-%d_%H-%M-%S')
                    counter = get_next_conversation_counter()
                    conversation_filename = f"conversation_{timestamp}__{counter:03d}.txt"

                    conversation_text = ' '.join(conversation_words)
                    with open(conversation_filename, 'w', encoding='utf-8') as f:
                        f.write(conversation_text)

                    print(f"💾 Conversation saved to {conversation_filename}")

                    # Process with B4M API (non-blocking - run in separate thread)
                    threading.Thread(target=process_conversation_queue, daemon=True).start()

            except Exception as e:
                if not shutdown_event.is_set():
                    print(f"❌ [Speech Recognition Thread] Error: {e}")
                    time.sleep(1)

    # Shutdown: Save current buffer if non-empty
    if word_buffer:
        timestamp = datetime.now().strftime('%Y-%m-%d_%H-%M-%S')
        counter = get_next_conversation_counter()
        conversation_filename = f"conversation_{timestamp}__{counter:03d}.txt"

        conversation_text = ' '.join(word_buffer)
        with open(conversation_filename, 'w', encoding='utf-8') as f:
            f.write(conversation_text)

        print(f"💾 Saved remaining {len(word_buffer)} words to {conversation_filename}")


def initialize_piper_tts():
    """Initialize Piper TTS and test with Hello World"""
    try:
        # Try importing piper_tts
        from piper import PiperVoice

        if not PIPER_MODEL_PATH or not os.path.exists(PIPER_MODEL_PATH):
            print("⚠️ Piper voice model not found - TTS disabled")
            return None

        # Load voice model
        voice = PiperVoice.load(PIPER_MODEL_PATH)
        print("🔊 Piper TTS initialized")

        # Startup voice test
        test_message = "Hi!"
        print(f"🔊 Speaking: \"{test_message}\"")

        # Synthesize audio
        audio_data = []
        for audio_chunk in voice.synthesize(test_message):
            audio_data.extend(audio_chunk.audio_int16_array)

        # Play audio (wait to ensure initialization completes)
        audio_array = np.array(audio_data, dtype=np.int16)
        sd.play(audio_array, samplerate=voice.config.sample_rate)
        sd.wait()

        return voice

    except Exception as e:
        print(f"❌ Piper TTS initialization failed: {e}")
        print("⚠️ Application requires working TTS - exiting")
        return None


def speak_response_file(voice, response_file):
    """Speak contents of response file using Piper TTS"""
    try:
        with response_queue_lock:
            if not os.path.exists(response_file):
                return False

            with open(response_file, 'r', encoding='utf-8') as f:
                response_text = f.read().strip()

        if not response_text:
            return False

        print(f"🔊 Speaking AI response from {response_file}")

        # Set speaking flag to pause recording
        tts_speaking_event.set()

        # Give recording thread time to check the flag and pause
        time.sleep(0.2)

        # Stop any ongoing recording to avoid conflicts
        sd.stop()

        try:
            # Synthesize audio
            audio_data = []
            for audio_chunk in voice.synthesize(response_text):
                audio_data.extend(audio_chunk.audio_int16_array)

            # Play audio
            audio_array = np.array(audio_data, dtype=np.int16)
            sd.play(audio_array, samplerate=voice.config.sample_rate)
            sd.wait()  # Wait for playback to complete
        finally:
            # Clear speaking flag to resume recording
            tts_speaking_event.clear()

        # Delete response file after speaking
        with response_queue_lock:
            if os.path.exists(response_file):
                os.remove(response_file)
                print(f"💾 Cleared {response_file} after speaking")

        # Signal STT thread to clear the speech buffer
        clear_buffer_event.set()

        return True

    except Exception as e:
        print(f"❌ [TTS Thread] Error speaking response: {e}")
        return False


def tts_thread(args, voice):
    """Secondary thread: TTS output + trigger detection"""
    if args.interactive:
        print("🎙️ TTS Thread started (Interactive Mode)")
    else:
        print(f"🎙️ TTS Thread started (Keyword Trigger: '{TRIGGER_WORD}')")

    last_trigger_check = time.time()
    trigger_check_interval = 1.0  # Check for triggers every second

    # For interactive mode
    last_response_file_check = time.time()
    response_check_interval = 2.0  # Check for response files every 2 seconds

    while not shutdown_event.is_set():
        try:
            if args.interactive:
                # Interactive mode: Check for response files periodically
                current_time = time.time()
                if current_time - last_response_file_check >= response_check_interval:
                    last_response_file_check = current_time

                    # Find oldest response file
                    response_file = find_oldest_file('response_*.txt')
                    if response_file:
                        print("🤫 Speaking AI response")
                        speak_response_file(voice, response_file)

            else:
                # Default mode: Monitor for trigger word "Rosie" in conversation files
                # We'll check conversation files for trigger word
                current_time = time.time()
                if current_time - last_trigger_check >= trigger_check_interval:
                    last_trigger_check = current_time

                    # Check if trigger word appeared in recent transcriptions
                    # (This is a simplified implementation - ideally we'd monitor transcriptions from STT thread)
                    # For now, we'll check response file availability on trigger

                    # Note: In full implementation, STT thread should signal TTS thread when "Rosie" detected
                    # For this version, we'll use a simple file-based approach
                    pass

            time.sleep(0.5)

        except Exception as e:
            if not shutdown_event.is_set():
                print(f"❌ [TTS Thread] Error: {e}")
                time.sleep(1)


def main():
    """Main application entry point"""
    parser = argparse.ArgumentParser(description='HA_converse - Speech-to-Text with B4M AI')
    parser.add_argument('--test', action='store_true', help='Use conversation_test.txt instead of microphone')
    parser.add_argument('--interactive', action='store_true', help='Interactive mode (auto-speak responses)')
    args = parser.parse_args()

    # Register signal handlers
    signal.signal(signal.SIGINT, signal_handler)
    signal.signal(signal.SIGTERM, signal_handler)

    # Cleanup old files
    cleanup_old_files()

    # Initialize Piper TTS
    voice = initialize_piper_tts()
    if voice is None:
        sys.exit(1)

    # Start threads
    stt_thread = threading.Thread(target=speech_recognition_thread, args=(args,), daemon=True)
    tts_thread_obj = threading.Thread(target=tts_thread, args=(args, voice), daemon=True)

    stt_thread.start()
    time.sleep(1)  # Let STT thread initialize first
    tts_thread_obj.start()

    # Wait for shutdown
    try:
        while not shutdown_event.is_set():
            time.sleep(0.5)
    except KeyboardInterrupt:
        pass

    # Graceful shutdown
    print("\n⏳ Waiting for threads to complete...")
    stt_thread.join(timeout=5)
    tts_thread_obj.join(timeout=5)

    print("✅ Shutdown complete")


if __name__ == '__main__':
    main()
