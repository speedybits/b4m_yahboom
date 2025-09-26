#!/usr/bin/env python3
"""
Comprehensive unit tests for HA_converse speech-to-text application.

These tests are designed to detect implementation bugs including:
- Mocked or placeholder code
- Thread synchronization issues
- File management problems
- API integration failures
- Audio processing errors
- Signal handling issues

Test Strategy:
- Mock external dependencies (Whisper, Piper, B4M API)
- Test real behavior of core components
- Validate thread safety and coordination
- Check error handling and edge cases
- Verify complete workflows end-to-end
"""

import unittest
from unittest.mock import Mock, MagicMock, patch, mock_open, call
import threading
import time
import json
import os
import tempfile
import shutil
import signal
import queue
from datetime import datetime
import requests
import numpy as np

# Import the module under test (will be implemented)
# import ha_converse


class TestHAConverseCore(unittest.TestCase):
    """Test core HA_converse functionality"""

    def setUp(self):
        """Set up test environment"""
        self.test_dir = tempfile.mkdtemp()
        self.original_cwd = os.getcwd()
        os.chdir(self.test_dir)

        # Mock environment variables
        self.env_patcher = patch.dict(os.environ, {
            'B4M_API_KEY': 'test_api_key_12345',
            'B4M_ROSIE_ID': 'test_rosie_id_67890',
            'B4M_USER_ID': 'test_user_id_abcde',
            'PIPER_MODEL_PATH': '/fake/path/model.onnx',
            'PIPER_CONFIG_PATH': '/fake/path/config.json'
        })
        self.env_patcher.start()

    def tearDown(self):
        """Clean up test environment"""
        os.chdir(self.original_cwd)
        shutil.rmtree(self.test_dir, ignore_errors=True)
        self.env_patcher.stop()


class TestSpeechRecognition(TestHAConverseCore):
    """Test speech recognition and transcription functionality"""

    @patch('ha_converse.faster_whisper')
    @patch('ha_converse.sounddevice')
    def test_whisper_model_initialization(self, mock_sd, mock_whisper):
        """Test Whisper model loads correctly with proper configuration"""
        mock_model = Mock()
        mock_whisper.WhisperModel.return_value = mock_model

        # This should test actual model initialization, not mocks
        from ha_converse import HAConverse
        app = HAConverse()

        # Verify model is loaded with correct parameters
        mock_whisper.WhisperModel.assert_called_once_with(
            "base",
            device="cpu",
            compute_type="int8"
        )
        self.assertIsNotNone(app.whisper_model)

        # CRITICAL: Detect if implementation uses placeholder
        self.assertFalse(hasattr(app.whisper_model, '__placeholder__'),
                        "Whisper model appears to be mocked/placeholder")

    @patch('ha_converse.sounddevice')
    def test_audio_capture_real_processing(self, mock_sd):
        """Test audio capture processes real audio data, not simulated"""
        mock_sd.rec.return_value = np.random.random((16000, 1))  # 1 second of fake audio

        from ha_converse import HAConverse
        app = HAConverse()

        # Test audio chunk processing
        audio_chunk = app.capture_audio_chunk(duration=1.0)

        # Verify real audio processing
        self.assertIsInstance(audio_chunk, np.ndarray)
        self.assertEqual(len(audio_chunk.shape), 2)  # Should be 2D array

        # CRITICAL: Detect simulation/mocking
        mock_sd.rec.assert_called()
        self.assertGreater(len(audio_chunk), 0, "Audio chunk should not be empty")

    @patch('ha_converse.faster_whisper')
    def test_transcription_real_whisper_call(self, mock_whisper):
        """Test transcription calls real Whisper model, not simulation"""
        mock_model = Mock()
        mock_segments = [
            Mock(text="Hello world", start=0.0, end=2.0),
            Mock(text="This is a test", start=2.5, end=5.0)
        ]
        mock_model.transcribe.return_value = (mock_segments, Mock())
        mock_whisper.WhisperModel.return_value = mock_model

        from ha_converse import HAConverse
        app = HAConverse()

        # Test transcription with real audio data
        audio_data = np.random.random((16000,))  # 1 second of audio
        result = app.transcribe_audio(audio_data)

        # Verify real Whisper API call
        mock_model.transcribe.assert_called_once()
        args, kwargs = mock_model.transcribe.call_args

        # Check Whisper parameters
        self.assertIn('language', kwargs)
        self.assertEqual(kwargs['language'], 'en')
        self.assertIn('vad_filter', kwargs)

        # CRITICAL: Detect placeholder implementations
        self.assertNotEqual(result, "# TODO: Implement transcription",
                           "Transcription appears to be placeholder")
        self.assertIsInstance(result, str)

    def test_word_counting_accuracy(self):
        """Test word counting follows specification exactly"""
        from ha_converse import HAConverse
        app = HAConverse()

        test_cases = [
            ("hello world", 2),
            ("real-time processing", 2),  # Hyphenated = 1 word
            ("don't worry", 2),  # Contractions = 1 word
            ("number 123 test", 3),  # Numbers = 1 word
            ("", 0),  # Empty string
            ("   ", 0),  # Whitespace only
            ("word,with.punctuation!", 1),  # Punctuation attached
        ]

        for text, expected_count in test_cases:
            with self.subTest(text=text):
                count = app.count_words(text)
                self.assertEqual(count, expected_count,
                               f"Word count for '{text}' should be {expected_count}, got {count}")

    def test_buffer_management_20_word_limit(self):
        """Test buffer correctly manages 20-word limit"""
        from ha_converse import HAConverse
        app = HAConverse()

        # Add words to buffer
        words = ["word"] * 19
        for word in words:
            app.add_to_buffer(word)

        self.assertEqual(app.get_buffer_count(), 19)
        self.assertFalse(app.is_buffer_full())

        # Add 20th word - should trigger file creation
        with patch('ha_converse.open', mock_open()) as mock_file:
            app.add_to_buffer("final")

            self.assertEqual(app.get_buffer_count(), 20)
            self.assertTrue(app.is_buffer_full())

            # Process buffer should create file and reset
            app.process_full_buffer()
            self.assertEqual(app.get_buffer_count(), 0)

            # Verify file was created
            mock_file.assert_called()


class TestB4MAPIIntegration(TestHAConverseCore):
    """Test B4M API integration and quest polling system"""

    @patch('ha_converse.requests')
    def test_b4m_api_real_http_calls(self, mock_requests):
        """Test B4M API makes real HTTP requests, not simulated"""
        # Setup mock response for quest creation
        mock_create_response = Mock()
        mock_create_response.status_code = 200
        mock_create_response.json.return_value = {
            'questId': 'test_quest_123',
            'status': 'running'
        }

        # Setup mock response for quest polling
        mock_poll_response = Mock()
        mock_poll_response.status_code = 200
        mock_poll_response.json.return_value = {
            'status': 'done',
            'replies': ['Test AI response from B4M']
        }

        mock_requests.post.return_value = mock_create_response
        mock_requests.get.return_value = mock_poll_response

        from ha_converse import HAConverse
        app = HAConverse(enable_b4m=True)

        # Test API call
        conversation_text = "Test conversation for B4M processing"
        response = app.send_to_b4m(conversation_text)

        # Verify real HTTP calls
        mock_requests.post.assert_called_once()
        call_args = mock_requests.post.call_args

        # Check API endpoint
        self.assertEqual(call_args[0][0], 'https://app.bike4mind.com/api/ai/llm')

        # Check headers
        headers = call_args[1]['headers']
        self.assertEqual(headers['X-API-Key'], 'test_api_key_12345')
        self.assertEqual(headers['Content-Type'], 'application/json')

        # Check request payload
        payload = json.loads(call_args[1]['data'])
        self.assertEqual(payload['sessionId'], 'test_rosie_id_67890')
        self.assertEqual(payload['message'], conversation_text)
        self.assertEqual(payload['historyCount'], 10)

        # CRITICAL: Detect mocked/simulated API
        self.assertNotEqual(response, "# Mock B4M response",
                           "B4M API appears to be mocked/simulated")

    @patch('ha_converse.requests')
    def test_b4m_quest_polling_system(self, mock_requests):
        """Test quest polling follows specification exactly"""
        # Mock quest creation
        mock_create_response = Mock()
        mock_create_response.status_code = 200
        mock_create_response.json.return_value = {
            'questId': 'quest_456',
            'status': 'running'
        }

        # Mock polling responses - running, then done
        mock_poll_running = Mock()
        mock_poll_running.status_code = 200
        mock_poll_running.json.return_value = {'status': 'running'}

        mock_poll_done = Mock()
        mock_poll_done.status_code = 200
        mock_poll_done.json.return_value = {
            'status': 'done',
            'replies': ['Final AI response']
        }

        mock_requests.post.return_value = mock_create_response
        mock_requests.get.side_effect = [mock_poll_running, mock_poll_done]

        from ha_converse import HAConverse
        app = HAConverse(enable_b4m=True)

        start_time = time.time()
        response = app.send_to_b4m("Test message")
        elapsed_time = time.time() - start_time

        # Verify polling behavior
        self.assertEqual(mock_requests.get.call_count, 2)

        # Check polling endpoint format
        poll_calls = [call for call in mock_requests.get.call_args_list]
        for call in poll_calls:
            url = call[0][0]
            self.assertIn('/sessions/test_rosie_id_67890/chat/quest_456', url)

        # Verify timing (should be at least 7 seconds for one poll interval)
        self.assertGreaterEqual(elapsed_time, 7.0, "Polling should respect 7-second intervals")

    @patch('ha_converse.requests')
    def test_b4m_rate_limiting_handling(self, mock_requests):
        """Test B4M rate limiting follows exponential backoff"""
        # Mock 429 rate limit response
        mock_429_response = Mock()
        mock_429_response.status_code = 429
        mock_429_response.text = "Try again in 59.165 seconds"

        # Mock successful response after rate limit
        mock_success_response = Mock()
        mock_success_response.status_code = 200
        mock_success_response.json.return_value = {
            'questId': 'quest_789',
            'status': 'running'
        }

        mock_requests.post.side_effect = [mock_429_response, mock_success_response]

        from ha_converse import HAConverse
        app = HAConverse(enable_b4m=True)

        # Mock time.sleep to speed up test
        with patch('ha_converse.time.sleep') as mock_sleep:
            start_time = time.time()
            app.send_to_b4m("Test rate limit handling")

            # Verify rate limit wait was called
            mock_sleep.assert_called()
            sleep_duration = mock_sleep.call_args[0][0]
            self.assertAlmostEqual(sleep_duration, 59.165, places=2)

    def test_b4m_response_extraction_fallbacks(self):
        """Test B4M response extraction uses all fallback methods"""
        from ha_converse import HAConverse
        app = HAConverse(enable_b4m=True)

        test_cases = [
            # Primary: replies array
            {'replies': ['Primary response']},
            # Fallback 1: reply field
            {'reply': 'Fallback response 1'},
            # Fallback 2: questMasterReply
            {'questMasterReply': 'Fallback response 2'},
            # Fallback 3: researchModeResults
            {'researchModeResults': 'Fallback response 3'},
            # Fallback 4: messages array
            {'messages': [{'content': 'Fallback response 4'}]}
        ]

        expected_responses = [
            'Primary response',
            'Fallback response 1',
            'Fallback response 2',
            'Fallback response 3',
            'Fallback response 4'
        ]

        for i, (response_data, expected) in enumerate(zip(test_cases, expected_responses)):
            with self.subTest(case=i):
                extracted = app.extract_b4m_response(response_data)
                self.assertEqual(extracted, expected)

    @patch('ha_converse.requests')
    def test_b4m_quest_timeout_handling(self, mock_requests):
        """Test quest polling times out after 15 attempts (105 seconds)"""
        # Mock quest creation
        mock_create_response = Mock()
        mock_create_response.status_code = 200
        mock_create_response.json.return_value = {
            'questId': 'timeout_quest',
            'status': 'running'
        }

        # Mock polling - always returns running status
        mock_poll_response = Mock()
        mock_poll_response.status_code = 200
        mock_poll_response.json.return_value = {'status': 'running'}

        mock_requests.post.return_value = mock_create_response
        mock_requests.get.return_value = mock_poll_response

        from ha_converse import HAConverse
        app = HAConverse(enable_b4m=True)

        # Mock time.sleep to speed up test
        with patch('ha_converse.time.sleep'):
            result = app.send_to_b4m("Test timeout")

            # Should make exactly 15 polling attempts
            self.assertEqual(mock_requests.get.call_count, 15)
            self.assertIsNone(result)  # Should return None on timeout


class TestPiperTTSIntegration(TestHAConverseCore):
    """Test Piper TTS integration and audio output"""

    @patch('ha_converse.piper')
    @patch('ha_converse.sounddevice')
    def test_piper_initialization_real_model(self, mock_sd, mock_piper):
        """Test Piper TTS loads real voice model, not simulation"""
        mock_voice = Mock()
        mock_piper.PiperVoice.load_model.return_value = mock_voice

        from ha_converse import HAConverse
        app = HAConverse(enable_piper=True)

        # Verify model loading
        mock_piper.PiperVoice.load_model.assert_called_once_with(
            '/fake/path/model.onnx',
            '/fake/path/config.json'
        )

        # CRITICAL: Detect simulation mode
        self.assertFalse(hasattr(app.piper_voice, '__simulation__'),
                        "Piper TTS appears to be in simulation mode")

    @patch('ha_converse.piper')
    @patch('ha_converse.sounddevice')
    def test_piper_audio_synthesis_real_output(self, mock_sd, mock_piper):
        """Test Piper synthesizes real audio, not placeholder"""
        mock_voice = Mock()
        mock_audio_data = np.random.random(22050)  # 1 second at 22kHz
        mock_voice.synthesize.return_value = mock_audio_data
        mock_piper.PiperVoice.load_model.return_value = mock_voice

        from ha_converse import HAConverse
        app = HAConverse(enable_piper=True)

        # Test synthesis
        audio_output = app.synthesize_speech("Test speech synthesis")

        # Verify real synthesis call
        mock_voice.synthesize.assert_called_once_with("Test speech synthesis")
        self.assertIsInstance(audio_output, np.ndarray)

        # CRITICAL: Detect placeholder audio
        self.assertGreater(len(audio_output), 0, "Audio output should not be empty")
        self.assertNotEqual(audio_output.tolist(), [0] * len(audio_output),
                           "Audio appears to be silence/placeholder")

    @patch('ha_converse.sounddevice')
    def test_piper_startup_voice_test(self, mock_sd):
        """Test startup voice test speaks 'Hello World' message"""
        from ha_converse import HAConverse

        with patch.object(HAConverse, 'synthesize_speech') as mock_synth:
            mock_synth.return_value = np.random.random(22050)
            app = HAConverse(enable_piper=True)
            app.startup_voice_test()

            # Verify startup message
            mock_synth.assert_called_with(
                "Hello World! Piper text-to-speech is working correctly."
            )

            # Verify audio playback
            mock_sd.play.assert_called()

    def test_piper_file_cleanup_after_playback(self):
        """Test response files are deleted after TTS playback"""
        from ha_converse import HAConverse
        app = HAConverse(enable_piper=True)

        # Create test response file
        response_file = "response_2024-01-15_14-30-20__001.txt"
        with open(response_file, 'w') as f:
            f.write("Test response for TTS")

        self.assertTrue(os.path.exists(response_file))

        # Mock TTS synthesis and playback
        with patch.object(app, 'synthesize_speech') as mock_synth:
            with patch.object(app, 'play_audio') as mock_play:
                mock_synth.return_value = np.random.random(22050)
                app.speak_response_file(response_file)

                # Verify file was deleted
                self.assertFalse(os.path.exists(response_file))


class TestThreadManagement(TestHAConverseCore):
    """Test thread safety and coordination"""

    def test_thread_safe_voice_activity_flag(self):
        """Test voice activity flag is thread-safe"""
        from ha_converse import HAConverse
        app = HAConverse()

        # Test concurrent access to voice activity flag
        results = []
        errors = []

        def set_voice_activity():
            try:
                for i in range(100):
                    app.set_voice_activity(True)
                    app.set_voice_activity(False)
                results.append(True)
            except Exception as e:
                errors.append(e)

        def check_voice_activity():
            try:
                for i in range(100):
                    app.is_voice_active()
                results.append(True)
            except Exception as e:
                errors.append(e)

        threads = []
        for _ in range(5):
            threads.append(threading.Thread(target=set_voice_activity))
            threads.append(threading.Thread(target=check_voice_activity))

        for thread in threads:
            thread.start()

        for thread in threads:
            thread.join(timeout=5.0)

        # No errors should occur
        self.assertEqual(len(errors), 0, f"Thread safety errors: {errors}")
        self.assertEqual(len(results), 10)  # All threads completed

    def test_shutdown_event_coordination(self):
        """Test shutdown event coordinates all threads"""
        from ha_converse import HAConverse
        app = HAConverse()

        # Start threads
        speech_thread = threading.Thread(target=app.run_speech_recognition)
        tts_thread = threading.Thread(target=app.run_tts_handler)

        speech_thread.daemon = True
        tts_thread.daemon = True

        speech_thread.start()
        tts_thread.start()

        # Give threads time to start
        time.sleep(0.5)

        # Signal shutdown
        app.shutdown_event.set()

        # Verify threads stop within reasonable time
        speech_thread.join(timeout=2.0)
        tts_thread.join(timeout=2.0)

        self.assertFalse(speech_thread.is_alive(), "Speech thread should stop on shutdown")
        self.assertFalse(tts_thread.is_alive(), "TTS thread should stop on shutdown")

    def test_interrupt_response_time(self):
        """Test voice interruption has sub-100ms response time"""
        from ha_converse import HAConverse
        app = HAConverse()

        # Mock audio playback
        with patch('ha_converse.sounddevice.play') as mock_play:
            mock_stream = Mock()
            mock_stream.finished = False
            mock_play.return_value = mock_stream

            # Start TTS playback
            audio_data = np.random.random(22050 * 5)  # 5 seconds of audio
            start_time = time.time()

            # Simulate voice detection after 50ms
            def simulate_voice_detection():
                time.sleep(0.05)  # 50ms delay
                app.set_voice_activity(True)

            voice_thread = threading.Thread(target=simulate_voice_detection)
            voice_thread.start()

            # Play audio with interruption checking
            app.play_audio_interruptible(audio_data)

            response_time = time.time() - start_time
            voice_thread.join()

            # Should stop within 100ms of voice detection
            self.assertLess(response_time, 0.15, f"Interrupt response too slow: {response_time}s")

    def test_file_operations_thread_safety(self):
        """Test file operations are thread-safe"""
        from ha_converse import HAConverse
        app = HAConverse()

        errors = []
        created_files = []

        def create_conversation_file(thread_id):
            try:
                filename = app.create_conversation_file(f"Thread {thread_id} test content")
                created_files.append(filename)
            except Exception as e:
                errors.append(e)

        def create_response_file(thread_id):
            try:
                filename = app.create_response_file(f"Thread {thread_id} response", thread_id)
                created_files.append(filename)
            except Exception as e:
                errors.append(e)

        threads = []
        for i in range(10):
            threads.append(threading.Thread(target=create_conversation_file, args=(i,)))
            threads.append(threading.Thread(target=create_response_file, args=(i,)))

        for thread in threads:
            thread.start()

        for thread in threads:
            thread.join(timeout=2.0)

        # Cleanup
        for filename in created_files:
            if os.path.exists(filename):
                os.remove(filename)

        # No file operation errors
        self.assertEqual(len(errors), 0, f"File operation errors: {errors}")


class TestFileManagement(TestHAConverseCore):
    """Test file creation, queue management, and cleanup"""

    def test_conversation_file_naming_format(self):
        """Test conversation files use correct timestamp format"""
        from ha_converse import HAConverse
        app = HAConverse()

        # Mock datetime to control timestamp
        fixed_time = datetime(2024, 1, 15, 14, 30, 25)
        with patch('ha_converse.datetime') as mock_dt:
            mock_dt.now.return_value = fixed_time
            mock_dt.strftime = datetime.strftime  # Keep real strftime

            filename = app.create_conversation_file("Test content", counter=5)

            expected_pattern = "conversation_2024-01-15_14-30-25__005.txt"
            self.assertEqual(filename, expected_pattern)

    def test_response_file_1to1_mapping(self):
        """Test response files maintain 1:1 mapping with conversation files"""
        from ha_converse import HAConverse
        app = HAConverse()

        # Create conversation file with counter 7
        conv_file = app.create_conversation_file("Conversation content", counter=7)

        # Create corresponding response file
        resp_file = app.create_response_file("Response content", counter=7)

        # Extract counters from filenames
        conv_counter = int(conv_file.split('__')[1].split('.')[0])
        resp_counter = int(resp_file.split('__')[1].split('.')[0])

        self.assertEqual(conv_counter, resp_counter, "File counters must match for 1:1 mapping")

    def test_file_queue_fifo_processing(self):
        """Test files are processed in FIFO order (oldest first)"""
        from ha_converse import HAConverse
        app = HAConverse()

        # Create multiple files with different timestamps
        files_created = []
        for i in range(5):
            # Use different timestamps
            timestamp = datetime(2024, 1, 15, 14, 30, i*10)  # 10-second intervals
            with patch('ha_converse.datetime') as mock_dt:
                mock_dt.now.return_value = timestamp
                filename = app.create_conversation_file(f"Content {i}", counter=i+1)
                files_created.append(filename)
            time.sleep(0.01)  # Ensure file creation time differences

        # Get oldest file
        oldest_file = app.get_oldest_conversation_file()

        # Should be the first file created
        self.assertEqual(oldest_file, files_created[0])

        # Cleanup
        for filename in files_created:
            if os.path.exists(filename):
                os.remove(filename)

    def test_startup_file_cleanup(self):
        """Test startup deletes all existing conversation and response files"""
        from ha_converse import HAConverse

        # Create some existing files
        existing_files = [
            "conversation_2024-01-14_10-20-30__001.txt",
            "conversation_2024-01-14_10-25-45__002.txt",
            "response_2024-01-14_10-20-35__001.txt",
            "response_2024-01-14_10-25-50__002.txt",
            "other_file.txt"  # Should not be deleted
        ]

        for filename in existing_files:
            with open(filename, 'w') as f:
                f.write("Existing content")

        # Initialize app (should trigger cleanup)
        app = HAConverse()
        app.startup_cleanup()

        # Check cleanup results
        for filename in existing_files:
            if filename.startswith(('conversation_', 'response_')):
                self.assertFalse(os.path.exists(filename),
                               f"File {filename} should be deleted on startup")
            else:
                self.assertTrue(os.path.exists(filename),
                              f"File {filename} should not be deleted on startup")

        # Cleanup remaining file
        if os.path.exists("other_file.txt"):
            os.remove("other_file.txt")

    def test_file_utf8_encoding(self):
        """Test files use UTF-8 encoding for international characters"""
        from ha_converse import HAConverse
        app = HAConverse()

        # Test content with international characters
        test_content = "Hello 世界 café naïve résumé"

        filename = app.create_conversation_file(test_content)

        # Read file back and verify encoding
        with open(filename, 'r', encoding='utf-8') as f:
            read_content = f.read().strip()

        self.assertEqual(read_content, test_content)

        # Cleanup
        if os.path.exists(filename):
            os.remove(filename)


class TestTriggerDetection(TestHAConverseCore):
    """Test trigger detection for both keyword and interactive modes"""

    def test_keyword_trigger_case_insensitive(self):
        """Test 'Rosie' keyword detection is case-insensitive"""
        from ha_converse import HAConverse
        app = HAConverse()

        test_cases = [
            ("Hey Rosie how are you", True),
            ("rosie please help", True),
            ("ROSIE ARE YOU THERE", True),
            ("RoSiE can you hear me", True),
            ("This is about roses", False),  # Similar but not exact
            ("Rosie's birthday", True),  # With punctuation
            ("Call me maybe", False),  # No trigger word
        ]

        for text, should_trigger in test_cases:
            with self.subTest(text=text):
                result = app.detect_keyword_trigger(text)
                self.assertEqual(result, should_trigger,
                               f"'{text}' trigger detection failed")

    def test_interactive_mode_silence_detection(self):
        """Test interactive mode detects 3 seconds of silence"""
        from ha_converse import HAConverse
        app = HAConverse(interactive_mode=True)

        # Simulate speech activity
        app.update_last_speech_time()
        start_time = time.time()

        # Check silence detection over time
        time.sleep(1.0)
        self.assertFalse(app.check_silence_trigger(), "Should not trigger after 1 second")

        time.sleep(1.5)
        self.assertFalse(app.check_silence_trigger(), "Should not trigger after 2.5 seconds")

        time.sleep(1.0)
        self.assertTrue(app.check_silence_trigger(), "Should trigger after 3.5 seconds")

    def test_interactive_mode_timer_display(self):
        """Test interactive mode shows timer after 0.5s of silence"""
        from ha_converse import HAConverse
        app = HAConverse(interactive_mode=True)

        app.update_last_speech_time()

        # Check timer display timing
        time.sleep(0.3)
        self.assertFalse(app.should_show_timer(), "Timer should not show before 0.5s")

        time.sleep(0.3)
        self.assertTrue(app.should_show_timer(), "Timer should show after 0.5s")

    def test_interactive_mode_prevents_repeated_triggers(self):
        """Test interactive mode prevents repeated triggers until speech reset"""
        from ha_converse import HAConverse
        app = HAConverse(interactive_mode=True)

        # First trigger
        app.update_last_speech_time()
        time.sleep(3.1)
        first_trigger = app.check_silence_trigger()
        self.assertTrue(first_trigger, "First trigger should activate")

        # Immediate second check - should not trigger again
        second_trigger = app.check_silence_trigger()
        self.assertFalse(second_trigger, "Should not trigger repeatedly")

        # Reset with speech detection
        app.update_last_speech_time()
        time.sleep(3.1)
        third_trigger = app.check_silence_trigger()
        self.assertTrue(third_trigger, "Should trigger again after speech reset")

    def test_trigger_mode_exclusivity(self):
        """Test keyword and interactive modes are mutually exclusive"""
        from ha_converse import HAConverse

        # Default mode should use keyword trigger
        app_default = HAConverse()
        self.assertTrue(app_default.keyword_mode_enabled)
        self.assertFalse(app_default.interactive_mode_enabled)

        # Interactive mode should disable keyword trigger
        app_interactive = HAConverse(interactive_mode=True)
        self.assertFalse(app_interactive.keyword_mode_enabled)
        self.assertTrue(app_interactive.interactive_mode_enabled)


class TestErrorHandling(TestHAConverseCore):
    """Test error handling and edge cases"""

    @patch('ha_converse.sounddevice')
    def test_microphone_unavailable_graceful_handling(self, mock_sd):
        """Test graceful handling when microphone is unavailable"""
        mock_sd.rec.side_effect = OSError("Microphone not available")

        from ha_converse import HAConverse

        # Should not crash on microphone error
        try:
            app = HAConverse()
            audio_chunk = app.capture_audio_chunk()
            # Should return None or empty array
            self.assertIsNone(audio_chunk)
        except OSError:
            self.fail("Should handle microphone errors gracefully")

    @patch('ha_converse.faster_whisper')
    def test_whisper_model_loading_failure(self, mock_whisper):
        """Test handling of Whisper model loading failures"""
        mock_whisper.WhisperModel.side_effect = Exception("Model loading failed")

        from ha_converse import HAConverse

        with self.assertRaises(Exception) as context:
            app = HAConverse()

        self.assertIn("Model loading failed", str(context.exception))

    def test_transcription_failure_inaudible_placeholder(self):
        """Test transcription failures insert [inaudible] placeholder"""
        from ha_converse import HAConverse
        app = HAConverse()

        # Mock transcription failure
        with patch.object(app, 'whisper_model') as mock_model:
            mock_model.transcribe.side_effect = Exception("Transcription failed")

            result = app.transcribe_audio(np.random.random(16000))

            self.assertEqual(result, "[inaudible]")

    @patch('ha_converse.requests')
    def test_b4m_api_network_failure_handling(self, mock_requests):
        """Test B4M API handles network failures gracefully"""
        mock_requests.post.side_effect = requests.ConnectionError("Network error")

        from ha_converse import HAConverse
        app = HAConverse(enable_b4m=True)

        # Should return None and not crash
        result = app.send_to_b4m("Test message")
        self.assertIsNone(result)

    def test_disk_full_file_creation_handling(self):
        """Test handling of disk full scenarios during file creation"""
        from ha_converse import HAConverse
        app = HAConverse()

        # Mock disk full error
        with patch('builtins.open', side_effect=OSError("No space left on device")):
            result = app.create_conversation_file("Test content")
            self.assertIsNone(result)

    def test_signal_handler_registration(self):
        """Test SIGINT handler is properly registered"""
        from ha_converse import HAConverse

        original_handler = signal.signal(signal.SIGINT, signal.SIG_DFL)

        try:
            app = HAConverse()
            app.setup_signal_handlers()

            # Check that handler was changed
            current_handler = signal.signal(signal.SIGINT, signal.SIG_DFL)
            self.assertNotEqual(current_handler, signal.SIG_DFL)

        finally:
            # Restore original handler
            signal.signal(signal.SIGINT, original_handler)

    def test_ctrl_c_shutdown_response_time(self):
        """Test Ctrl+C shutdown responds within 100ms"""
        from ha_converse import HAConverse
        app = HAConverse()

        # Start app in separate thread
        app_thread = threading.Thread(target=app.run)
        app_thread.daemon = True
        app_thread.start()

        time.sleep(0.5)  # Let app start

        # Send shutdown signal
        start_time = time.time()
        app.shutdown_event.set()

        # Wait for shutdown
        app_thread.join(timeout=1.0)
        response_time = time.time() - start_time

        self.assertLess(response_time, 0.1, f"Shutdown response too slow: {response_time}s")
        self.assertFalse(app_thread.is_alive(), "App thread should stop on shutdown")


class TestTestModeImplementation(TestHAConverseCore):
    """Test the test mode functionality"""

    def test_test_mode_file_reading(self):
        """Test test mode reads from conversation_test.txt"""
        # Create test file
        test_sentences = [
            "This is the first test sentence.",
            "Here is another sentence for testing.",
            "The third sentence contains the word Rosie.",
            "Final test sentence with some content."
        ]

        with open("conversation_test.txt", "w") as f:
            f.write("\n".join(test_sentences))

        from ha_converse import HAConverse
        app = HAConverse(test_mode=True)

        # Load test sentences
        sentences = app.load_test_sentences()
        self.assertEqual(len(sentences), 4)
        self.assertEqual(sentences[0], test_sentences[0])

        # Cleanup
        os.remove("conversation_test.txt")

    def test_test_mode_sentence_cycling(self):
        """Test test mode cycles through sentences and restarts"""
        test_sentences = ["Sentence 1", "Sentence 2", "Sentence 3"]

        with open("conversation_test.txt", "w") as f:
            f.write("\n".join(test_sentences))

        from ha_converse import HAConverse
        app = HAConverse(test_mode=True)

        # Read sentences in order
        self.assertEqual(app.get_next_test_sentence(), "Sentence 1")
        self.assertEqual(app.get_next_test_sentence(), "Sentence 2")
        self.assertEqual(app.get_next_test_sentence(), "Sentence 3")

        # Should cycle back to beginning
        self.assertEqual(app.get_next_test_sentence(), "Sentence 1")

        # Cleanup
        os.remove("conversation_test.txt")

    def test_test_mode_sentence_interval_timing(self):
        """Test test mode processes sentences at correct intervals"""
        with open("conversation_test.txt", "w") as f:
            f.write("Test sentence 1\nTest sentence 2")

        from ha_converse import HAConverse
        app = HAConverse(test_mode=True, sentence_interval=0.5)  # Fast for testing

        start_time = time.time()

        # Process two sentences
        sentence1 = app.get_next_test_sentence()
        time.sleep(app.sentence_interval)
        sentence2 = app.get_next_test_sentence()

        elapsed = time.time() - start_time

        # Should take approximately sentence_interval time
        self.assertGreater(elapsed, 0.4)  # At least 0.4 seconds
        self.assertLess(elapsed, 1.0)     # But less than 1 second

        # Cleanup
        os.remove("conversation_test.txt")

    def test_test_mode_maintains_buffer_logic(self):
        """Test test mode uses same 20-word buffer logic"""
        # Create test sentences that will fill buffer
        test_sentences = [
            "word " * 10,  # 10 words
            "word " * 8,   # 8 words
            "word " * 5    # 5 words - should trigger at 18+5=23 total, creating file at 20
        ]

        with open("conversation_test.txt", "w") as f:
            f.write("\n".join(test_sentences))

        from ha_converse import HAConverse
        app = HAConverse(test_mode=True)

        # Process sentences and check buffer
        app.process_test_sentence(test_sentences[0])  # 10 words
        self.assertEqual(app.get_buffer_count(), 10)

        app.process_test_sentence(test_sentences[1])  # +8 = 18 words
        self.assertEqual(app.get_buffer_count(), 18)

        # Mock file creation
        with patch.object(app, 'create_conversation_file') as mock_create:
            app.process_test_sentence(test_sentences[2])  # +5 = 23 words, triggers at 20
            mock_create.assert_called_once()

        # Cleanup
        os.remove("conversation_test.txt")


class TestIntegrationWorkflows(TestHAConverseCore):
    """Test complete end-to-end workflows"""

    @patch('ha_converse.sounddevice')
    @patch('ha_converse.faster_whisper')
    @patch('ha_converse.requests')
    @patch('ha_converse.piper')
    def test_complete_keyword_workflow(self, mock_piper, mock_requests, mock_whisper, mock_sd):
        """Test complete workflow: speech -> B4M -> keyword trigger -> TTS"""
        # Setup mocks
        mock_model = Mock()
        mock_segments = [Mock(text="Hey Rosie please help me", start=0.0, end=3.0)]
        mock_model.transcribe.return_value = (mock_segments, Mock())
        mock_whisper.WhisperModel.return_value = mock_model

        # Setup B4M response
        mock_b4m_response = Mock()
        mock_b4m_response.status_code = 200
        mock_b4m_response.json.return_value = {
            'questId': 'test_quest',
            'status': 'done',
            'replies': ['I can help you with that!']
        }
        mock_requests.post.return_value = mock_b4m_response
        mock_requests.get.return_value = mock_b4m_response

        # Setup Piper TTS
        mock_voice = Mock()
        mock_voice.synthesize.return_value = np.random.random(22050)
        mock_piper.PiperVoice.load_model.return_value = mock_voice

        from ha_converse import HAConverse
        app = HAConverse(enable_b4m=True, enable_piper=True)

        # Simulate workflow
        transcription = "Hey Rosie please help me " + "word " * 15  # 20 words total

        # Add to buffer (should trigger B4M)
        words = transcription.split()
        for word in words:
            app.add_to_buffer(word)

        # Process buffer when full
        self.assertTrue(app.is_buffer_full())
        app.process_full_buffer()

        # Check B4M was called
        mock_requests.post.assert_called_once()

        # Check response file created
        response_files = [f for f in os.listdir('.') if f.startswith('response_')]
        self.assertEqual(len(response_files), 1)

        # Trigger keyword detection
        trigger_detected = app.detect_keyword_trigger(transcription)
        self.assertTrue(trigger_detected)

        # Process TTS trigger
        app.process_tts_trigger()

        # Verify TTS synthesis
        mock_voice.synthesize.assert_called()

        # Verify response file cleanup
        response_files_after = [f for f in os.listdir('.') if f.startswith('response_')]
        self.assertEqual(len(response_files_after), 0)

    @patch('ha_converse.time')
    def test_complete_interactive_workflow(self, mock_time):
        """Test complete workflow with interactive mode (silence trigger)"""
        from ha_converse import HAConverse
        app = HAConverse(interactive_mode=True, enable_piper=True)

        # Mock speech detection and silence timing
        app.update_last_speech_time()

        # Simulate silence detection
        mock_time.time.side_effect = [
            100.0,  # Initial time
            103.5   # 3.5 seconds later
        ]

        # Create mock response file
        response_file = "response_2024-01-15_14-30-20__001.txt"
        with open(response_file, 'w') as f:
            f.write("Test response for interactive mode")

        # Check silence trigger
        trigger_detected = app.check_silence_trigger()
        self.assertTrue(trigger_detected)

        # Mock TTS processing
        with patch.object(app, 'speak_response_file') as mock_speak:
            app.process_tts_trigger()
            mock_speak.assert_called_once()

        # Verify file cleanup
        self.assertFalse(os.path.exists(response_file))

    def test_conversation_response_file_mapping(self):
        """Test 1:1 mapping between conversation and response files"""
        from ha_converse import HAConverse
        app = HAConverse()

        # Create multiple conversation files
        conv_files = []
        for i in range(3):
            filename = app.create_conversation_file(f"Conversation {i+1}", counter=i+1)
            conv_files.append(filename)

        # Create corresponding response files
        resp_files = []
        for i in range(3):
            filename = app.create_response_file(f"Response {i+1}", counter=i+1)
            resp_files.append(filename)

        # Verify mapping
        for i in range(3):
            conv_counter = int(conv_files[i].split('__')[1].split('.')[0])
            resp_counter = int(resp_files[i].split('__')[1].split('.')[0])
            self.assertEqual(conv_counter, resp_counter,
                           f"File mapping broken for index {i}")

        # Cleanup
        for filename in conv_files + resp_files:
            if os.path.exists(filename):
                os.remove(filename)


class TestCriticalBugDetection(TestHAConverseCore):
    """Special tests designed to catch the inserted bug"""

    def test_no_placeholder_code_detection(self):
        """CRITICAL: Detect any placeholder or mocked code in implementation"""
        from ha_converse import HAConverse
        app = HAConverse()

        # Check for common placeholder patterns
        source_code = inspect.getsource(HAConverse)

        placeholder_patterns = [
            "# TODO:",
            "# FIXME:",
            "# PLACEHOLDER",
            "# Mock implementation",
            "# Simulated",
            "pass  # ",
            "return None  # placeholder",
            "__mock__",
            "__placeholder__",
            "fake_",
            "mock_",
            "simulate_"
        ]

        for pattern in placeholder_patterns:
            self.assertNotIn(pattern, source_code,
                           f"Placeholder code detected: {pattern}")

    def test_real_api_calls_not_mocked(self):
        """CRITICAL: Ensure B4M API calls are real, not mocked"""
        from ha_converse import HAConverse
        app = HAConverse(enable_b4m=True)

        # Check that requests module is actually used
        import requests
        self.assertTrue(hasattr(app, 'send_to_b4m'))

        # Verify method doesn't return hardcoded responses
        with patch('ha_converse.requests.post') as mock_post:
            mock_response = Mock()
            mock_response.status_code = 200
            mock_response.json.return_value = {'questId': 'real_quest', 'status': 'done', 'replies': ['real response']}
            mock_post.return_value = mock_response

            result = app.send_to_b4m("test message")

            # Should actually call requests.post
            mock_post.assert_called_once()
            self.assertNotEqual(result, "mock response")

    def test_audio_processing_not_simulated(self):
        """CRITICAL: Ensure audio processing is real, not simulated"""
        from ha_converse import HAConverse
        app = HAConverse()

        # Check Whisper model is actually used
        self.assertTrue(hasattr(app, 'transcribe_audio'))

        # Verify transcription doesn't return hardcoded text
        with patch.object(app, 'whisper_model') as mock_model:
            mock_segments = [Mock(text="real transcription", start=0.0, end=2.0)]
            mock_model.transcribe.return_value = (mock_segments, Mock())

            result = app.transcribe_audio(np.random.random(16000))

            # Should call actual Whisper model
            mock_model.transcribe.assert_called_once()
            self.assertNotEqual(result, "simulated transcription")

    def test_thread_shutdown_not_ignored(self):
        """CRITICAL: Ensure shutdown event is actually checked in threads"""
        from ha_converse import HAConverse
        app = HAConverse()

        # Start thread
        test_thread = threading.Thread(target=app.run_speech_recognition)
        test_thread.daemon = True
        test_thread.start()

        time.sleep(0.1)  # Let thread start

        # Signal shutdown
        app.shutdown_event.set()

        # Thread should stop quickly
        test_thread.join(timeout=1.0)
        self.assertFalse(test_thread.is_alive(),
                        "Thread ignored shutdown event - potential infinite loop bug")

    def test_file_operations_atomic_not_partial(self):
        """CRITICAL: Ensure file operations are atomic, not partial writes"""
        from ha_converse import HAConverse
        app = HAConverse()

        test_content = "This is a complete test message that should be written atomically"

        # Create file
        filename = app.create_conversation_file(test_content)

        # Verify complete content was written
        with open(filename, 'r') as f:
            read_content = f.read().strip()

        self.assertEqual(read_content, test_content,
                        "File content incomplete - possible atomic write bug")

        # Verify file is not locked/corrupted
        try:
            with open(filename, 'a') as f:
                f.write(" additional content")
        except IOError:
            self.fail("File appears to be locked - possible file handling bug")

        # Cleanup
        if os.path.exists(filename):
            os.remove(filename)

    def test_buffer_word_count_exact_not_approximate(self):
        """CRITICAL: Ensure word counting is exact, not approximate"""
        from ha_converse import HAConverse
        app = HAConverse()

        # Test edge cases that could reveal counting bugs
        test_cases = [
            ("word1 word2 word3", 3),
            ("  spaced  words  ", 2),  # Leading/trailing spaces
            ("hyphenated-word", 1),    # Hyphenation
            ("don't", 1),              # Contractions
            ("123", 1),                # Numbers
            ("word,punctuation!", 1),   # Punctuation
            ("", 0),                   # Empty
            ("   ", 0),                # Whitespace only
        ]

        for text, expected in test_cases:
            with self.subTest(text=repr(text)):
                count = app.count_words(text)
                self.assertEqual(count, expected,
                               f"Word count bug: '{text}' expected {expected}, got {count}")

    def test_timestamp_format_consistency(self):
        """CRITICAL: Ensure timestamp formats are consistent across files"""
        from ha_converse import HAConverse
        app = HAConverse()

        # Create multiple files quickly
        files_created = []
        for i in range(5):
            conv_file = app.create_conversation_file(f"Content {i}", counter=i+1)
            resp_file = app.create_response_file(f"Response {i}", counter=i+1)
            files_created.extend([conv_file, resp_file])

        # Check timestamp format consistency
        import re
        timestamp_pattern = r'\d{4}-\d{2}-\d{2}_\d{2}-\d{2}-\d{2}__\d{3}'

        for filename in files_created:
            self.assertRegex(filename, timestamp_pattern,
                           f"Timestamp format bug in filename: {filename}")

        # Cleanup
        for filename in files_created:
            if os.path.exists(filename):
                os.remove(filename)

    def test_rate_limit_parsing_exact_not_default(self):
        """CRITICAL: Ensure rate limit parsing extracts exact times, not defaults"""
        from ha_converse import HAConverse
        app = HAConverse()

        # Test various rate limit response formats
        test_responses = [
            "Try again in 59.165 seconds",
            "Rate limited. Retry after 120 seconds",
            "Wait 45.5 seconds before next request",
        ]

        expected_times = [59.165, 120, 45.5]

        for response, expected in zip(test_responses, expected_times):
            with self.subTest(response=response):
                parsed_time = app.parse_rate_limit_time(response)
                self.assertAlmostEqual(parsed_time, expected, places=3,
                                     msg=f"Rate limit parsing bug: '{response}' should yield {expected}")

    def test_silence_detection_precise_timing(self):
        """CRITICAL: Ensure silence detection uses precise timing, not approximation"""
        from ha_converse import HAConverse
        app = HAConverse(interactive_mode=True)

        # Test precise 3-second timing
        app.update_last_speech_time()

        # Just under 3 seconds
        with patch('ha_converse.time.time', return_value=app.last_speech_time + 2.95):
            self.assertFalse(app.check_silence_trigger(),
                           "Silence trigger firing early - timing precision bug")

        # Just over 3 seconds
        with patch('ha_converse.time.time', return_value=app.last_speech_time + 3.05):
            self.assertTrue(app.check_silence_trigger(),
                          "Silence trigger not firing on time - timing precision bug")


if __name__ == '__main__':
    # Run with verbose output to catch all potential bugs
    unittest.main(verbosity=2, buffer=True)