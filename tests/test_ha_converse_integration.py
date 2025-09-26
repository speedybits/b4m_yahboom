#!/usr/bin/env python3
"""
Integration tests for HA_converse implementation.

These tests verify complete end-to-end workflows and cross-component integration:
- Full speech-to-text-to-API-to-TTS workflows
- Real file system operations
- Complete thread lifecycle testing
- Environment variable integration
- Command-line argument handling
- Signal handling integration
"""

import unittest
from unittest.mock import Mock, patch, MagicMock, mock_open
import threading
import time
import os
import tempfile
import shutil
import subprocess
import signal
import json
import sys
from datetime import datetime
import numpy as np


class TestHAConverseIntegration(unittest.TestCase):
    """Integration tests for complete HA_converse workflows"""

    def setUp(self):
        """Set up integration test environment"""
        self.test_dir = tempfile.mkdtemp()
        self.original_cwd = os.getcwd()
        os.chdir(self.test_dir)

        # Set up test environment variables
        self.env_vars = {
            'B4M_API_KEY': 'test_integration_key_123',
            'B4M_ROSIE_ID': 'test_integration_rosie_456',
            'B4M_USER_ID': 'test_integration_user_789',
            'PIPER_MODEL_PATH': os.path.join(self.test_dir, 'test_model.onnx'),
            'PIPER_CONFIG_PATH': os.path.join(self.test_dir, 'test_config.json')
        }

        # Create fake Piper model files
        with open(self.env_vars['PIPER_MODEL_PATH'], 'wb') as f:
            f.write(b'fake_onnx_model_data')
        with open(self.env_vars['PIPER_CONFIG_PATH'], 'w') as f:
            json.dump({"sample_rate": 22050, "num_speakers": 1}, f)

        self.env_patcher = patch.dict(os.environ, self.env_vars)
        self.env_patcher.start()

    def tearDown(self):
        """Clean up integration test environment"""
        self.env_patcher.stop()
        os.chdir(self.original_cwd)
        shutil.rmtree(self.test_dir, ignore_errors=True)


class TestCompleteWorkflows(TestHAConverseIntegration):
    """Test complete end-to-end workflows"""

    @patch('ha_converse.sounddevice')
    @patch('ha_converse.faster_whisper')
    @patch('ha_converse.requests')
    @patch('ha_converse.piper')
    def test_full_keyword_workflow_integration(self, mock_piper, mock_requests,
                                             mock_whisper, mock_sd):
        """Test complete workflow: Audio -> Whisper -> B4M -> Keyword -> TTS -> Cleanup"""

        # Setup Whisper transcription
        mock_model = Mock()
        mock_segments = []

        # Simulate accumulating exactly 20 words with trigger word
        transcription_parts = [
            "Hello this is a test message",  # 7 words
            "for the speech recognition system",  # 5 words
            "that will trigger Rosie when complete",  # 6 words
            "and ready for processing now"  # 5 words = 23 total, trigger at 20
        ]

        for i, part in enumerate(transcription_parts):
            segment = Mock()
            segment.text = part
            segment.start = i * 3.0
            segment.end = (i + 1) * 3.0
            mock_segments.append(segment)

        mock_model.transcribe.return_value = (mock_segments, Mock())
        mock_whisper.WhisperModel.return_value = mock_model

        # Setup B4M API responses
        quest_response = Mock()
        quest_response.status_code = 200
        quest_response.json.return_value = {
            'questId': 'integration_quest_123',
            'status': 'running'
        }

        completion_response = Mock()
        completion_response.status_code = 200
        completion_response.json.return_value = {
            'status': 'done',
            'replies': ['Integration test response from B4M AI']
        }

        mock_requests.post.return_value = quest_response
        mock_requests.get.return_value = completion_response

        # Setup Piper TTS
        mock_voice = Mock()
        mock_voice.synthesize.return_value = np.random.random(22050)
        mock_piper.PiperVoice.load_model.return_value = mock_voice

        # Setup audio recording
        mock_sd.rec.return_value = np.random.random((16000, 1))

        # Run the integration test
        from ha_converse import HAConverse
        app = HAConverse(enable_b4m=True, enable_piper=True)

        # Simulate the workflow step by step
        workflow_steps = []

        # Step 1: Audio capture and transcription
        audio_data = app.capture_audio_chunk()
        self.assertIsNotNone(audio_data)
        workflow_steps.append("audio_captured")

        # Step 2: Transcribe audio
        full_transcription = ""
        for segment in mock_segments:
            transcription = app.transcribe_audio(audio_data)
            full_transcription += " " + segment.text

            # Add words to buffer
            words = segment.text.split()
            for word in words:
                app.add_to_buffer(word)

                # Check if buffer is full (triggers B4M processing)
                if app.is_buffer_full():
                    workflow_steps.append("buffer_full")

                    # Step 3: Process with B4M API
                    buffer_content = app.get_buffer_content()
                    response = app.send_to_b4m(buffer_content)
                    self.assertIsNotNone(response)
                    workflow_steps.append("b4m_processed")

                    # Step 4: Create response file
                    response_file = app.create_response_file(response, app.get_file_counter())
                    self.assertIsNotNone(response_file)
                    self.assertTrue(os.path.exists(response_file))
                    workflow_steps.append("response_file_created")

                    # Reset buffer
                    app.reset_buffer()
                    break

        # Step 5: Detect keyword trigger
        trigger_detected = app.detect_keyword_trigger(full_transcription)
        self.assertTrue(trigger_detected, "Should detect 'Rosie' in transcription")
        workflow_steps.append("trigger_detected")

        # Step 6: Process TTS trigger
        oldest_response = app.get_oldest_response_file()
        self.assertIsNotNone(oldest_response)

        app.speak_response_file(oldest_response)
        workflow_steps.append("tts_completed")

        # Step 7: Verify cleanup
        self.assertFalse(os.path.exists(oldest_response), "Response file should be deleted after TTS")
        workflow_steps.append("cleanup_completed")

        # Verify complete workflow
        expected_steps = [
            "audio_captured",
            "buffer_full",
            "b4m_processed",
            "response_file_created",
            "trigger_detected",
            "tts_completed",
            "cleanup_completed"
        ]

        self.assertEqual(workflow_steps, expected_steps, "Workflow steps don't match expected sequence")

        # Verify API calls were made correctly
        mock_requests.post.assert_called_once()
        post_args = mock_requests.post.call_args
        self.assertEqual(post_args[0][0], 'https://app.bike4mind.com/api/ai/llm')

        # Verify TTS synthesis was called
        mock_voice.synthesize.assert_called_once()

    @patch('ha_converse.time')
    def test_full_interactive_workflow_integration(self, mock_time):
        """Test complete interactive mode workflow with silence detection"""

        # Mock time progression for silence detection
        time_sequence = [
            100.0,  # Initial speech time
            100.5,  # 0.5s later - should not show timer yet
            101.0,  # 1.0s later - should show timer
            102.0,  # 2.0s later - still showing timer
            103.5   # 3.5s later - should trigger
        ]

        mock_time.time.side_effect = time_sequence

        from ha_converse import HAConverse
        app = HAConverse(interactive_mode=True, enable_piper=True)

        workflow_steps = []

        # Step 1: Initial speech activity
        app.update_last_speech_time()
        workflow_steps.append("speech_detected")

        # Step 2: Check timer display progression
        mock_time.time.return_value = time_sequence[1]  # 0.5s
        should_show = app.should_show_timer()
        self.assertTrue(should_show, "Timer should show after 0.5s")
        workflow_steps.append("timer_displayed")

        # Step 3: Continue silence progression
        mock_time.time.return_value = time_sequence[3]  # 2.0s
        triggered = app.check_silence_trigger()
        self.assertFalse(triggered, "Should not trigger before 3.0s")

        # Step 4: Trigger at 3.5s
        mock_time.time.return_value = time_sequence[4]  # 3.5s
        triggered = app.check_silence_trigger()
        self.assertTrue(triggered, "Should trigger after 3.0s")
        workflow_steps.append("silence_triggered")

        # Step 5: Create mock response file for TTS
        response_content = "Interactive mode test response"
        response_file = app.create_response_file(response_content, 1)
        self.assertTrue(os.path.exists(response_file))
        workflow_steps.append("response_ready")

        # Step 6: Process TTS trigger
        with patch.object(app, 'synthesize_speech') as mock_synth:
            mock_synth.return_value = np.random.random(22050)
            app.process_tts_trigger()
            workflow_steps.append("tts_processed")

        # Step 7: Verify response file cleanup
        self.assertFalse(os.path.exists(response_file), "Response file should be cleaned up")
        workflow_steps.append("cleanup_completed")

        # Verify workflow sequence
        expected_steps = [
            "speech_detected",
            "timer_displayed",
            "silence_triggered",
            "response_ready",
            "tts_processed",
            "cleanup_completed"
        ]

        self.assertEqual(workflow_steps, expected_steps)

    def test_test_mode_complete_integration(self):
        """Test complete test mode workflow with file-based input"""

        # Create test conversation file
        test_sentences = [
            "This is the first test sentence with some words",  # 9 words
            "Here is another sentence for our testing system today",  # 9 words
            "The third sentence contains the trigger word Rosie here",  # 9 words (27 total - should create 1 file at 20, continue with 7)
            "Final test sentence to complete our integration test"  # 8 words (15 total)
        ]

        with open("conversation_test.txt", "w") as f:
            f.write("\n".join(test_sentences))

        from ha_converse import HAConverse
        app = HAConverse(test_mode=True, enable_b4m=True, enable_piper=True)

        workflow_steps = []

        # Step 1: Load test sentences
        sentences = app.load_test_sentences()
        self.assertEqual(len(sentences), 4)
        workflow_steps.append("test_file_loaded")

        # Step 2: Process sentences and track buffer
        conversation_files_created = 0
        trigger_detected = False

        for sentence in sentences:
            # Process sentence
            app.process_test_sentence(sentence)
            workflow_steps.append(f"processed_sentence")

            # Check for trigger word
            if app.detect_keyword_trigger(sentence):
                trigger_detected = True
                workflow_steps.append("trigger_found")

            # Check if buffer triggered file creation
            if app.get_buffer_count() == 0 and len([f for f in os.listdir('.') if f.startswith('conversation_')]) > conversation_files_created:
                conversation_files_created += 1
                workflow_steps.append("conversation_file_created")

        # Step 3: Verify file creation occurred
        self.assertGreater(conversation_files_created, 0, "Should create at least one conversation file")

        # Step 4: Verify trigger detection
        self.assertTrue(trigger_detected, "Should detect trigger word in test sentences")

        # Step 5: Verify file pattern
        conversation_files = [f for f in os.listdir('.') if f.startswith('conversation_')]
        for filename in conversation_files:
            self.assertRegex(filename, r'conversation_\d{4}-\d{2}-\d{2}_\d{2}-\d{2}-\d{2}__\d{3}\.txt')

        workflow_steps.append("integration_completed")

        # Cleanup
        os.remove("conversation_test.txt")
        for filename in conversation_files:
            if os.path.exists(filename):
                os.remove(filename)

        # Verify expected workflow
        self.assertIn("test_file_loaded", workflow_steps)
        self.assertIn("conversation_file_created", workflow_steps)
        self.assertIn("trigger_found", workflow_steps)


class TestEnvironmentIntegration(TestHAConverseIntegration):
    """Test environment variable and configuration integration"""

    def test_environment_variable_loading(self):
        """Test all environment variables are loaded correctly"""
        from ha_converse import HAConverse
        app = HAConverse(enable_b4m=True, enable_piper=True)

        # Verify B4M configuration
        self.assertEqual(app.b4m_api_key, 'test_integration_key_123')
        self.assertEqual(app.b4m_rosie_id, 'test_integration_rosie_456')
        self.assertEqual(app.b4m_user_id, 'test_integration_user_789')

        # Verify Piper configuration
        self.assertEqual(app.piper_model_path, self.env_vars['PIPER_MODEL_PATH'])
        self.assertEqual(app.piper_config_path, self.env_vars['PIPER_CONFIG_PATH'])

    def test_missing_environment_variables_handling(self):
        """Test graceful handling of missing environment variables"""
        # Remove required environment variables
        with patch.dict(os.environ, {}, clear=True):
            from ha_converse import HAConverse

            # Should handle missing B4M variables
            with self.assertRaises(ValueError) as context:
                app = HAConverse(enable_b4m=True)

            self.assertIn("B4M_API_KEY", str(context.exception))

    def test_invalid_piper_model_paths(self):
        """Test handling of invalid Piper model paths"""
        # Set invalid paths
        invalid_env = self.env_vars.copy()
        invalid_env['PIPER_MODEL_PATH'] = '/nonexistent/path/model.onnx'
        invalid_env['PIPER_CONFIG_PATH'] = '/nonexistent/path/config.json'

        with patch.dict(os.environ, invalid_env):
            from ha_converse import HAConverse

            # Should handle missing files gracefully
            try:
                app = HAConverse(enable_piper=True)
                # Should either fall back to simulation or raise clear error
                self.assertTrue(True)  # Reached this point without crashing
            except FileNotFoundError as e:
                # Should provide clear error message
                self.assertIn("Piper", str(e))

    def test_command_line_argument_integration(self):
        """Test command-line argument parsing and application"""
        from ha_converse import HAConverse

        # Test different argument combinations
        test_cases = [
            (['--interactive'], {'interactive_mode': True}),
            (['--test'], {'test_mode': True}),
            (['--b4m'], {'enable_b4m': True}),
            (['--piper'], {'enable_piper': True}),
            (['--interactive', '--piper'], {'interactive_mode': True, 'enable_piper': True}),
            (['--b4m', '--piper', '--interactive'], {
                'enable_b4m': True,
                'enable_piper': True,
                'interactive_mode': True
            }),
        ]

        for args, expected_config in test_cases:
            with self.subTest(args=args):
                with patch('sys.argv', ['ha_converse.py'] + args):
                    app = HAConverse.from_command_line()

                    for key, expected_value in expected_config.items():
                        actual_value = getattr(app, key)
                        self.assertEqual(actual_value, expected_value,
                                       f"Argument {key} not set correctly for args {args}")


class TestThreadIntegration(TestHAConverseIntegration):
    """Test thread integration and lifecycle management"""

    def test_complete_thread_lifecycle(self):
        """Test complete thread startup and shutdown lifecycle"""
        from ha_converse import HAConverse
        app = HAConverse()

        # Track thread states
        thread_states = {
            'speech_started': False,
            'tts_started': False,
            'speech_stopped': False,
            'tts_stopped': False
        }

        # Mock thread target functions to track lifecycle
        def mock_speech_recognition():
            thread_states['speech_started'] = True
            while not app.shutdown_event.is_set():
                time.sleep(0.01)
            thread_states['speech_stopped'] = True

        def mock_tts_handler():
            thread_states['tts_started'] = True
            while not app.shutdown_event.is_set():
                time.sleep(0.01)
            thread_states['tts_stopped'] = True

        with patch.object(app, 'run_speech_recognition', mock_speech_recognition):
            with patch.object(app, 'run_tts_handler', mock_tts_handler):

                # Start threads
                speech_thread = threading.Thread(target=app.run_speech_recognition)
                tts_thread = threading.Thread(target=app.run_tts_handler)

                speech_thread.daemon = True
                tts_thread.daemon = True

                speech_thread.start()
                tts_thread.start()

                # Wait for threads to start
                time.sleep(0.1)
                self.assertTrue(thread_states['speech_started'])
                self.assertTrue(thread_states['tts_started'])

                # Signal shutdown
                app.shutdown_event.set()

                # Wait for threads to stop
                speech_thread.join(timeout=1.0)
                tts_thread.join(timeout=1.0)

                # Verify clean shutdown
                self.assertTrue(thread_states['speech_stopped'])
                self.assertTrue(thread_states['tts_stopped'])
                self.assertFalse(speech_thread.is_alive())
                self.assertFalse(tts_thread.is_alive())

    def test_thread_communication_integration(self):
        """Test thread communication mechanisms work correctly"""
        from ha_converse import HAConverse
        app = HAConverse()

        communication_log = []

        def mock_speech_thread():
            communication_log.append("speech_started")

            # Simulate speech detection
            app.set_voice_activity(True)
            communication_log.append("voice_activity_set")

            # Wait a bit then clear
            time.sleep(0.1)
            app.set_voice_activity(False)
            communication_log.append("voice_activity_cleared")

            # Wait for shutdown
            while not app.shutdown_event.is_set():
                time.sleep(0.01)
            communication_log.append("speech_shutdown")

        def mock_tts_thread():
            communication_log.append("tts_started")

            # Monitor voice activity
            while not app.shutdown_event.is_set():
                if app.is_voice_active():
                    communication_log.append("tts_detected_voice")
                    break
                time.sleep(0.01)

            communication_log.append("tts_shutdown")

        # Start threads
        speech_thread = threading.Thread(target=mock_speech_thread)
        tts_thread = threading.Thread(target=mock_tts_thread)

        speech_thread.daemon = True
        tts_thread.daemon = True

        speech_thread.start()
        tts_thread.start()

        # Let communication happen
        time.sleep(0.5)

        # Shutdown
        app.shutdown_event.set()
        speech_thread.join(timeout=1.0)
        tts_thread.join(timeout=1.0)

        # Verify communication occurred
        expected_events = [
            "speech_started",
            "tts_started",
            "voice_activity_set",
            "tts_detected_voice"
        ]

        for event in expected_events:
            self.assertIn(event, communication_log, f"Missing communication event: {event}")


class TestSignalHandlingIntegration(TestHAConverseIntegration):
    """Test signal handling integration"""

    def test_sigint_handler_integration(self):
        """Test SIGINT (Ctrl+C) signal handling works correctly"""
        from ha_converse import HAConverse

        shutdown_handled = threading.Event()

        def mock_signal_handler(signum, frame):
            shutdown_handled.set()

        app = HAConverse()

        # Mock signal handler registration
        with patch('signal.signal') as mock_signal:
            app.setup_signal_handlers()

            # Verify SIGINT handler was registered
            mock_signal.assert_any_call(signal.SIGINT, app.signal_handler)
            mock_signal.assert_any_call(signal.SIGTERM, app.signal_handler)

        # Test actual signal handling
        original_handler = signal.signal(signal.SIGINT, mock_signal_handler)

        try:
            # Simulate SIGINT
            os.kill(os.getpid(), signal.SIGINT)

            # Handler should be called quickly
            signal_handled = shutdown_handled.wait(timeout=0.5)
            self.assertTrue(signal_handled, "SIGINT handler not called")

        finally:
            # Restore original handler
            signal.signal(signal.SIGINT, original_handler)

    def test_graceful_shutdown_integration(self):
        """Test complete graceful shutdown sequence"""
        from ha_converse import HAConverse
        app = HAConverse()

        shutdown_steps = []

        # Mock shutdown sequence methods
        def mock_cleanup():
            shutdown_steps.append("cleanup_called")

        def mock_save_buffer():
            shutdown_steps.append("buffer_saved")

        def mock_stop_audio():
            shutdown_steps.append("audio_stopped")

        with patch.object(app, 'cleanup_temp_files', mock_cleanup):
            with patch.object(app, 'save_current_buffer', mock_save_buffer):
                with patch.object(app, 'stop_audio_streams', mock_stop_audio):

                    # Trigger graceful shutdown
                    app.graceful_shutdown(signal.SIGINT, None)

                    # Verify shutdown sequence
                    expected_steps = ["buffer_saved", "audio_stopped", "cleanup_called"]
                    for step in expected_steps:
                        self.assertIn(step, shutdown_steps, f"Missing shutdown step: {step}")


class TestFileSystemIntegration(TestHAConverseIntegration):
    """Test file system operations integration"""

    def test_startup_cleanup_integration(self):
        """Test startup cleanup removes existing files correctly"""
        # Create various test files
        test_files = [
            "conversation_2024-01-15_10-20-30__001.txt",
            "conversation_2024-01-15_10-25-45__002.txt",
            "response_2024-01-15_10-20-35__001.txt",
            "response_2024-01-15_10-25-50__002.txt",
            "other_important_file.txt",
            "README.md"
        ]

        for filename in test_files:
            with open(filename, 'w') as f:
                f.write("Test content")

        from ha_converse import HAConverse
        app = HAConverse()
        app.startup_cleanup()

        # Check cleanup results
        for filename in test_files:
            if filename.startswith(('conversation_', 'response_')):
                self.assertFalse(os.path.exists(filename),
                               f"File {filename} should be cleaned up on startup")
            else:
                self.assertTrue(os.path.exists(filename),
                              f"File {filename} should not be cleaned up")

        # Cleanup remaining files
        for filename in ["other_important_file.txt", "README.md"]:
            if os.path.exists(filename):
                os.remove(filename)

    def test_file_queue_processing_integration(self):
        """Test complete file queue processing workflow"""
        from ha_converse import HAConverse
        app = HAConverse(enable_b4m=True)

        # Create multiple conversation files with different timestamps
        conversation_files = []
        response_files = []

        base_time = datetime.now()
        for i in range(5):
            # Create conversation file
            timestamp = base_time.replace(second=i*10)  # 10-second intervals
            with patch('ha_converse.datetime') as mock_dt:
                mock_dt.now.return_value = timestamp
                conv_file = app.create_conversation_file(f"Conversation content {i}", counter=i+1)
                conversation_files.append(conv_file)

        # Verify files created
        self.assertEqual(len(conversation_files), 5)
        for filename in conversation_files:
            self.assertTrue(os.path.exists(filename))

        # Process queue in order
        with patch.object(app, 'send_to_b4m') as mock_b4m:
            mock_b4m.return_value = "Mocked B4M response"

            # Process files in FIFO order
            for i in range(5):
                oldest_file = app.get_oldest_conversation_file()
                self.assertIsNotNone(oldest_file)

                # Should be the earliest timestamp
                self.assertEqual(oldest_file, conversation_files[i])

                # Process file
                with open(oldest_file, 'r') as f:
                    content = f.read()

                response = app.send_to_b4m(content)
                response_file = app.create_response_file(response, counter=i+1)
                response_files.append(response_file)

                # Remove processed conversation file
                os.remove(oldest_file)

        # Verify processing results
        self.assertEqual(len(response_files), 5)
        for filename in response_files:
            self.assertTrue(os.path.exists(filename))

        # Verify no conversation files remain
        remaining_conv_files = [f for f in os.listdir('.') if f.startswith('conversation_')]
        self.assertEqual(len(remaining_conv_files), 0)

        # Cleanup response files
        for filename in response_files:
            if os.path.exists(filename):
                os.remove(filename)


if __name__ == '__main__':
    unittest.main(verbosity=2)