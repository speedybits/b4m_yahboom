#!/usr/bin/env python3
"""
Stress tests and edge case tests for HA_converse implementation.

These tests focus on stress conditions and edge cases that could reveal bugs:
- Concurrent operations under load
- Memory usage patterns
- File system stress
- Network timeout edge cases
- Thread synchronization under pressure
- Resource cleanup verification
"""

import unittest
from unittest.mock import Mock, patch, MagicMock
import threading
import time
import os
import tempfile
import shutil
import gc
import psutil
import concurrent.futures
import random
import string


class TestHAConverseStress(unittest.TestCase):
    """Stress tests for HA_converse implementation"""

    def setUp(self):
        """Set up stress test environment"""
        self.test_dir = tempfile.mkdtemp()
        self.original_cwd = os.getcwd()
        os.chdir(self.test_dir)

        # Track initial resource usage
        self.initial_memory = psutil.Process().memory_info().rss
        self.initial_threads = threading.active_count()

    def tearDown(self):
        """Clean up and verify no resource leaks"""
        os.chdir(self.original_cwd)
        shutil.rmtree(self.test_dir, ignore_errors=True)

        # Force garbage collection
        gc.collect()

        # Check for resource leaks
        final_memory = psutil.Process().memory_info().rss
        final_threads = threading.active_count()

        # Memory should not grow excessively (allow 50MB growth)
        memory_growth = final_memory - self.initial_memory
        self.assertLess(memory_growth, 50 * 1024 * 1024,
                       f"Potential memory leak: {memory_growth / 1024 / 1024:.1f}MB growth")

        # Thread count should return to initial (allow 2 thread tolerance)
        thread_growth = final_threads - self.initial_threads
        self.assertLessEqual(thread_growth, 2,
                            f"Potential thread leak: {thread_growth} extra threads")

    def test_concurrent_buffer_operations(self):
        """Test buffer operations under concurrent access"""
        from ha_converse import HAConverse
        app = HAConverse()

        errors = []
        results = []

        def add_words_worker(thread_id, word_count):
            try:
                for i in range(word_count):
                    word = f"thread{thread_id}_word{i}"
                    app.add_to_buffer(word)
                    # Small random delay to increase contention
                    time.sleep(random.uniform(0.001, 0.005))
                results.append(f"Thread {thread_id} completed")
            except Exception as e:
                errors.append(f"Thread {thread_id}: {e}")

        # Start multiple threads adding words concurrently
        threads = []
        for i in range(10):
            thread = threading.Thread(target=add_words_worker, args=(i, 25))
            threads.append(thread)
            thread.start()

        # Wait for all threads
        for thread in threads:
            thread.join(timeout=5.0)
            self.assertFalse(thread.is_alive(), "Thread did not complete in time")

        # Check for errors
        self.assertEqual(len(errors), 0, f"Concurrent buffer errors: {errors}")
        self.assertEqual(len(results), 10, "Not all threads completed successfully")

    def test_rapid_file_creation_stress(self):
        """Test rapid file creation and deletion under stress"""
        from ha_converse import HAConverse
        app = HAConverse()

        files_created = []
        errors = []

        def create_files_worker(thread_id):
            try:
                for i in range(50):  # Create 50 files per thread
                    content = f"Thread {thread_id} file {i} " + "word " * 20
                    filename = app.create_conversation_file(content, counter=thread_id*100+i)
                    if filename:
                        files_created.append(filename)
                    time.sleep(0.001)  # Minimal delay
            except Exception as e:
                errors.append(f"Thread {thread_id}: {e}")

        # Start multiple file creation threads
        threads = []
        for i in range(5):
            thread = threading.Thread(target=create_files_worker, args=(i,))
            threads.append(thread)
            thread.start()

        # Wait for completion
        for thread in threads:
            thread.join(timeout=10.0)

        # Verify no errors
        self.assertEqual(len(errors), 0, f"File creation errors: {errors}")

        # Verify files were actually created
        self.assertGreater(len(files_created), 200, "Not enough files created")

        # Cleanup files
        for filename in files_created:
            if os.path.exists(filename):
                os.remove(filename)

    def test_memory_usage_under_load(self):
        """Test memory usage doesn't grow excessively under load"""
        from ha_converse import HAConverse
        app = HAConverse()

        initial_memory = psutil.Process().memory_info().rss

        # Generate large amount of transcription data
        for cycle in range(100):
            # Simulate large transcription
            large_text = "word " * 1000  # 1000 words
            words = large_text.split()

            # Process in chunks
            for i in range(0, len(words), 20):
                chunk = words[i:i+20]
                for word in chunk:
                    app.add_to_buffer(word)

                if app.is_buffer_full():
                    # Mock file creation to avoid disk I/O
                    with patch.object(app, 'create_conversation_file'):
                        app.process_full_buffer()

            # Force garbage collection every 10 cycles
            if cycle % 10 == 0:
                gc.collect()

        final_memory = psutil.Process().memory_info().rss
        memory_growth = final_memory - initial_memory

        # Memory growth should be reasonable (less than 20MB for this test)
        self.assertLess(memory_growth, 20 * 1024 * 1024,
                       f"Excessive memory growth: {memory_growth / 1024 / 1024:.1f}MB")

    def test_thread_creation_destruction_stress(self):
        """Test thread creation and destruction under stress"""
        from ha_converse import HAConverse

        def create_destroy_app():
            try:
                app = HAConverse()
                # Start threads
                speech_thread = threading.Thread(target=app.run_speech_recognition)
                tts_thread = threading.Thread(target=app.run_tts_handler)

                speech_thread.daemon = True
                tts_thread.daemon = True

                speech_thread.start()
                tts_thread.start()

                # Run briefly
                time.sleep(0.1)

                # Shutdown
                app.shutdown_event.set()
                speech_thread.join(timeout=1.0)
                tts_thread.join(timeout=1.0)

                return True
            except Exception:
                return False

        # Create and destroy apps rapidly
        results = []
        with concurrent.futures.ThreadPoolExecutor(max_workers=5) as executor:
            futures = [executor.submit(create_destroy_app) for _ in range(20)]
            results = [f.result(timeout=5.0) for f in futures]

        # All should succeed
        self.assertTrue(all(results), "Some app creation/destruction cycles failed")

    def test_file_system_stress_many_files(self):
        """Test file system operations with many files"""
        from ha_converse import HAConverse
        app = HAConverse()

        # Create many files quickly
        files = []
        for i in range(500):
            filename = f"stress_test_{i:03d}.txt"
            with open(filename, 'w') as f:
                f.write(f"Stress test file {i}")
            files.append(filename)

        # Test finding oldest file with many files present
        start_time = time.time()
        oldest = app.get_oldest_conversation_file()
        search_time = time.time() - start_time

        # Should complete quickly even with many files
        self.assertLess(search_time, 1.0, f"File search too slow: {search_time:.3f}s")

        # Cleanup
        for filename in files:
            if os.path.exists(filename):
                os.remove(filename)

    @patch('ha_converse.requests')
    def test_api_timeout_stress(self, mock_requests):
        """Test API timeout handling under various delay scenarios"""
        from ha_converse import HAConverse
        app = HAConverse(enable_b4m=True)

        # Test various timeout scenarios
        timeout_scenarios = [0.1, 0.5, 1.0, 2.0, 5.0, 10.0, 15.0]

        def slow_response(delay):
            def side_effect(*args, **kwargs):
                time.sleep(delay)
                response = Mock()
                response.status_code = 200
                response.json.return_value = {
                    'questId': f'quest_{delay}',
                    'status': 'done',
                    'replies': [f'Response after {delay}s']
                }
                return response
            return side_effect

        for delay in timeout_scenarios:
            with self.subTest(delay=delay):
                mock_requests.post.side_effect = slow_response(delay)

                start_time = time.time()
                try:
                    result = app.send_to_b4m(f"Test message with {delay}s delay")
                    response_time = time.time() - start_time

                    if delay < 10.0:  # Should succeed
                        self.assertIsNotNone(result)
                    else:  # Should timeout
                        self.assertIsNone(result)

                except Exception as e:
                    # Should handle timeout gracefully
                    self.assertIn("timeout", str(e).lower())

    def test_audio_buffer_overflow_simulation(self):
        """Test handling of audio buffer overflow scenarios"""
        from ha_converse import HAConverse
        app = HAConverse()

        # Simulate rapid audio chunk processing
        import numpy as np

        def process_audio_chunk():
            # Large audio chunk
            audio_data = np.random.random((48000, 1))  # 3 seconds of audio
            try:
                return app.transcribe_audio(audio_data)
            except Exception:
                return None

        # Process many chunks concurrently
        with concurrent.futures.ThreadPoolExecutor(max_workers=8) as executor:
            futures = [executor.submit(process_audio_chunk) for _ in range(20)]

            # Should not crash or deadlock
            results = []
            for future in concurrent.futures.as_completed(futures, timeout=30):
                results.append(future.result())

        # Should complete without hanging
        self.assertEqual(len(results), 20)

    def test_trigger_detection_spam_resilience(self):
        """Test trigger detection can handle spam without breaking"""
        from ha_converse import HAConverse
        app = HAConverse()

        # Generate spam trigger attempts
        spam_texts = [
            "Rosie " * 100,  # Repeated triggers
            "ROSIE rosie RoSiE " * 50,  # Case variations
            "Hey Rosie, Rosie, Rosie!",  # Multiple in one text
        ]

        trigger_count = 0
        for spam_text in spam_texts * 100:  # Repeat 100 times
            if app.detect_keyword_trigger(spam_text):
                trigger_count += 1

        # Should detect triggers but not crash
        self.assertGreater(trigger_count, 0)
        self.assertLess(trigger_count, 1000)  # Reasonable upper bound

    def test_file_counter_overflow_handling(self):
        """Test file counter handling with very large numbers"""
        from ha_converse import HAConverse
        app = HAConverse()

        # Test large counter values
        large_counters = [999, 9999, 99999, 999999]

        for counter in large_counters:
            with self.subTest(counter=counter):
                filename = app.create_conversation_file("Test content", counter=counter)

                # Verify filename format is correct
                self.assertIn(f"__{counter:03d}.txt", filename)

                # Cleanup
                if filename and os.path.exists(filename):
                    os.remove(filename)

    def test_unicode_stress_text_processing(self):
        """Test text processing with various Unicode characters"""
        from ha_converse import HAConverse
        app = HAConverse()

        # Various Unicode text samples
        unicode_texts = [
            "Hello 世界 café naïve résumé",  # Mixed languages
            "🎤🔊🎯💾🗑️⚠️✅❌",  # Emojis
            "Ñoño niña año señor mañana",  # Spanish accents
            "Москва Санкт-Петербург",  # Cyrillic
            "東京 大阪 京都",  # Japanese
            "العربية الفصحى",  # Arabic
            "नमस्ते दुनिया",  # Hindi
        ]

        for text in unicode_texts:
            with self.subTest(text=text):
                # Should handle Unicode without crashing
                word_count = app.count_words(text)
                self.assertGreaterEqual(word_count, 0)

                # Should create files with Unicode content
                filename = app.create_conversation_file(text)
                self.assertIsNotNone(filename)

                # Verify content preserved
                with open(filename, 'r', encoding='utf-8') as f:
                    read_text = f.read().strip()
                self.assertEqual(read_text, text)

                # Cleanup
                if os.path.exists(filename):
                    os.remove(filename)

    def test_rapid_shutdown_signal_stress(self):
        """Test rapid shutdown signals don't cause issues"""
        from ha_converse import HAConverse

        def rapid_shutdown_test():
            app = HAConverse()

            # Start app
            app_thread = threading.Thread(target=app.run)
            app_thread.daemon = True
            app_thread.start()

            time.sleep(0.01)  # Very brief run

            # Rapid shutdown signals
            for _ in range(10):
                app.shutdown_event.set()
                time.sleep(0.001)

            app_thread.join(timeout=1.0)
            return not app_thread.is_alive()

        # Run multiple rapid shutdown tests
        results = []
        for _ in range(10):
            results.append(rapid_shutdown_test())

        # All should complete successfully
        self.assertTrue(all(results), "Some rapid shutdown tests failed")


class TestEdgeCaseBugDetection(unittest.TestCase):
    """Edge case tests specifically designed to catch subtle bugs"""

    def test_empty_string_word_counting_edge_cases(self):
        """Test word counting with tricky empty/whitespace cases"""
        from ha_converse import HAConverse
        app = HAConverse()

        edge_cases = [
            ("", 0),
            (" ", 0),
            ("  ", 0),
            ("\t", 0),
            ("\n", 0),
            (" \t \n ", 0),
            ("a", 1),
            (" a ", 1),
            ("  a  ", 1),
            ("\ta\n", 1),
        ]

        for text, expected in edge_cases:
            with self.subTest(text=repr(text)):
                count = app.count_words(text)
                self.assertEqual(count, expected,
                               f"Edge case bug: {repr(text)} should count {expected}, got {count}")

    def test_timestamp_edge_cases_year_boundary(self):
        """Test timestamp handling around year boundaries"""
        from ha_converse import HAConverse
        from datetime import datetime
        app = HAConverse()

        # Test year boundary cases
        edge_times = [
            datetime(2023, 12, 31, 23, 59, 59),  # End of year
            datetime(2024, 1, 1, 0, 0, 0),       # Start of year
            datetime(2024, 2, 29, 12, 0, 0),     # Leap year
            datetime(2024, 12, 31, 23, 59, 59),  # End of leap year
        ]

        for test_time in edge_times:
            with self.subTest(time=test_time):
                with patch('ha_converse.datetime') as mock_dt:
                    mock_dt.now.return_value = test_time
                    mock_dt.strftime = datetime.strftime

                    filename = app.create_conversation_file("Test", counter=1)

                    # Should create valid filename
                    self.assertIsNotNone(filename)
                    self.assertTrue(filename.endswith('.txt'))

                    # Cleanup
                    if os.path.exists(filename):
                        os.remove(filename)

    def test_buffer_boundary_conditions(self):
        """Test buffer behavior at exact boundaries"""
        from ha_converse import HAConverse
        app = HAConverse()

        # Test exactly 20 words
        words = ["word"] * 20
        for word in words[:-1]:  # Add 19 words
            app.add_to_buffer(word)

        self.assertEqual(app.get_buffer_count(), 19)
        self.assertFalse(app.is_buffer_full())

        # Add exactly the 20th word
        app.add_to_buffer(words[-1])
        self.assertEqual(app.get_buffer_count(), 20)
        self.assertTrue(app.is_buffer_full())

        # Process should reset to 0
        with patch.object(app, 'create_conversation_file'):
            app.process_full_buffer()

        self.assertEqual(app.get_buffer_count(), 0)
        self.assertFalse(app.is_buffer_full())

    def test_silence_timer_precision_edge_cases(self):
        """Test silence timer with precise timing edge cases"""
        from ha_converse import HAConverse
        app = HAConverse(interactive_mode=True)

        app.update_last_speech_time()

        # Test right at the boundary
        with patch('ha_converse.time.time', return_value=app.last_speech_time + 2.999):
            self.assertFalse(app.check_silence_trigger(), "Triggered too early by 1ms")

        with patch('ha_converse.time.time', return_value=app.last_speech_time + 3.000):
            self.assertTrue(app.check_silence_trigger(), "Did not trigger exactly at 3.000s")

        with patch('ha_converse.time.time', return_value=app.last_speech_time + 3.001):
            # Reset for next test
            app.update_last_speech_time()
            self.assertTrue(app.check_silence_trigger(), "Did not trigger at 3.001s")

    def test_rate_limit_parsing_malformed_responses(self):
        """Test rate limit parsing with malformed server responses"""
        from ha_converse import HAConverse
        app = HAConverse()

        malformed_responses = [
            "",  # Empty
            "Try again",  # No time specified
            "Try again in seconds",  # No number
            "Try again in -5 seconds",  # Negative time
            "Try again in abc seconds",  # Non-numeric
            "Try again in 999999 seconds",  # Extremely large
            "Completely unrelated message",  # No rate limit info
        ]

        for response in malformed_responses:
            with self.subTest(response=response):
                # Should handle gracefully, not crash
                try:
                    result = app.parse_rate_limit_time(response)
                    # Should return None or reasonable default for malformed input
                    if result is not None:
                        self.assertGreater(result, 0)
                        self.assertLess(result, 86400)  # Less than a day
                except Exception as e:
                    self.fail(f"Rate limit parsing crashed on: {repr(response)}, error: {e}")

    def test_file_counter_sequence_gaps(self):
        """Test file counter handles gaps in sequence correctly"""
        from ha_converse import HAConverse
        app = HAConverse()

        # Create files with non-sequential counters
        counters = [1, 3, 5, 7, 10, 15]
        created_files = []

        for counter in counters:
            filename = app.create_conversation_file(f"Content {counter}", counter=counter)
            created_files.append(filename)

        # Should handle gaps without issues
        oldest = app.get_oldest_conversation_file()
        self.assertIsNotNone(oldest)

        # Cleanup
        for filename in created_files:
            if os.path.exists(filename):
                os.remove(filename)

    def test_thread_interruption_edge_cases(self):
        """Test thread interruption at various execution points"""
        from ha_converse import HAConverse

        def interrupt_at_various_points():
            app = HAConverse()

            # Start threads
            threads = []
            for target in [app.run_speech_recognition, app.run_tts_handler]:
                thread = threading.Thread(target=target)
                thread.daemon = True
                thread.start()
                threads.append(thread)

            # Interrupt at different delays
            delays = [0.001, 0.01, 0.05, 0.1, 0.2]

            for delay in delays:
                time.sleep(delay)
                app.shutdown_event.set()

                # All threads should stop
                for thread in threads:
                    thread.join(timeout=1.0)
                    if thread.is_alive():
                        return False

                # Reset for next iteration
                app.shutdown_event.clear()

            return True

        # Test should pass - threads should be interruptible at any point
        result = interrupt_at_various_points()
        self.assertTrue(result, "Thread interruption failed at some execution point")


if __name__ == '__main__':
    unittest.main(verbosity=2)