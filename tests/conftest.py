#!/usr/bin/env python3
"""
Pytest configuration and shared fixtures for HA_converse tests.

This file provides common test fixtures and configuration for all test files.
"""

import pytest
import tempfile
import shutil
import os
import threading
import time
from unittest.mock import Mock, patch
import numpy as np


@pytest.fixture
def temp_test_dir():
    """Provide a temporary directory for tests"""
    test_dir = tempfile.mkdtemp()
    original_cwd = os.getcwd()
    os.chdir(test_dir)

    yield test_dir

    os.chdir(original_cwd)
    shutil.rmtree(test_dir, ignore_errors=True)


@pytest.fixture
def mock_environment():
    """Provide mocked environment variables for testing"""
    test_env = {
        'B4M_API_KEY': 'test_api_key_fixture',
        'B4M_ROSIE_ID': 'test_rosie_id_fixture',
        'B4M_USER_ID': 'test_user_id_fixture',
        'PIPER_MODEL_PATH': '/fake/test/model.onnx',
        'PIPER_CONFIG_PATH': '/fake/test/config.json'
    }

    with patch.dict(os.environ, test_env):
        yield test_env


@pytest.fixture
def mock_whisper_model():
    """Provide a mocked Whisper model for testing"""
    mock_model = Mock()

    # Default transcription behavior
    def mock_transcribe(audio, **kwargs):
        # Return mock segments based on audio length
        duration = len(audio) / 16000  # Assume 16kHz sample rate
        segments = [
            Mock(text="test transcription", start=0.0, end=duration)
        ]
        return segments, Mock()

    mock_model.transcribe.side_effect = mock_transcribe

    with patch('ha_converse.faster_whisper.WhisperModel', return_value=mock_model):
        yield mock_model


@pytest.fixture
def mock_piper_voice():
    """Provide a mocked Piper voice for testing"""
    mock_voice = Mock()

    # Default synthesis behavior
    def mock_synthesize(text):
        # Return fake audio data proportional to text length
        audio_length = len(text) * 100  # Roughly 100 samples per character
        return np.random.random(audio_length)

    mock_voice.synthesize.side_effect = mock_synthesize

    with patch('ha_converse.piper.PiperVoice.load_model', return_value=mock_voice):
        yield mock_voice


@pytest.fixture
def mock_sounddevice():
    """Provide mocked sounddevice for audio I/O testing"""
    with patch('ha_converse.sounddevice') as mock_sd:
        # Default audio recording behavior
        mock_sd.rec.return_value = np.random.random((16000, 1))  # 1 second of mono audio

        # Default audio playback behavior
        mock_stream = Mock()
        mock_stream.finished = False
        mock_sd.play.return_value = mock_stream

        yield mock_sd


@pytest.fixture
def mock_requests():
    """Provide mocked requests for B4M API testing"""
    with patch('ha_converse.requests') as mock_req:
        # Default B4M API response
        mock_response = Mock()
        mock_response.status_code = 200
        mock_response.json.return_value = {
            'questId': 'test_quest_fixture',
            'status': 'done',
            'replies': ['Test B4M response from fixture']
        }

        mock_req.post.return_value = mock_response
        mock_req.get.return_value = mock_response

        yield mock_req


@pytest.fixture
def ha_converse_app(temp_test_dir, mock_environment):
    """Provide a basic HA_converse app instance for testing"""
    from ha_converse import HAConverse

    with patch('ha_converse.faster_whisper.WhisperModel'):
        with patch('ha_converse.sounddevice'):
            app = HAConverse()
            yield app


@pytest.fixture
def conversation_test_file(temp_test_dir):
    """Provide a test conversation file for test mode"""
    test_sentences = [
        "This is the first test sentence for our system.",
        "Here we have another sentence with different words.",
        "The third sentence mentions the trigger word Rosie clearly.",
        "Final sentence to complete our testing scenario today."
    ]

    filename = "conversation_test.txt"
    with open(filename, 'w') as f:
        f.write('\n'.join(test_sentences))

    yield filename

    if os.path.exists(filename):
        os.remove(filename)


@pytest.fixture
def sample_audio_data():
    """Provide sample audio data for testing"""
    # Generate 1 second of random audio data at 16kHz
    sample_rate = 16000
    duration = 1.0
    samples = int(sample_rate * duration)

    audio_data = np.random.random(samples) * 0.1  # Low amplitude to avoid clipping
    return audio_data


@pytest.fixture
def thread_timeout():
    """Provide a reasonable timeout for thread operations in tests"""
    return 2.0  # 2 seconds should be enough for most thread operations


class TestResourceTracker:
    """Helper class to track resource usage during tests"""

    def __init__(self):
        self.initial_thread_count = threading.active_count()
        self.leaked_threads = []

    def check_thread_leaks(self):
        """Check for thread leaks and return list of leaked threads"""
        current_count = threading.active_count()
        if current_count > self.initial_thread_count:
            # Identify leaked threads
            all_threads = threading.enumerate()
            for thread in all_threads:
                if thread.is_alive() and hasattr(thread, '_target'):
                    self.leaked_threads.append(thread.name)

        return self.leaked_threads


@pytest.fixture
def resource_tracker():
    """Provide resource tracking for tests"""
    tracker = TestResourceTracker()
    yield tracker

    # Check for leaks after test
    leaked = tracker.check_thread_leaks()
    if leaked:
        pytest.fail(f"Thread leaks detected: {leaked}")


@pytest.fixture(scope="session")
def test_configuration():
    """Provide test configuration settings"""
    return {
        'audio_sample_rate': 16000,
        'test_timeout': 5.0,
        'buffer_size': 20,
        'silence_threshold': 3.0,
        'api_timeout': 10.0
    }


# Pytest configuration
def pytest_configure(config):
    """Configure pytest settings"""
    # Add custom markers
    config.addinivalue_line(
        "markers", "slow: marks tests as slow (deselect with '-m \"not slow\"')"
    )
    config.addinivalue_line(
        "markers", "integration: marks tests as integration tests"
    )
    config.addinivalue_line(
        "markers", "unit: marks tests as unit tests"
    )
    config.addinivalue_line(
        "markers", "stress: marks tests as stress tests"
    )


def pytest_collection_modifyitems(config, items):
    """Modify test collection to add markers automatically"""
    for item in items:
        # Add markers based on test file names
        if "integration" in item.fspath.basename:
            item.add_marker(pytest.mark.integration)
        elif "stress" in item.fspath.basename:
            item.add_marker(pytest.mark.stress)
            item.add_marker(pytest.mark.slow)
        else:
            item.add_marker(pytest.mark.unit)


# Custom assertion helpers
def assert_audio_data_valid(audio_data):
    """Assert that audio data is valid"""
    assert isinstance(audio_data, np.ndarray), "Audio data must be numpy array"
    assert len(audio_data) > 0, "Audio data must not be empty"
    assert audio_data.dtype in [np.float32, np.float64], "Audio data must be float type"


def assert_timestamp_format(filename):
    """Assert that filename has correct timestamp format"""
    import re
    pattern = r'\d{4}-\d{2}-\d{2}_\d{2}-\d{2}-\d{2}__\d{3}\.txt'
    assert re.search(pattern, filename), f"Filename {filename} doesn't match timestamp format"


def assert_thread_responsive(thread, timeout=1.0):
    """Assert that thread responds to shutdown within timeout"""
    thread.join(timeout=timeout)
    assert not thread.is_alive(), f"Thread {thread.name} did not shutdown within {timeout}s"


# Make custom assertions available to all tests
pytest.assert_audio_data_valid = assert_audio_data_valid
pytest.assert_timestamp_format = assert_timestamp_format
pytest.assert_thread_responsive = assert_thread_responsive