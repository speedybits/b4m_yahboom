# HA_converse Test Suite

Comprehensive unit tests for the HA_converse speech-to-text application designed to detect any bugs in the implementation, including mocked or placeholder code.

## Overview

This test suite is specifically designed to:
- **Detect the inserted bug** in the HA_converse implementation
- Catch placeholder or mocked code in the actual implementation
- Verify complete end-to-end functionality
- Test thread safety and synchronization
- Validate resource management and cleanup
- Ensure precise timing and behavior

## Test Files

### Core Test Files

1. **`test_ha_converse.py`** - Main unit tests (50KB+)
   - Speech recognition functionality
   - B4M API integration and polling system
   - Piper TTS integration
   - Thread management and synchronization
   - File management and queue operations
   - Trigger detection (keyword and interactive modes)
   - Error handling and edge cases
   - **Critical bug detection tests** - specifically designed to catch the inserted bug

2. **`test_ha_converse_stress.py`** - Stress and performance tests (21KB+)
   - Concurrent operations under load
   - Memory usage patterns
   - File system stress testing
   - Network timeout edge cases
   - Thread synchronization under pressure
   - Resource cleanup verification
   - Unicode and edge case handling

3. **`test_ha_converse_integration.py`** - Integration tests (26KB+)
   - Complete end-to-end workflows
   - Environment variable integration
   - Command-line argument handling
   - Signal handling integration
   - File system operations
   - Thread lifecycle management

### Configuration Files

4. **`conftest.py`** - Pytest configuration and shared fixtures
5. **`pytest.ini`** - Test configuration settings
6. **`requirements_test.txt`** - Test-specific dependencies
7. **`run_tests.sh`** - Automated test runner script

## Critical Bug Detection

The test suite includes specialized tests in `TestCriticalBugDetection` class that are designed to catch:

- **Placeholder Code**: Tests detect `# TODO`, `# Mock implementation`, `pass # placeholder`
- **Simulated APIs**: Ensures B4M API calls are real HTTP requests, not simulations
- **Fake Audio Processing**: Verifies Whisper transcription uses actual model calls
- **Thread Issues**: Detects if threads ignore shutdown events (infinite loops)
- **Incomplete File Operations**: Ensures atomic file writes and proper cleanup
- **Timing Precision**: Validates exact timing for silence detection and rate limiting
- **Resource Leaks**: Checks for memory and thread leaks

## Running the Tests

### Quick Start
```bash
# Run all tests with the automated script
./tests/run_tests.sh
```

### Manual Test Execution

```bash
# Install test dependencies
pip install -r tests/requirements_test.txt

# Run specific test categories
pytest tests/test_ha_converse.py -m unit -v                    # Unit tests only
pytest tests/test_ha_converse_integration.py -m integration -v # Integration tests
pytest tests/test_ha_converse_stress.py -m stress -v          # Stress tests

# Run critical bug detection tests
pytest tests/test_ha_converse.py::TestCriticalBugDetection -v

# Run with coverage
pytest tests/ --cov=ha_converse --cov-report=html
```

### Test Markers

Tests are organized with markers for selective execution:

- `unit` - Fast, isolated unit tests
- `integration` - Component interaction tests
- `stress` - Resource-intensive stress tests
- `slow` - Tests that may take more than 1 second
- `b4m` - B4M API integration tests
- `piper` - Piper TTS integration tests
- `audio` - Audio processing tests

## Expected Test Results

### If Implementation is Bug-Free
✅ All tests should pass
✅ Coverage should be >85%
✅ No resource leaks detected
✅ All critical bug detection tests pass

### If Bug is Present
❌ One or more critical tests will fail
❌ Clear error messages indicating the type of bug
❌ Specific test methods point to the problem area

## Test Strategy

### Mock External Dependencies
- Whisper models (to avoid loading large files)
- Audio devices (sounddevice)
- Network requests (B4M API)
- Piper TTS models
- File system operations (when appropriate)

### Test Real Behavior
- Core business logic
- Thread synchronization
- File management
- Word counting and buffer logic
- Trigger detection algorithms
- Error handling flows

### Edge Cases Covered
- Empty/whitespace inputs
- Unicode text processing
- Large file operations
- Concurrent access patterns
- Network timeouts
- Resource exhaustion scenarios
- Signal handling precision

## Bug Detection Methodology

1. **Code Inspection**: Tests use `inspect.getsource()` to examine implementation source code for placeholder patterns

2. **Behavior Validation**: Tests verify actual functionality occurs, not simulation:
   - API calls make real HTTP requests (mocked for testing, but structure verified)
   - Audio processing calls actual Whisper model methods
   - File operations create real files with expected content

3. **Timing Precision**: Tests validate exact timing requirements:
   - Silence detection triggers at exactly 3.000 seconds
   - Rate limit parsing extracts exact wait times
   - Thread shutdown responds within 100ms

4. **Resource Tracking**: Tests monitor for leaks:
   - Thread count before/after operations
   - Memory usage patterns
   - File descriptor management

## Common Bug Patterns Detected

1. **Placeholder Code**:
   ```python
   # TODO: Implement actual functionality
   return "mock response"
   ```

2. **Simulated Behavior**:
   ```python
   def transcribe_audio(self, audio):
       return "simulated transcription"  # Should use real Whisper
   ```

3. **Ignored Shutdown Events**:
   ```python
   while True:  # Should check shutdown_event
       process_audio()
   ```

4. **Approximate Timing**:
   ```python
   if silence_time > 2.5:  # Should be exactly 3.0 seconds
   ```

5. **Incomplete File Operations**:
   ```python
   with open(file, 'w') as f:
       f.write(content[:10])  # Should write complete content
   ```

## Coverage Requirements

- **Minimum Coverage**: 85%
- **Critical Paths**: 100% coverage required for:
  - Buffer management
  - File operations
  - Thread synchronization
  - Signal handling
  - API integration points

## Performance Benchmarks

- **Thread Startup**: <100ms
- **Shutdown Response**: <500ms
- **File Operations**: <10ms per file
- **Memory Growth**: <50MB during stress testing
- **Thread Leaks**: 0 threads remaining after tests

## Maintenance

When modifying HA_converse implementation:

1. Run full test suite: `./tests/run_tests.sh`
2. Check coverage report: `tests/htmlcov/index.html`
3. Add new tests for new functionality
4. Update tests if behavior changes intentionally
5. Ensure critical bug detection tests still pass

## Troubleshooting

### Common Test Failures

1. **Import Errors**: Ensure `ha_converse.py` is in the Python path
2. **Missing Dependencies**: Run `pip install -r tests/requirements_test.txt`
3. **Timeout Issues**: Increase timeout values in slow environments
4. **Resource Conflicts**: Ensure no other processes are using audio devices

### Debug Mode

Run tests with extra debugging:
```bash
pytest tests/ -v -s --tb=long --capture=no
```

## Contributing

When adding new tests:

1. Follow existing naming conventions
2. Add appropriate markers
3. Include docstrings explaining test purpose
4. Mock external dependencies appropriately
5. Test both success and failure paths
6. Add edge cases and boundary conditions

The test suite is designed to be comprehensive and catch any implementation bugs, ensuring the HA_converse application functions correctly as specified.