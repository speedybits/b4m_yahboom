# Running HA_converse Unit Tests

This document provides instructions for running the comprehensive unit test suite for the HA_converse speech-to-text application.

## Overview

The HA_converse test suite is designed to:
- **Detect any bugs** in the implementation, including the randomly inserted bug
- **Catch placeholder or mocked code** in the actual implementation
- **Verify complete functionality** matches the specification
- **Test thread safety** and resource management
- **Validate precise timing** and behavior requirements

## Quick Start

### Automated Test Runner (Recommended)

The easiest way to run all HA_converse tests with comprehensive bug detection:

```bash
# Make sure you're in the project root directory
cd /home/mike/projects/b4m_yahboom

# Run the complete HA_converse test suite (NOT the robot tests)
./run_ha_converse_tests.sh
```

**Important:** Do NOT use `./tests/run_tests.sh` - that runs robot/ROS integration tests. Use `./run_ha_converse_tests.sh` for speech-to-text tests only.

This script will:
1. Install test dependencies if needed
2. Run unit tests
3. Run integration tests
4. Run stress tests
5. Execute critical bug detection tests
6. Generate coverage reports
7. Provide a detailed summary

## Manual Test Execution

### Prerequisites

Install test dependencies:
```bash
pip install -r tests/requirements_test.txt
```

### Test Categories

#### 1. Unit Tests (Core Functionality)
```bash
# Run all unit tests
pytest tests/test_ha_converse.py -v

# Run specific test classes
pytest tests/test_ha_converse.py::TestSpeechRecognition -v
pytest tests/test_ha_converse.py::TestB4MAPIIntegration -v
pytest tests/test_ha_converse.py::TestPiperTTSIntegration -v
pytest tests/test_ha_converse.py::TestThreadManagement -v
```

#### 2. Critical Bug Detection Tests
```bash
# Run the most important tests that detect the inserted bug
pytest tests/test_ha_converse.py::TestCriticalBugDetection -v

# Run individual critical tests
pytest tests/test_ha_converse.py::TestCriticalBugDetection::test_no_placeholder_code_detection -v
pytest tests/test_ha_converse.py::TestCriticalBugDetection::test_real_api_calls_not_mocked -v
pytest tests/test_ha_converse.py::TestCriticalBugDetection::test_thread_shutdown_not_ignored -v
```

#### 3. Integration Tests
```bash
# Run complete workflow tests
pytest tests/test_ha_converse_integration.py -v

# Run specific integration workflows
pytest tests/test_ha_converse_integration.py::TestCompleteWorkflows::test_full_keyword_workflow_integration -v
pytest tests/test_ha_converse_integration.py::TestCompleteWorkflows::test_full_interactive_workflow_integration -v
```

#### 4. Stress Tests
```bash
# Run performance and edge case tests
pytest tests/test_ha_converse_stress.py -v

# Run specific stress tests
pytest tests/test_ha_converse_stress.py::TestHAConverseStress::test_concurrent_buffer_operations -v
pytest tests/test_ha_converse_stress.py::TestHAConverseStress::test_memory_usage_under_load -v
```

### Using Test Markers

Tests are organized with markers for selective execution:

```bash
# Run only unit tests
pytest tests/ -m unit -v

# Run only integration tests
pytest tests/ -m integration -v

# Run only stress tests
pytest tests/ -m stress -v

# Run tests related to B4M API
pytest tests/ -m b4m -v

# Run tests related to Piper TTS
pytest tests/ -m piper -v

# Exclude slow tests
pytest tests/ -m "not slow" -v
```

## Coverage Analysis

### Generate Coverage Report
```bash
# Run tests with coverage
pytest tests/ --cov=ha_converse --cov-report=html --cov-report=term-missing

# View HTML coverage report
open tests/htmlcov/index.html  # macOS
xdg-open tests/htmlcov/index.html  # Linux
```

### Coverage Requirements
- **Minimum**: 85% overall coverage
- **Critical paths**: 100% coverage required for buffer management, file operations, thread sync

## Test Output Interpretation

### ✅ Success Indicators
```
✅ ALL CRITICAL TESTS PASSED
   No bugs detected in implementation
   Implementation appears to be complete and functional
```

### ❌ Bug Detection
```
❌ CRITICAL BUGS DETECTED!
   Failed tests:
   - test_no_placeholder_code_detection
   - test_real_api_calls_not_mocked

🐛 BUG ANALYSIS:
   These test failures indicate:
   - Placeholder/mock code in implementation
   - Simulated behavior instead of real functionality
```

## Specific Test Commands

### Environment Setup Tests
```bash
# Test environment variable loading
pytest tests/test_ha_converse_integration.py::TestEnvironmentIntegration::test_environment_variable_loading -v

# Test missing environment variables
pytest tests/test_ha_converse_integration.py::TestEnvironmentIntegration::test_missing_environment_variables_handling -v
```

### Thread Safety Tests
```bash
# Test thread synchronization
pytest tests/test_ha_converse.py::TestThreadManagement::test_thread_safe_voice_activity_flag -v

# Test shutdown coordination
pytest tests/test_ha_converse.py::TestThreadManagement::test_shutdown_event_coordination -v

# Test interrupt response time
pytest tests/test_ha_converse.py::TestThreadManagement::test_interrupt_response_time -v
```

### File Management Tests
```bash
# Test file creation and naming
pytest tests/test_ha_converse.py::TestFileManagement::test_conversation_file_naming_format -v

# Test 1:1 file mapping
pytest tests/test_ha_converse.py::TestFileManagement::test_response_file_1to1_mapping -v

# Test FIFO queue processing
pytest tests/test_ha_converse.py::TestFileManagement::test_file_queue_fifo_processing -v
```

### API Integration Tests
```bash
# Test B4M API calls
pytest tests/test_ha_converse.py::TestB4MAPIIntegration::test_b4m_api_real_http_calls -v

# Test quest polling system
pytest tests/test_ha_converse.py::TestB4MAPIIntegration::test_b4m_quest_polling_system -v

# Test rate limiting
pytest tests/test_ha_converse.py::TestB4MAPIIntegration::test_b4m_rate_limiting_handling -v
```

## Debug Mode

Run tests with detailed debugging information:

```bash
# Maximum verbosity with full tracebacks
pytest tests/ -v -s --tb=long --capture=no

# Stop on first failure
pytest tests/ -x -v

# Run specific test with debugging
pytest tests/test_ha_converse.py::TestCriticalBugDetection::test_no_placeholder_code_detection -v -s --tb=long
```

## Test Configuration

### Custom pytest.ini Settings
```bash
# Run with custom markers
pytest tests/ --strict-markers -v

# Custom timeout (default is 30 seconds)
pytest tests/ --timeout=60 -v

# Parallel execution (if pytest-xdist installed)
pytest tests/ -n auto -v
```

### Environment Variables for Testing

Set these before running tests if needed:
```bash
export B4M_API_KEY="your_test_api_key"
export B4M_ROSIE_ID="your_test_rosie_id"
export B4M_USER_ID="your_test_user_id"
export PIPER_MODEL_PATH="/path/to/test/model.onnx"
export PIPER_CONFIG_PATH="/path/to/test/config.json"
```

## Common Issues and Solutions

### Import Errors
```bash
# If ha_converse module not found
export PYTHONPATH="${PYTHONPATH}:/home/mike/projects/b4m_yahboom"

# Or run from project root
cd /home/mike/projects/b4m_yahboom
pytest tests/
```

### Missing Dependencies
```bash
# Install all test requirements
pip install -r tests/requirements_test.txt

# Install specific missing packages
pip install pytest pytest-cov psutil
```

### Permission Issues
```bash
# Make test runner executable
chmod +x tests/run_tests.sh

# Fix file permissions if needed
chmod 644 tests/*.py
```

### Audio Device Conflicts
```bash
# Tests should mock audio devices, but if conflicts occur:
# Stop any audio applications
# Run tests with audio device mocking verified:
pytest tests/ -k "not audio" -v  # Skip audio-specific tests if needed
```

## Test Results Analysis

### Expected Results When Bug is Present

The tests are designed to catch the randomly inserted bug. Look for:

1. **Failed Critical Tests**: One or more tests in `TestCriticalBugDetection` will fail
2. **Specific Error Messages**: Clear indication of what type of bug was detected
3. **Code Location**: Tests will point to the specific area with the problem

### Expected Results When Implementation is Correct

1. **All Tests Pass**: ✅ Green checkmarks for all test categories
2. **High Coverage**: >85% code coverage achieved
3. **No Resource Leaks**: Memory and thread usage remains stable
4. **Fast Execution**: Most tests complete in <30 seconds total

## Continuous Testing

### Watch Mode (if pytest-watch installed)
```bash
pip install pytest-watch
ptw tests/ -- -v
```

### Pre-commit Testing
```bash
# Add to .git/hooks/pre-commit
#!/bin/bash
./tests/run_tests.sh
```

## Performance Monitoring

### Memory Usage Monitoring
```bash
# Run stress tests with memory monitoring
pytest tests/test_ha_converse_stress.py::TestHAConverseStress::test_memory_usage_under_load -v
```

### Thread Leak Detection
```bash
# Run thread management tests
pytest tests/test_ha_converse_stress.py::TestHAConverseStress::test_thread_creation_destruction_stress -v
```

## Final Verification

To ensure the implementation is ready:

```bash
# Run complete test suite
./tests/run_tests.sh

# Check exit code
echo $?  # Should be 0 for success

# Generate final coverage report
pytest tests/ --cov=ha_converse --cov-report=html --cov-fail-under=85
```

## Next Steps After Testing

1. **If tests pass**: Implementation is ready for production use
2. **If tests fail**: Review failed test output to identify and fix bugs
3. **Coverage review**: Check `tests/htmlcov/index.html` for uncovered code
4. **Performance review**: Monitor resource usage during actual operation

The test suite is comprehensive and designed to catch any implementation issues, ensuring the HA_converse application works correctly according to the specification.