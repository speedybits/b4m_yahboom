#!/usr/bin/env python3
"""
Ollama Latency Test Script

Tests Ollama response times with different prompt sizes and configurations.
Helps optimize ROSIE conversational AI performance.
"""

import os
import sys
import time
import json
import requests
import statistics
from pathlib import Path
from dotenv import load_dotenv

# Load configuration
env_file = Path(__file__).parent / '.env.rosie.example'
if env_file.exists():
    load_dotenv(env_file, override=False)


class OllamaLatencyTester:
    """Test Ollama API latency with various configurations"""

    def __init__(self):
        self.ollama_url = 'http://localhost:11434/api/generate'
        self.ollama_model = os.getenv('OLLAMA_MODEL', 'qwen2.5:0.5b')
        self.ollama_temperature = float(os.getenv('OLLAMA_TEMPERATURE', '0.7'))
        self.ollama_max_tokens = int(os.getenv('OLLAMA_MAX_TOKENS', '100'))

        print(f"\n{'='*70}")
        print("Ollama Latency Test")
        print(f"{'='*70}")
        print(f"Model: {self.ollama_model}")
        print(f"Temperature: {self.ollama_temperature}")
        print(f"Max Tokens: {self.ollama_max_tokens}")
        print(f"URL: {self.ollama_url}")
        print(f"{'='*70}\n")

        # Check GPU availability
        self.check_gpu_availability()

    def check_gpu_availability(self):
        """Check if GPU/CUDA is available for Ollama"""
        print("GPU/CUDA Detection:")

        # Check nvidia-smi
        try:
            import subprocess
            result = subprocess.run(
                ['nvidia-smi', '--query-gpu=name,memory.total', '--format=csv,noheader'],
                capture_output=True,
                text=True,
                timeout=5
            )
            if result.returncode == 0 and result.stdout.strip():
                gpu_info = result.stdout.strip().split(',')
                print(f"  ✓ GPU detected: {gpu_info[0].strip()}")
                print(f"  ✓ GPU memory: {gpu_info[1].strip()}")
            else:
                print(f"  ✗ No NVIDIA GPU detected")
                return
        except FileNotFoundError:
            print(f"  ✗ nvidia-smi not found (no NVIDIA GPU driver)")
            return
        except Exception as e:
            print(f"  ⚠ GPU detection error: {e}")
            return

        # Check if Ollama is actually using GPU
        print("\nOllama GPU Usage:")
        try:
            # Get loaded models info
            response = requests.get('http://localhost:11434/api/ps', timeout=5)
            if response.status_code == 200:
                data = response.json()
                models = data.get('models', [])
                if models:
                    for model in models:
                        vram_usage = model.get('size_vram', 0)
                        if vram_usage > 0:
                            vram_gb = vram_usage / (1024**3)
                            print(f"  ✓ Model '{model['name']}' loaded in VRAM: {vram_gb:.2f} GB")
                        else:
                            print(f"  ⚠ Model '{model['name']}' NOT using GPU (running on CPU)")
                            print(f"    This will result in MUCH slower inference!")
                else:
                    print(f"  ℹ No models currently loaded")
        except Exception as e:
            print(f"  ⚠ Could not check Ollama GPU usage: {e}")

        print()

    def check_ollama_running(self):
        """Verify Ollama is running and accessible"""
        try:
            response = requests.get('http://localhost:11434/api/tags', timeout=5)
            if response.status_code == 200:
                print("✓ Ollama server is running")
                models = response.json().get('models', [])
                model_names = [m['name'] for m in models]
                print(f"✓ Available models: {', '.join(model_names)}")

                # Check if configured model is available
                if self.ollama_model in model_names:
                    print(f"✓ Configured model '{self.ollama_model}' is available")
                else:
                    print(f"⚠ Warning: Configured model '{self.ollama_model}' not found")
                    print(f"  Available models: {', '.join(model_names)}")
                return True
            else:
                print(f"✗ Ollama server returned status {response.status_code}")
                return False
        except requests.RequestException as e:
            print(f"✗ Cannot connect to Ollama server: {e}")
            print("  Make sure Ollama is running: 'ollama serve'")
            return False

    def test_single_request(self, prompt, max_tokens=None, description=""):
        """Test a single Ollama request and return timing info"""
        if max_tokens is None:
            max_tokens = self.ollama_max_tokens

        payload = {
            'model': self.ollama_model,
            'prompt': prompt,
            'temperature': self.ollama_temperature,
            'max_tokens': max_tokens,
            'stream': False
        }

        try:
            start_time = time.time()
            response = requests.post(self.ollama_url, json=payload, timeout=30)
            end_time = time.time()

            latency = (end_time - start_time) * 1000  # Convert to milliseconds

            if response.status_code == 200:
                result = response.json()
                response_text = result.get('response', '').strip()
                response_length = len(response_text.split())

                return {
                    'success': True,
                    'latency_ms': latency,
                    'response_length': response_length,
                    'response_text': response_text,
                    'prompt_length': len(prompt.split()),
                    'description': description
                }
            else:
                return {
                    'success': False,
                    'error': f"HTTP {response.status_code}",
                    'latency_ms': latency,
                    'description': description
                }

        except Exception as e:
            return {
                'success': False,
                'error': str(e),
                'description': description
            }

    def run_latency_tests(self):
        """Run comprehensive latency tests"""
        test_cases = [
            {
                'name': 'Short prompt (conversational)',
                'prompt': 'Hello, how are you today?',
                'max_tokens': 50,
                'iterations': 5
            },
            {
                'name': 'Medium prompt (typical ROSIE query)',
                'prompt': 'Human said: Rosie, what is the weather like today? Robot said: I can help with that. What city are you in?',
                'max_tokens': 100,
                'iterations': 5
            },
            {
                'name': 'Long prompt (with context)',
                'prompt': (
                    'Conversation history:\n'
                    'Human said: Rosie, I need to know about robot navigation.\n'
                    'Robot said: I can help you with robot navigation. What specific aspect are you interested in?\n'
                    'Human said: How does SLAM work?\n\n'
                    'Please respond to this conversation. Your primary goal is to keep the human '
                    'engaged and talking. Ask follow-up questions and maintain natural dialogue.'
                ),
                'max_tokens': 100,
                'iterations': 5
            },
            {
                'name': 'Maximum context (stress test)',
                'prompt': (
                    'Conversation history:\n'
                    'Human said: Rosie, tell me about autonomous robots.\n'
                    'Robot said: Autonomous robots are fascinating! They use sensors and algorithms.\n'
                    'Human said: What sensors do they use?\n'
                    'Robot said: Common sensors include LIDAR, cameras, IMU, and encoders.\n'
                    'Human said: How does LIDAR work?\n\n'
                    'Intelligence summary:\n'
                    'LIDAR (Light Detection and Ranging) uses laser pulses to measure distances. '
                    'It creates 3D point clouds of the environment for mapping and obstacle detection.\n\n'
                    'Please respond to this conversation. Your primary goal is to keep the human '
                    'engaged and talking. Ask follow-up questions, express curiosity, and maintain '
                    'natural dialogue.'
                ),
                'max_tokens': 150,
                'iterations': 3
            }
        ]

        results = []

        for test_case in test_cases:
            print(f"\n{'-'*70}")
            print(f"Test: {test_case['name']}")
            print(f"Prompt length: {len(test_case['prompt'].split())} words")
            print(f"Max tokens: {test_case['max_tokens']}")
            print(f"Iterations: {test_case['iterations']}")
            print(f"{'-'*70}")

            latencies = []
            successful = 0

            for i in range(test_case['iterations']):
                result = self.test_single_request(
                    test_case['prompt'],
                    test_case['max_tokens'],
                    f"{test_case['name']} - iteration {i+1}"
                )

                if result['success']:
                    latencies.append(result['latency_ms'])
                    successful += 1
                    print(f"  [{i+1}/{test_case['iterations']}] "
                          f"✓ {result['latency_ms']:.0f}ms "
                          f"({result['response_length']} words)")
                else:
                    print(f"  [{i+1}/{test_case['iterations']}] "
                          f"✗ Failed: {result.get('error', 'Unknown error')}")

                # Small delay between requests
                time.sleep(0.5)

            if latencies:
                avg_latency = statistics.mean(latencies)
                min_latency = min(latencies)
                max_latency = max(latencies)
                std_dev = statistics.stdev(latencies) if len(latencies) > 1 else 0

                print(f"\nResults:")
                print(f"  Success rate: {successful}/{test_case['iterations']}")
                print(f"  Average latency: {avg_latency:.0f}ms")
                print(f"  Min latency: {min_latency:.0f}ms")
                print(f"  Max latency: {max_latency:.0f}ms")
                print(f"  Std deviation: {std_dev:.0f}ms")

                # ROSIE requirement check
                if avg_latency < 1000:
                    print(f"  ✓ PASSES ROSIE requirement (<1 second)")
                else:
                    print(f"  ✗ FAILS ROSIE requirement (<1 second)")

                results.append({
                    'test_name': test_case['name'],
                    'avg_latency_ms': avg_latency,
                    'min_latency_ms': min_latency,
                    'max_latency_ms': max_latency,
                    'std_dev_ms': std_dev,
                    'success_rate': f"{successful}/{test_case['iterations']}",
                    'meets_requirement': avg_latency < 1000
                })
            else:
                print(f"\n✗ All iterations failed for this test")

        return results

    def print_summary(self, results):
        """Print comprehensive test summary"""
        print(f"\n{'='*70}")
        print("SUMMARY")
        print(f"{'='*70}\n")

        if not results:
            print("No successful tests to summarize")
            return

        print(f"{'Test Name':<40} {'Avg (ms)':<12} {'Min (ms)':<12} {'Max (ms)':<12} {'Status'}")
        print(f"{'-'*70}")

        for result in results:
            status = "✓ PASS" if result['meets_requirement'] else "✗ FAIL"
            print(f"{result['test_name']:<40} "
                  f"{result['avg_latency_ms']:>8.0f}    "
                  f"{result['min_latency_ms']:>8.0f}    "
                  f"{result['max_latency_ms']:>8.0f}    "
                  f"{status}")

        # Overall assessment
        all_pass = all(r['meets_requirement'] for r in results)
        print(f"\n{'-'*70}")
        if all_pass:
            print("✓ All tests meet ROSIE requirement (<1 second)")
        else:
            failing = [r['test_name'] for r in results if not r['meets_requirement']]
            print(f"⚠ Some tests exceed ROSIE requirement:")
            for name in failing:
                print(f"  - {name}")

        print(f"{'='*70}\n")


def main():
    """Main entry point"""
    tester = OllamaLatencyTester()

    # Check if Ollama is running
    if not tester.check_ollama_running():
        print("\n✗ Cannot proceed without Ollama server")
        print("  Start Ollama: 'ollama serve'")
        sys.exit(1)

    print("\nStarting latency tests...\n")

    # Run tests
    results = tester.run_latency_tests()

    # Print summary
    tester.print_summary(results)


if __name__ == '__main__':
    main()
