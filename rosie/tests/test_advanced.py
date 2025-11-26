#!/usr/bin/env python3
"""
ROSIE Advanced Conversation Test Runner

Tests ROSIE's conversational naturalness against human responses:
1. Piper speaks question
2. Whisper captures human answer (golden reference)
3. ROSIE responds to same question in text mode
4. LLM scores ROSIE vs human for human-likeness
"""

import subprocess
import time
import json
import argparse
import signal
import sys
from pathlib import Path
from datetime import datetime
from typing import Dict, List, Optional
import os

# Import our modules
from human_likeness_scorer import HumanLikenessScorer


class AdvancedROSIETester:
    """Advanced test runner for ROSIE conversational naturalness"""

    def __init__(self, rosie_dir: Path, use_cache: bool = False):
        """
        Initialize tester

        Args:
            rosie_dir: Path to rosie root directory
            use_cache: If True, use cached golden references instead of recording new ones
        """
        self.rosie_dir = Path(rosie_dir)
        self.tests_dir = self.rosie_dir / 'tests'
        self.run_script = self.rosie_dir / 'scripts' / 'run.sh'
        self.results_dir = self.tests_dir / 'advanced_results'
        self.results_dir.mkdir(exist_ok=True)

        self.golden_refs_file = self.results_dir / 'golden_references.json'
        self.use_cache = use_cache

        # Initialize components
        self.scorer = HumanLikenessScorer()
        self.rosie_process = None

        # Load or initialize golden references
        self.golden_refs = self._load_golden_references()

    def _load_golden_references(self) -> Dict:
        """Load cached golden references if they exist"""
        if self.golden_refs_file.exists():
            with open(self.golden_refs_file) as f:
                return json.load(f)
        return {}

    def _save_golden_references(self):
        """Save golden references to cache"""
        with open(self.golden_refs_file, 'w') as f:
            json.dump(self.golden_refs, f, indent=2)

    def speak_with_piper(self, text: str):
        """
        Speak text using Piper TTS

        Args:
            text: Text to speak
        """
        print(f"[PIPER] Speaking: {text}")

        # Use Piper with same model paths as ROSIE
        piper_model = os.environ.get('PIPER_MODEL_PATH')
        piper_config = os.environ.get('PIPER_CONFIG_PATH')

        if not piper_model:
            print("[ERROR] PIPER_MODEL_PATH not set")
            return

        try:
            # Create temporary file for Piper output
            temp_wav = '/tmp/piper_question.wav'

            # Run Piper to generate WAV (same format as ROSIE uses)
            if piper_config:
                piper_cmd = f'echo "{text}" | piper --model {piper_model} --config {piper_config} --output_file {temp_wav}'
            else:
                piper_cmd = f'echo "{text}" | piper --model {piper_model} --output_file {temp_wav}'
            subprocess.run(piper_cmd, shell=True, check=True)

            # Play WAV with aplay
            subprocess.run(['aplay', '-q', temp_wav], check=True)

            print("[PIPER] Playback complete")

        except Exception as e:
            print(f"[ERROR] Piper TTS failed: {e}")

    def capture_with_whisper(self, silence_duration: float = 3.0, max_duration: int = 30) -> Optional[str]:
        """
        Capture audio with VAD and transcribe with Whisper
        Records until 3 seconds of silence after speech ends

        Args:
            silence_duration: Seconds of silence before stopping (default: 3.0)
            max_duration: Maximum recording duration in seconds (default: 30)

        Returns:
            Transcribed text or None on error
        """
        print(f"[WHISPER] Recording (will stop {silence_duration}s after you finish speaking)...")
        print("[WHISPER] Please speak your answer now...")

        try:
            import pyaudio
            import wave
            import numpy as np

            # Audio parameters (same as ROSIE)
            CHUNK = 1024
            FORMAT = pyaudio.paInt16
            CHANNELS = 1
            RATE = 16000

            # VAD parameters
            SILENCE_THRESHOLD = 500  # Energy threshold for silence detection
            silence_chunks = int(silence_duration * RATE / CHUNK)  # Number of silent chunks before stopping

            p = pyaudio.PyAudio()

            # Find the correct device (plughw:3,0)
            device_index = None
            for i in range(p.get_device_count()):
                dev = p.get_device_info_by_index(i)
                if 'plughw:3,0' in str(dev.get('name', '')):
                    device_index = i
                    break

            # Open stream
            stream = p.open(
                format=FORMAT,
                channels=CHANNELS,
                rate=RATE,
                input=True,
                input_device_index=device_index,
                frames_per_buffer=CHUNK
            )

            print("[WHISPER] Listening...")
            frames = []
            silent_chunks_count = 0
            speech_detected = False
            max_chunks = int(max_duration * RATE / CHUNK)

            for i in range(max_chunks):
                data = stream.read(CHUNK, exception_on_overflow=False)
                frames.append(data)

                # Calculate energy/volume of this chunk
                audio_data = np.frombuffer(data, dtype=np.int16)
                energy = np.abs(audio_data).mean()

                if energy > SILENCE_THRESHOLD:
                    # Speech detected
                    speech_detected = True
                    silent_chunks_count = 0
                elif speech_detected:
                    # Silence detected after speech
                    silent_chunks_count += 1
                    if silent_chunks_count >= silence_chunks:
                        print(f"[WHISPER] {silence_duration}s of silence detected, stopping...")
                        break

            # Stop and close stream
            stream.stop_stream()
            stream.close()
            p.terminate()

            # Save to WAV file
            temp_wav = '/tmp/whisper_answer.wav'
            wf = wave.open(temp_wav, 'wb')
            wf.setnchannels(CHANNELS)
            wf.setsampwidth(p.get_sample_size(FORMAT))
            wf.setframerate(RATE)
            wf.writeframes(b''.join(frames))
            wf.close()

            print("[WHISPER] Recording complete, transcribing...")

            # Transcribe with faster-whisper
            from faster_whisper import WhisperModel

            whisper_model = WhisperModel(
                "base.en",
                device="cuda",
                compute_type="float16"
            )

            segments, info = whisper_model.transcribe(
                temp_wav,
                language="en",
                vad_filter=True,
                vad_parameters=dict(
                    min_silence_duration_ms=500,
                    speech_pad_ms=200
                )
            )

            # Combine segments
            transcription = ' '.join([seg.text.strip() for seg in segments])

            print(f"[WHISPER] Transcribed: {transcription}")
            return transcription

        except Exception as e:
            print(f"[ERROR] Whisper transcription failed: {e}")
            return None

    def get_golden_reference(self, question_id: str, question: str) -> Optional[str]:
        """
        Get golden reference response (from cache or by recording)

        Args:
            question_id: Unique question ID
            question: Question text

        Returns:
            Human response text or None on error
        """
        # Check cache first
        if self.use_cache and question_id in self.golden_refs:
            print(f"[CACHE] Using cached golden reference for {question_id}")
            return self.golden_refs[question_id]

        # Record new golden reference
        print(f"\n{'='*70}")
        print(f"RECORDING GOLDEN REFERENCE FOR: {question_id}")
        print(f"{'='*70}")
        print(f"Question: {question}")
        print("\nPiper will speak the question, then you have 5 seconds to respond.")
        print("Press ENTER when ready...")
        input()

        # Speak question
        self.speak_with_piper(question)

        # Brief pause
        time.sleep(1)

        # Capture answer
        human_response = self.capture_with_whisper()

        if human_response:
            # Save to cache
            self.golden_refs[question_id] = human_response
            self._save_golden_references()
            print(f"[CACHE] Saved golden reference for {question_id}")
            return human_response
        else:
            print(f"[ERROR] Failed to capture golden reference for {question_id}")
            return None

    def start_rosie(self):
        """Start ROSIE in text-only mode"""
        print("[TEST] Starting ROSIE in --text-only mode...")

        cmd = [str(self.run_script), '--no-web', '--text-only']

        self.rosie_process = subprocess.Popen(
            cmd,
            stdin=subprocess.PIPE,
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            text=True,
            bufsize=1,
            cwd=str(self.rosie_dir.parent)
        )

        # Wait for ROSIE to be ready
        print("[TEST] Waiting for ROSIE to initialize...")
        startup_output = []

        while True:
            line = self.rosie_process.stdout.readline()
            if not line:
                break

            startup_output.append(line)

            if "You:" in line or "Type your messages" in line:
                print("[TEST] ROSIE ready for input")
                break

            if len(startup_output) > 1000:
                raise RuntimeError("ROSIE startup timeout")

        return True

    def send_to_rosie(self, question: str) -> Optional[str]:
        """
        Send question to ROSIE and capture response

        Args:
            question: Question text

        Returns:
            ROSIE's response or None on error
        """
        print(f"[TEST] Sending to ROSIE: {question}")

        if not self.rosie_process or not self.rosie_process.stdin:
            raise RuntimeError("ROSIE process not running")

        # Send input
        self.rosie_process.stdin.write(question + '\n')
        self.rosie_process.stdin.flush()

        # Capture response
        response_text = None
        start_time = time.time()
        timeout = 30

        while time.time() - start_time < timeout:
            line = self.rosie_process.stdout.readline()
            if not line:
                time.sleep(0.1)
                continue

            line_stripped = line.rstrip()

            # Look for OLLAMA response
            if '[OLLAMA] Response text:' in line:
                import re
                match = re.search(r"Response text: '(.+)'", line)
                if match:
                    response_text = match.group(1)
                    print(f"[TEST] Got ROSIE response: {response_text[:80]}...")

            # Check for completion
            if '→ LISTENING' in line and response_text:
                break

        if not response_text:
            print("[TEST] Warning: No response from ROSIE")
            return None

        return response_text

    def stop_rosie(self):
        """Stop ROSIE process"""
        if self.rosie_process:
            print("\n[TEST] Stopping ROSIE...")
            try:
                self.rosie_process.send_signal(signal.SIGINT)
                self.rosie_process.wait(timeout=5)
            except:
                self.rosie_process.kill()
                self.rosie_process.wait()
            print("[TEST] ROSIE stopped")

    def run_scenario(self, scenario: Dict) -> Optional[Dict]:
        """
        Run a single test scenario

        Args:
            scenario: Scenario dict from test_advanced_scenarios.json

        Returns:
            Test result dict or None on error
        """
        question_id = scenario['id']
        question = scenario['question']
        expected_traits = scenario.get('expected_traits', [])

        print(f"\n{'='*70}")
        print(f"Testing: {question_id}")
        print(f"Question: {question}")
        print(f"Category: {scenario.get('category', 'N/A')}")
        print(f"{'='*70}")

        # Get golden reference
        human_response = self.get_golden_reference(question_id, question)
        if not human_response:
            print(f"[ERROR] Skipping {question_id} - no golden reference")
            return None

        # Get ROSIE response
        rosie_response = self.send_to_rosie(question)
        if not rosie_response:
            print(f"[ERROR] Skipping {question_id} - no ROSIE response")
            return None

        # Score with LLM
        print(f"[SCORING] Comparing responses...")
        scores = self.scorer.score_response(
            question,
            human_response,
            rosie_response,
            expected_traits
        )

        # Display scoring results immediately
        winner_label = "ROSIE" if scores.get('rosie_won', False) else "Human"
        print(f"\n[SCORING] Results:")
        print(f"  Human Response: \"{human_response}\"")
        print(f"  ROSIE Response: \"{rosie_response}\"")
        print(f"  Winner: {winner_label}")
        print(f"  Reasoning: {scores.get('reasoning', 'N/A')}")
        print(f"{'='*70}\n")

        # Build result
        result = {
            'id': question_id,
            'question': question,
            'category': scenario.get('category'),
            'expected_traits': expected_traits,
            'human_response': human_response,
            'rosie_response': rosie_response,
            'scores': scores,
            'timestamp': datetime.now().isoformat()
        }

        return result

    def run_all_scenarios(self, scenarios_file: Path, scenario_ids: List[str] = None) -> Dict:
        """
        Run all or selected test scenarios

        Args:
            scenarios_file: Path to test_advanced_scenarios.json
            scenario_ids: Optional list of specific scenario IDs to run

        Returns:
            Complete test results dict
        """
        # Load scenarios
        with open(scenarios_file) as f:
            scenarios_data = json.load(f)

        scenarios = scenarios_data['scenarios']

        # Filter scenarios if requested
        if scenario_ids:
            scenarios = [s for s in scenarios if s['id'] in scenario_ids]

        print(f"\n{'='*70}")
        print(f"ROSIE ADVANCED CONVERSATIONAL TESTING")
        print(f"{'='*70}")
        print(f"Total scenarios: {len(scenarios)}")
        print(f"Use cache: {self.use_cache}")
        print(f"{'='*70}")

        all_results = []

        # Start ROSIE once for all tests
        self.start_rosie()

        try:
            for idx, scenario in enumerate(scenarios):
                if idx > 0:
                    print(f"\n[TEST] Moving to next scenario...")
                    time.sleep(1)

                result = self.run_scenario(scenario)
                if result:
                    all_results.append(result)

        finally:
            self.stop_rosie()

        # Calculate aggregate scores (win/loss percentage)
        if all_results:
            # Count wins (ROSIE sounded more human than golden reference)
            rosie_wins = sum(1 for r in all_results if r['scores'].get('rosie_won', False))
            total_tests = len(all_results)
            win_percentage = (rosie_wins / total_tests * 100) if total_tests > 0 else 0

            avg_scores = {
                'rosie_wins': rosie_wins,
                'human_wins': total_tests - rosie_wins,
                'total_tests': total_tests,
                'win_percentage': round(win_percentage, 2)
            }
        else:
            avg_scores = {
                'rosie_wins': 0,
                'human_wins': 0,
                'total_tests': 0,
                'win_percentage': 0
            }

        # Build final results
        final_results = {
            'test_run': datetime.now().isoformat(),
            'test_type': 'advanced_conversational',
            'scenarios_tested': len(all_results),
            'average_scores': {k: round(v, 2) for k, v in avg_scores.items()},
            'results': all_results
        }

        return final_results


def main():
    """Main entry point"""
    parser = argparse.ArgumentParser(
        description='ROSIE Advanced Conversational Test Runner',
        formatter_class=argparse.RawDescriptionHelpFormatter
    )

    parser.add_argument('--all', action='store_true',
                       help='Run all test scenarios')
    parser.add_argument('--scenario', type=str, action='append',
                       help='Run specific scenario by ID (can be used multiple times)')
    parser.add_argument('--output', type=str, default=None,
                       help='Output file for results (default: advanced_results/test_results_YYYYMMDD_HHMMSS.json)')
    parser.add_argument('--scenarios-file', type=str, default=None,
                       help='Path to test scenarios JSON file')
    parser.add_argument('--use-cache', action='store_true',
                       help='Use cached golden references instead of recording new ones')

    args = parser.parse_args()

    # Find rosie directory
    rosie_dir = Path(__file__).parent.parent
    scenarios_file = Path(args.scenarios_file) if args.scenarios_file else rosie_dir / 'tests' / 'test_advanced_scenarios.json'

    if not scenarios_file.exists():
        print(f"ERROR: Scenarios file not found: {scenarios_file}")
        sys.exit(1)

    # Default output filename
    if not args.output:
        timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
        args.output = f"advanced_results/test_results_{timestamp}.json"

    print("=" * 70)
    print("ROSIE Advanced Conversational Test Runner")
    print("=" * 70)
    print(f"Scenarios file: {scenarios_file}")
    print(f"Output file: {args.output}")
    print(f"Use cache: {args.use_cache}")

    # Initialize tester
    tester = AdvancedROSIETester(rosie_dir, use_cache=args.use_cache)

    # Run tests
    if args.all:
        results = tester.run_all_scenarios(scenarios_file)
    elif args.scenario:
        results = tester.run_all_scenarios(scenarios_file, scenario_ids=args.scenario)
    else:
        print("ERROR: Must specify --all or --scenario")
        sys.exit(1)

    # Save results
    output_path = tester.tests_dir / args.output
    output_path.parent.mkdir(exist_ok=True, parents=True)

    with open(output_path, 'w') as f:
        json.dump(results, f, indent=2)

    print(f"\n{'='*70}")
    print("Test Results Summary (Binary Vote)")
    print(f"{'='*70}")
    print(f"Scenarios tested: {results['scenarios_tested']}")

    scores = results['average_scores']
    print(f"\nResults:")
    print(f"  ROSIE Wins: {scores.get('rosie_wins', 0)} (ROSIE sounded more human)")
    print(f"  Human Wins: {scores.get('human_wins', 0)} (Human sounded more human)")
    print(f"  Total Tests: {scores.get('total_tests', 0)}")
    print(f"  Win Percentage: {scores.get('win_percentage', 0)}%")

    print(f"\nResults saved to: {output_path}")


if __name__ == '__main__':
    main()
