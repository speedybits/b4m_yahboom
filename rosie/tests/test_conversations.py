#!/usr/bin/env python3
"""
ROSIE Conversation Test Runner

Programmatically tests ROSIE conversations by:
1. Spawning ROSIE in --text-only mode
2. Sending test inputs
3. Capturing responses
4. Scoring responses (accuracy, depth, topic transitions)
5. Generating JSON results

Usage:
    python3 test_conversations.py --all
    python3 test_conversations.py --scenario "Factual Accuracy"
    python3 test_conversations.py --output results.json
"""

import subprocess
import time
import json
import argparse
import re
from pathlib import Path
from datetime import datetime
from typing import Dict, List, Optional
import signal
import sys

# Import our modules
from kb_parser import KnowledgeBaseParser
from scoring import ROSIEScorer


class ROSIETester:
    """Test runner for ROSIE conversations"""

    def __init__(self, rosie_dir: Path):
        """
        Initialize tester

        Args:
            rosie_dir: Path to rosie root directory
        """
        self.rosie_dir = Path(rosie_dir)
        self.kb_dir = self.rosie_dir / 'knowledge_base'
        self.run_script = self.rosie_dir / 'scripts' / 'run.sh'

        # Initialize components
        self.kb_parser = KnowledgeBaseParser(self.kb_dir)
        self.scorer = ROSIEScorer(self.kb_parser)

        self.rosie_process = None

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
            bufsize=1,  # Line buffered
            cwd=str(self.rosie_dir.parent)  # Run from b4m_yahboom directory
        )

        # Wait for ROSIE to start and reach the input prompt
        print("[TEST] Waiting for ROSIE to initialize...")
        startup_output = []

        while True:
            line = self.rosie_process.stdout.readline()
            if not line:
                break

            startup_output.append(line)
            print(f"[ROSIE] {line.rstrip()}")

            # Check if we've reached the input prompt
            if "You:" in line or "Type your messages" in line:
                print("[TEST] ROSIE ready for input")
                break

            # Timeout after 60 seconds
            if len(startup_output) > 1000:
                raise RuntimeError("ROSIE startup timeout")

        return True

    def send_input(self, user_input: str):
        """Send input to ROSIE"""
        print(f"\n[TEST] Sending: {user_input}")

        if not self.rosie_process or not self.rosie_process.stdin:
            raise RuntimeError("ROSIE process not running")

        # Send input
        self.rosie_process.stdin.write(user_input + '\n')
        self.rosie_process.stdin.flush()

    def capture_response(self, timeout=30) -> Optional[str]:
        """
        Capture ROSIE's response

        Args:
            timeout: Maximum seconds to wait for response

        Returns:
            ROSIE's response text, or None if timeout
        """
        response_text = None
        start_time = time.time()

        while time.time() - start_time < timeout:
            line = self.rosie_process.stdout.readline()
            if not line:
                time.sleep(0.1)
                continue

            line_stripped = line.rstrip()

            # Print all debug output
            if '[' in line and ']' in line:
                print(f"[DEBUG] {line_stripped}")

                # Look for the OLLAMA response text in debug output
                if '[OLLAMA] Response text:' in line:
                    # Extract text between quotes
                    match = re.search(r"Response text: '(.+)'", line)
                    if match:
                        response_text = match.group(1)
                        print(f"[TEST] Captured response from OLLAMA: {response_text[:80]}...")

                # Check for state transition back to LISTENING (indicates response complete)
                if '→ LISTENING' in line and response_text:
                    # Response captured and complete
                    break
                continue

            # Also check for non-debug ROSIE output (fallback)
            if "ROSIE:" in line and not response_text:
                response_text = line.split("ROSIE:", 1)[1].strip()
                print(f"[TEST] Captured response from output: {response_text[:80]}...")

            # Check for next prompt (fallback completion indicator)
            if "You:" in line and response_text:
                break

        if not response_text:
            print("[TEST] Warning: No response captured")
            return None

        print(f"[TEST] Final response: {response_text[:100]}...")
        return response_text

    def stop_rosie(self):
        """Stop ROSIE process"""
        if self.rosie_process:
            print("\n[TEST] Stopping ROSIE...")
            try:
                # Send CTRL+C
                self.rosie_process.send_signal(signal.SIGINT)
                self.rosie_process.wait(timeout=5)
            except:
                self.rosie_process.kill()
                self.rosie_process.wait()
            print("[TEST] ROSIE stopped")

    def run_scenario(self, scenario: Dict) -> List[Dict]:
        """
        Run a test scenario

        Args:
            scenario: Scenario dict from test_scenarios.json

        Returns:
            List of test results
        """
        print(f"\n{'='*70}")
        print(f"Running Scenario: {scenario['name']}")
        print(f"Description: {scenario.get('description', 'N/A')}")
        print(f"{'='*70}")

        results = []
        conversation_history = []
        previous_topic = None

        for idx, conv in enumerate(scenario['conversations']):
            turn_number = conv.get('turn', idx + 1)
            user_input = conv['input']
            current_topic = conv.get('topic', None)

            # Send input to ROSIE
            self.send_input(user_input)

            # Capture response
            response = self.capture_response()

            if not response:
                print(f"[TEST] ERROR: No response for input: {user_input}")
                continue

            # Score the response
            scores = {}

            # Accuracy scoring (for factual questions)
            if 'expected_facts' in conv:
                accuracy_score = self.scorer.score_accuracy(
                    response,
                    conv['expected_facts'],
                    conv.get('expected_source')
                )
                scores.update(accuracy_score)

            # Depth scoring
            if 'expected_depth' in conv:
                depth_score = self.scorer.score_depth(
                    response,
                    conv['expected_depth'],
                    turn_number,
                    current_topic
                )
                scores.update(depth_score)

            # Topic transition scoring
            if 'scoring' in conv and conv['scoring'].get('topic_change'):
                if previous_topic:
                    transition_score = self.scorer.score_topic_transition(
                        response,
                        previous_topic,
                        current_topic,
                        user_input,
                        conversation_history
                    )
                    scores.update(transition_score)

            # Store result
            result = {
                'scenario': scenario['name'],
                'turn': turn_number,
                'input': user_input,
                'response': response,
                'scores': scores,
                'test_type': scenario.get('test_type', 'unknown')
            }

            results.append(result)
            conversation_history.append(result)
            previous_topic = current_topic

            # Brief pause between turns
            time.sleep(0.5)

        return results

    def run_all_scenarios(self, scenarios_file: Path) -> Dict:
        """
        Run all test scenarios

        Args:
            scenarios_file: Path to test_scenarios.json

        Returns:
            Complete test results dict
        """
        # Load scenarios
        with open(scenarios_file) as f:
            scenarios_data = json.load(f)

        all_results = []

        # Start ROSIE once for all tests
        # No need to restart between scenarios since:
        # 1. Each test starts fresh (conversation history cleared on startup)
        # 2. Text-only mode is stateless
        # 3. Much faster to keep ROSIE running
        self.start_rosie()

        try:
            # Run each scenario without restarting
            for idx, scenario in enumerate(scenarios_data['scenarios']):
                if idx > 0:
                    # Brief pause between scenarios
                    print(f"\n[TEST] Moving to next scenario (keeping ROSIE running)...")
                    time.sleep(1)

                scenario_results = self.run_scenario(scenario)
                all_results.extend(scenario_results)

        finally:
            self.stop_rosie()

        # Calculate composite scores
        composite_scores = self.scorer.calculate_composite_score(all_results)

        # Build final results
        final_results = {
            'test_run': datetime.now().isoformat(),
            'overall_scores': composite_scores,
            'results': all_results
        }

        return final_results


def main():
    """Main entry point"""
    parser = argparse.ArgumentParser(
        description='ROSIE Conversation Test Runner',
        formatter_class=argparse.RawDescriptionHelpFormatter
    )

    parser.add_argument('--all', action='store_true',
                       help='Run all test scenarios')
    parser.add_argument('--scenario', type=str,
                       help='Run specific scenario by name')
    parser.add_argument('--output', type=str, default='test_results.json',
                       help='Output file for results (default: test_results.json)')
    parser.add_argument('--scenarios-file', type=str, default=None,
                       help='Path to test scenarios JSON file')

    args = parser.parse_args()

    # Find rosie directory
    rosie_dir = Path(__file__).parent.parent
    scenarios_file = Path(args.scenarios_file) if args.scenarios_file else rosie_dir / 'tests' / 'test_scenarios.json'

    if not scenarios_file.exists():
        print(f"ERROR: Scenarios file not found: {scenarios_file}")
        sys.exit(1)

    print("=" * 70)
    print("ROSIE Conversation Test Runner")
    print("=" * 70)
    print(f"Scenarios file: {scenarios_file}")
    print(f"Output file: {args.output}")

    # Initialize tester
    tester = ROSIETester(rosie_dir)

    # Run tests
    if args.all:
        results = tester.run_all_scenarios(scenarios_file)
    elif args.scenario:
        # Load and filter for specific scenario
        with open(scenarios_file) as f:
            scenarios_data = json.load(f)

        scenario = next((s for s in scenarios_data['scenarios'] if s['name'] == args.scenario), None)
        if not scenario:
            print(f"ERROR: Scenario '{args.scenario}' not found")
            sys.exit(1)

        tester.start_rosie()
        try:
            scenario_results = tester.run_scenario(scenario)
            composite_scores = tester.scorer.calculate_composite_score(scenario_results)

            results = {
                'test_run': datetime.now().isoformat(),
                'overall_scores': composite_scores,
                'results': scenario_results
            }
        finally:
            tester.stop_rosie()
    else:
        print("ERROR: Must specify --all or --scenario")
        sys.exit(1)

    # Save results
    output_path = Path(args.output)
    with open(output_path, 'w') as f:
        json.dump(results, f, indent=2)

    print(f"\n{'='*70}")
    print("Test Results Summary")
    print(f"{'='*70}")
    print(f"Overall Scores:")
    for metric, score in results['overall_scores'].items():
        if metric != 'test_counts':
            print(f"  {metric}: {score}")

    print(f"\nResults saved to: {output_path}")
    print(f"Total tests run: {len(results['results'])}")


if __name__ == '__main__':
    main()
