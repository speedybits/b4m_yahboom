#!/usr/bin/env python3
"""
Claude Judge for ROSIE Advanced Testing

Allows Claude Code to act as judge by reading/writing vote files.
Works asynchronously - displays comparison, waits for vote file, then continues.
"""

import json
import time
from typing import Dict
from pathlib import Path


class ClaudeJudge:
    """File-based judge interface for Claude Code to provide votes"""

    def __init__(self):
        """Initialize Claude judge with file paths"""
        self.tests_dir = Path(__file__).parent
        self.comparison_file = self.tests_dir / 'claude_judge_comparison.json'
        self.vote_file = self.tests_dir / 'claude_judge_vote.json'

    def score_response(self, question: str, human_response: str, rosie_response: str,
                      expected_traits: list = None) -> Dict:
        """
        Get Claude's vote by writing comparison and waiting for vote file

        Args:
            question: The question asked
            human_response: Human's response (golden reference)
            rosie_response: ROSIE's response
            expected_traits: Expected conversational traits

        Returns:
            Dict with judge's vote and reasoning
        """
        # Write comparison to file for Claude to read
        comparison = {
            'question': question,
            'human_response': human_response,
            'rosie_response': rosie_response,
            'expected_traits': expected_traits or []
        }

        with open(self.comparison_file, 'w') as f:
            json.dump(comparison, f, indent=2)

        # Display to console
        print(f"\n{'='*70}")
        print(f"QUESTION: {question}")
        if expected_traits:
            print(f"Expected traits: {', '.join(expected_traits)}")
        print(f"{'='*70}\n")

        print(f"[HUMAN] Response:")
        print(f'  "{human_response}"')
        print()

        print(f"[ROSIE] Response:")
        print(f'  "{rosie_response}"')
        print()

        print(f"[CLAUDE JUDGE] Comparison written to: {self.comparison_file}")
        print(f"[CLAUDE JUDGE] Waiting for vote file: {self.vote_file}")
        print(f"[CLAUDE JUDGE] Vote file format:")
        print(f'  {{"winner": "Human" or "ROSIE", "reasoning": "your reasoning"}}')
        print()

        # Wait for vote file (with timeout)
        timeout = 300  # 5 minutes
        start_time = time.time()

        while True:
            if self.vote_file.exists():
                try:
                    with open(self.vote_file) as f:
                        vote_data = json.load(f)

                    # Delete vote file for next round
                    self.vote_file.unlink()

                    # Parse vote
                    winner = vote_data.get('winner', '').strip()
                    reasoning = vote_data.get('reasoning', '')

                    if winner not in ['Human', 'ROSIE']:
                        print(f"[ERROR] Invalid winner in vote file: {winner}")
                        print(f"[ERROR] Must be 'Human' or 'ROSIE'")
                        continue

                    rosie_won = (winner == 'ROSIE')

                    print(f"[CLAUDE JUDGE] Vote received: {winner}")
                    print(f"[CLAUDE JUDGE] Reasoning: {reasoning}")

                    return {
                        'winner': winner,
                        'rosie_won': rosie_won,
                        'reasoning': reasoning,
                        'judge_type': 'claude'
                    }

                except json.JSONDecodeError as e:
                    print(f"[ERROR] Invalid JSON in vote file: {e}")
                    self.vote_file.unlink()
                    continue
                except Exception as e:
                    print(f"[ERROR] Failed to read vote file: {e}")
                    continue

            # Check timeout
            if time.time() - start_time > timeout:
                raise TimeoutError(f"No vote received within {timeout} seconds")

            # Wait a bit before checking again
            time.sleep(0.5)


if __name__ == '__main__':
    # Test the Claude judge
    print("Testing Claude Judge Interface")
    print("=" * 70)

    judge = ClaudeJudge()

    # Test case
    question = "How are you doing today?"
    human = "I'm doing pretty well, thanks for asking! How about you?"
    rosie = "I'm feeling great, thanks! How's your day going?"

    print("This will wait for you to create a vote file...")
    print(f"Vote file path: {judge.vote_file}")
    print()

    result = judge.score_response(question, human, rosie, ["friendly", "brief", "warm"])

    print(f"\n{'='*70}")
    print("RESULT:")
    print(f"  Winner: {result['winner']}")
    print(f"  ROSIE Won: {result['rosie_won']}")
    print(f"  Reasoning: {result['reasoning']}")
    print(f"{'='*70}")
