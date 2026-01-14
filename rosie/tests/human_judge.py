#!/usr/bin/env python3
"""
Human Judge for ROSIE Advanced Testing

Allows a human judge to compare ROSIE's responses against human "golden reference" responses
through a simple terminal interface.
"""

from typing import Dict


class HumanJudge:
    """Terminal-based human judge for comparing conversational naturalness"""

    def __init__(self):
        """Initialize human judge"""
        pass

    def score_response(self, question: str, human_response: str, rosie_response: str,
                      expected_traits: list = None) -> Dict:
        """
        Get human judge's vote on which response sounds more human

        Args:
            question: The question asked
            human_response: Human's response (golden reference)
            rosie_response: ROSIE's response
            expected_traits: Expected conversational traits (displayed to judge)

        Returns:
            Dict with judge's vote and reasoning
        """
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

        # Get judge's vote
        while True:
            vote = input("Who sounds more human? [H]uman or [R]OSIE: ").strip().upper()

            if vote in ['H', 'HUMAN']:
                winner = 'Human'
                rosie_won = False
                break
            elif vote in ['R', 'ROSIE']:
                winner = 'ROSIE'
                rosie_won = True
                break
            else:
                print("Invalid input. Please enter 'H' for Human or 'R' for ROSIE.")

        # Optional: Get reasoning
        reasoning = input("Brief reason for your choice (optional, press Enter to skip): ").strip()
        if not reasoning:
            reasoning = f"Judge selected {winner} as more human-sounding"

        return {
            'winner': winner,
            'rosie_won': rosie_won,
            'reasoning': reasoning,
            'judge_type': 'human'
        }


if __name__ == '__main__':
    # Test the human judge
    print("Testing Human Judge Interface")
    print("=" * 70)

    judge = HumanJudge()

    # Test case
    question = "How are you doing today?"
    human = "I'm doing pretty well, thanks for asking! How about you?"
    rosie = "I'm feeling great, thanks! How's your day going?"

    result = judge.score_response(question, human, rosie, ["friendly", "brief", "warm"])

    print(f"\n{'='*70}")
    print("RESULT:")
    print(f"  Winner: {result['winner']}")
    print(f"  ROSIE Won: {result['rosie_won']}")
    print(f"  Reasoning: {result['reasoning']}")
    print(f"{'='*70}")
