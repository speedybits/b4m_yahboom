#!/usr/bin/env python3
"""
Human-Likeness Scorer for ROSIE Advanced Testing

Uses LLM (Ollama) to compare ROSIE's responses against human "golden reference" responses
and score them on multiple dimensions of naturalness and human-likeness.
"""

import requests
import json
from typing import Dict, Optional
import os


class HumanLikenessScorer:
    """LLM-based scorer for comparing conversational naturalness"""

    def __init__(self, ollama_url: str = "http://localhost:11434", model: str = "llama3.1:8b"):
        """
        Initialize scorer

        Args:
            ollama_url: URL of Ollama server
            model: Ollama model to use for scoring (default: llama3.1:8b)
        """
        self.ollama_url = ollama_url
        self.model = model

    def score_response(self, question: str, human_response: str, rosie_response: str,
                      expected_traits: list = None) -> Dict:
        """
        Score ROSIE's response against human reference

        Args:
            question: The question asked
            human_response: Human's response (golden reference)
            rosie_response: ROSIE's response
            expected_traits: Expected conversational traits

        Returns:
            Dict with detailed scoring results
        """
        # Build scoring prompt
        prompt = self._build_scoring_prompt(
            question, human_response, rosie_response, expected_traits
        )

        # Get LLM evaluation
        llm_result = self._call_ollama(prompt)

        if not llm_result:
            return {
                'error': 'Failed to get LLM scoring',
                'overall_score': 0
            }

        # Parse LLM response
        scores = self._parse_llm_response(llm_result)

        # Add metadata
        scores['question'] = question
        scores['human_response'] = human_response
        scores['rosie_response'] = rosie_response
        scores['expected_traits'] = expected_traits

        return scores

    def _build_scoring_prompt(self, question: str, human_response: str,
                             rosie_response: str, expected_traits: list = None) -> str:
        """
        Build prompt for LLM scoring - simple binary vote

        Args:
            question: The question asked
            human_response: Human's response
            rosie_response: ROSIE's response
            expected_traits: Expected traits

        Returns:
            Scoring prompt
        """
        traits_note = ""
        if expected_traits:
            traits_note = f"\nExpected traits for this question: {', '.join(expected_traits)}"

        prompt = f"""You are an expert at evaluating conversational naturalness.

Your task: Compare two responses to the same question and vote for which one sounds MORE LIKE A REAL HUMAN.

Question: "{question}"{traits_note}

Response A: "{human_response}"
Response B: "{rosie_response}"

Which response sounds more like it came from a real human in a natural conversation?

Consider:
- Does it sound natural and spontaneous, not robotic or formulaic?
- Is the tone and length appropriate for the question?
- Does it show appropriate emotional warmth?
- Would this work in a real conversation?
- IMPORTANT: If either response makes up false/contradictory factual information (weather, dates, times, events), it should LOSE (humans don't fabricate observable facts)

Provide your vote in this EXACT JSON format:
{{
  "winner": "<A or B>",
  "reasoning": "<2-3 sentences explaining why the winner sounds more human>"
}}

Respond ONLY with valid JSON, no other text."""

        return prompt

    def _call_ollama(self, prompt: str) -> Optional[str]:
        """
        Call Ollama API

        Args:
            prompt: Prompt to send

        Returns:
            Response text or None on error
        """
        try:
            response = requests.post(
                f"{self.ollama_url}/api/generate",
                json={
                    "model": self.model,
                    "prompt": prompt,
                    "stream": False,
                    "options": {
                        "temperature": 0.3,  # Low temp for consistent scoring
                        "num_predict": 1000
                    }
                },
                timeout=60
            )

            if response.status_code == 200:
                result = response.json()
                return result.get('response', '')
            else:
                print(f"[ERROR] Ollama request failed: {response.status_code}")
                return None

        except Exception as e:
            print(f"[ERROR] Failed to call Ollama: {e}")
            return None

    def _parse_llm_response(self, llm_text: str) -> Dict:
        """
        Parse LLM JSON response - simple binary vote

        Args:
            llm_text: Raw LLM response

        Returns:
            Parsed vote dict with winner and reasoning
        """
        try:
            # Try to extract JSON from response
            # Sometimes LLM adds text before/after JSON
            start = llm_text.find('{')
            end = llm_text.rfind('}') + 1

            if start != -1 and end > start:
                json_str = llm_text[start:end]
                data = json.loads(json_str)

                winner = data.get('winner', '').upper()
                reasoning = data.get('reasoning', '')

                # Determine if ROSIE won
                # Response A = Human (golden reference)
                # Response B = ROSIE
                rosie_won = (winner == 'B')

                return {
                    'winner': winner,
                    'rosie_won': rosie_won,
                    'reasoning': reasoning,
                    'raw_llm_output': llm_text
                }
            else:
                print(f"[ERROR] No JSON found in LLM response")
                return {'error': 'Failed to parse JSON', 'rosie_won': False}

        except json.JSONDecodeError as e:
            print(f"[ERROR] JSON parsing failed: {e}")
            print(f"[DEBUG] LLM response: {llm_text}")
            return {'error': 'Invalid JSON from LLM', 'rosie_won': False}
        except Exception as e:
            print(f"[ERROR] Unexpected error parsing LLM response: {e}")
            return {'error': str(e), 'rosie_won': False}


if __name__ == '__main__':
    # Test the scorer
    print("Testing Human-Likeness Scorer (Binary Vote)")
    print("=" * 70)

    scorer = HumanLikenessScorer()

    # Test case 1: Greeting - Obviously robotic ROSIE
    print("\n--- Test 1: Greeting (Robotic ROSIE) ---")
    question = "How are you doing today?"
    human = "I'm doing pretty well, thanks! Just enjoying the morning coffee."
    rosie = "I am functioning within normal parameters. My systems are operational."

    result = scorer.score_response(question, human, rosie, ["friendly", "brief", "warm"])
    print(f"Question: {question}")
    print(f"Human: {human}")
    print(f"ROSIE: {rosie}")
    print(f"\nVote Result:")
    print(f"  Winner: Response {result.get('winner', 'N/A')}")
    print(f"  ROSIE Won: {result.get('rosie_won', False)}")
    print(f"  Reasoning: {result.get('reasoning', 'N/A')}")

    # Test case 2: Open-ended - Obviously robotic ROSIE
    print("\n--- Test 2: Open-Ended (Robotic ROSIE) ---")
    question = "What's on your mind?"
    human = "Oh, just thinking about what to make for dinner tonight. Maybe pasta?"
    rosie = "I am currently processing various inputs and maintaining operational awareness."

    result2 = scorer.score_response(question, human, rosie, ["thoughtful", "genuine", "conversational"])
    print(f"Question: {question}")
    print(f"Human: {human}")
    print(f"ROSIE: {rosie}")
    print(f"\nVote Result:")
    print(f"  Winner: Response {result2.get('winner', 'N/A')}")
    print(f"  ROSIE Won: {result2.get('rosie_won', False)}")
    print(f"  Reasoning: {result2.get('reasoning', 'N/A')}")

    # Test case 3: Natural ROSIE response
    print("\n--- Test 3: Greeting (Natural ROSIE) ---")
    question = "How are you doing?"
    human = "Pretty good, just relaxing today."
    rosie = "I'm doing well, thanks for asking!"

    result3 = scorer.score_response(question, human, rosie, ["friendly", "brief", "natural"])
    print(f"Question: {question}")
    print(f"Human: {human}")
    print(f"ROSIE: {rosie}")
    print(f"\nVote Result:")
    print(f"  Winner: Response {result3.get('winner', 'N/A')}")
    print(f"  ROSIE Won: {result3.get('rosie_won', False)}")
    print(f"  Reasoning: {result3.get('reasoning', 'N/A')}")
