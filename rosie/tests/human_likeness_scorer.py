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

    def __init__(self, ollama_url: str = "http://localhost:11434", model: str = "llama3.2:latest"):
        """
        Initialize scorer

        Args:
            ollama_url: URL of Ollama server
            model: Ollama model to use for scoring
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
        Build prompt for LLM scoring

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
            traits_note = f"\nExpected traits: {', '.join(expected_traits)}"

        prompt = f"""You are an expert at evaluating conversational AI responses for human-likeness and naturalness.

Your task is to compare two responses to the same question and score how human-like each response sounds.

Question: "{question}"{traits_note}

Response A: "{human_response}"
Response B: "{rosie_response}"

Evaluate BOTH responses on these criteria (0-100 scale for each):

1. NATURALNESS: Does it sound like a real human, not robotic or formulaic?
2. APPROPRIATENESS: Is the response length and tone appropriate for the question?
3. EMOTIONAL_INTELLIGENCE: Does it show appropriate emotional awareness and warmth?
4. SPONTANEITY: Does it feel spontaneous rather than templated or scripted?
5. CONVERSATIONAL_FLOW: Would this response work well in a real human conversation?

Provide your evaluation in this EXACT JSON format:
{{
  "response_a": {{
    "naturalness": <0-100>,
    "appropriateness": <0-100>,
    "emotional_intelligence": <0-100>,
    "spontaneity": <0-100>,
    "conversational_flow": <0-100>,
    "overall": <0-100>,
    "reasoning": "<brief explanation>"
  }},
  "response_b": {{
    "naturalness": <0-100>,
    "appropriateness": <0-100>,
    "emotional_intelligence": <0-100>,
    "spontaneity": <0-100>,
    "conversational_flow": <0-100>,
    "overall": <0-100>,
    "reasoning": "<brief explanation>"
  }},
  "comparison": {{
    "more_human": "<A or B>",
    "reasoning": "<why one sounds more human>"
  }}
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
        Parse LLM JSON response

        Args:
            llm_text: Raw LLM response

        Returns:
            Parsed scores dict
        """
        try:
            # Try to extract JSON from response
            # Sometimes LLM adds text before/after JSON
            start = llm_text.find('{')
            end = llm_text.rfind('}') + 1

            if start != -1 and end > start:
                json_str = llm_text[start:end]
                data = json.loads(json_str)

                # Extract Response B scores (ROSIE)
                rosie_scores = data.get('response_b', {})

                # Calculate weighted overall score
                weights = {
                    'naturalness': 0.3,
                    'appropriateness': 0.25,
                    'emotional_intelligence': 0.2,
                    'spontaneity': 0.15,
                    'conversational_flow': 0.1
                }

                overall = sum(
                    rosie_scores.get(key, 0) * weight
                    for key, weight in weights.items()
                )

                return {
                    'naturalness': rosie_scores.get('naturalness', 0),
                    'appropriateness': rosie_scores.get('appropriateness', 0),
                    'emotional_intelligence': rosie_scores.get('emotional_intelligence', 0),
                    'spontaneity': rosie_scores.get('spontaneity', 0),
                    'conversational_flow': rosie_scores.get('conversational_flow', 0),
                    'overall_score': round(overall, 2),
                    'rosie_reasoning': rosie_scores.get('reasoning', ''),
                    'human_scores': data.get('response_a', {}),
                    'comparison': data.get('comparison', {}),
                    'raw_llm_output': llm_text
                }
            else:
                print(f"[ERROR] No JSON found in LLM response")
                return {'error': 'Failed to parse JSON', 'overall_score': 0}

        except json.JSONDecodeError as e:
            print(f"[ERROR] JSON parsing failed: {e}")
            print(f"[DEBUG] LLM response: {llm_text}")
            return {'error': 'Invalid JSON from LLM', 'overall_score': 0}
        except Exception as e:
            print(f"[ERROR] Unexpected error parsing LLM response: {e}")
            return {'error': str(e), 'overall_score': 0}


if __name__ == '__main__':
    # Test the scorer
    print("Testing Human-Likeness Scorer")
    print("=" * 70)

    scorer = HumanLikenessScorer()

    # Test case 1: Greeting
    print("\n--- Test 1: Greeting ---")
    question = "How are you doing today?"
    human = "I'm doing pretty well, thanks! Just enjoying the morning coffee."
    rosie = "I am functioning within normal parameters. My systems are operational."

    result = scorer.score_response(question, human, rosie, ["friendly", "brief", "warm"])
    print(f"Question: {question}")
    print(f"Human: {human}")
    print(f"ROSIE: {rosie}")
    print(f"\nScores:")
    print(f"  Naturalness: {result.get('naturalness', 0)}/100")
    print(f"  Appropriateness: {result.get('appropriateness', 0)}/100")
    print(f"  Emotional Intelligence: {result.get('emotional_intelligence', 0)}/100")
    print(f"  Spontaneity: {result.get('spontaneity', 0)}/100")
    print(f"  Conversational Flow: {result.get('conversational_flow', 0)}/100")
    print(f"  OVERALL: {result.get('overall_score', 0)}/100")
    print(f"\nReasoning: {result.get('rosie_reasoning', 'N/A')}")
    print(f"Comparison: {result.get('comparison', {})}")

    # Test case 2: Open-ended
    print("\n--- Test 2: Open-Ended ---")
    question = "What's on your mind?"
    human = "Oh, just thinking about what to make for dinner tonight. Maybe pasta?"
    rosie = "I am currently processing various inputs and maintaining operational awareness."

    result2 = scorer.score_response(question, human, rosie, ["thoughtful", "genuine", "conversational"])
    print(f"Question: {question}")
    print(f"Human: {human}")
    print(f"ROSIE: {rosie}")
    print(f"\nROSIE Overall Score: {result2.get('overall_score', 0)}/100")
    print(f"Reasoning: {result2.get('rosie_reasoning', 'N/A')}")
