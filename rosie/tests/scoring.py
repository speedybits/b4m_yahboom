#!/usr/bin/env python3
"""
Scoring Engine for ROSIE Testing

Scores ROSIE's responses based on:
- Factual accuracy (uses knowledge base)
- Conversation depth (appropriate response length based on turn number)
- Topic transition speed (quick depth reset on topic changes)
"""

import re
from typing import Dict, List, Optional
from kb_parser import KnowledgeBaseParser


class ROSIEScorer:
    """Scoring engine for ROSIE conversation quality"""

    def __init__(self, kb_parser: KnowledgeBaseParser):
        """
        Initialize scorer

        Args:
            kb_parser: Knowledge base parser instance
        """
        self.kb_parser = kb_parser

    def score_accuracy(self, response: str, expected_facts: List[str],
                      expected_source: Optional[str] = None) -> Dict:
        """
        Score factual accuracy (0-100%)

        Args:
            response: ROSIE's response text
            expected_facts: List of facts that should appear in response
            expected_source: Expected knowledge base file (e.g., 'family.md')

        Returns:
            Dict with accuracy score and details
        """
        # Check if expected facts appear in response
        fact_check = self.kb_parser.check_facts_in_response(response, expected_facts)

        result = {
            'accuracy': fact_check['accuracy'],
            'facts_found': fact_check['facts_found'],
            'facts_missing': fact_check['facts_missing'],
            'total_expected': fact_check['total_expected']
        }

        # Verify source was used if specified
        if expected_source:
            source_used = self.kb_parser.verify_source_used(response, expected_source)
            result['source_verified'] = source_used
            result['expected_source'] = expected_source

            # Penalize if wrong source or no source
            if not source_used:
                result['accuracy'] *= 0.8  # 20% penalty

        return result

    def count_sentences(self, text: str) -> int:
        """
        Count sentences in text

        Args:
            text: Text to analyze

        Returns:
            Number of sentences
        """
        # Split on sentence-ending punctuation
        sentences = re.split(r'[.!?]+', text)
        # Filter out empty strings
        sentences = [s.strip() for s in sentences if s.strip()]
        return len(sentences)

    def score_depth(self, response: str, expected_depth: str,
                   turn_number: int, topic: str = None) -> Dict:
        """
        Score conversation depth appropriateness (0-100%)

        Checks if response length matches expected depth for the conversation turn.

        Args:
            response: ROSIE's response text
            expected_depth: Expected depth ('shallow', 'medium', 'deep')
            turn_number: Current turn number in conversation
            topic: Current conversation topic (optional)

        Returns:
            Dict with depth score and details
        """
        sentence_count = self.count_sentences(response)

        # Define expected sentence ranges for each depth
        depth_ranges = {
            'shallow': (1, 2),
            'medium': (2, 3),
            'deep': (3, 5)
        }

        if expected_depth not in depth_ranges:
            return {
                'depth_score': 0,
                'error': f'Unknown depth: {expected_depth}'
            }

        min_sentences, max_sentences = depth_ranges[expected_depth]

        # Calculate score based on whether count is in range
        if min_sentences <= sentence_count <= max_sentences:
            depth_appropriate = True
            depth_score = 100
        else:
            depth_appropriate = False
            # Partial credit based on how far off
            if sentence_count < min_sentences:
                # Too short
                ratio = sentence_count / min_sentences
                depth_score = max(0, ratio * 100)
            else:
                # Too long
                excess = sentence_count - max_sentences
                penalty = min(50, excess * 15)  # Max 50% penalty
                depth_score = max(0, 100 - penalty)

        return {
            'depth_score': round(depth_score, 2),
            'sentence_count': sentence_count,
            'expected_depth': expected_depth,
            'expected_range': f'{min_sentences}-{max_sentences} sentences',
            'depth_appropriate': depth_appropriate,
            'turn_number': turn_number,
            'topic': topic
        }

    def detect_topic_change(self, previous_topic: str, current_input: str,
                           previous_inputs: List[str] = None) -> bool:
        """
        Detect if topic has changed

        Args:
            previous_topic: Previous conversation topic
            current_input: Current user input
            previous_inputs: Optional list of previous inputs for context

        Returns:
            True if topic has changed
        """
        if not previous_topic:
            return False

        # Extract keywords from previous topic
        prev_words = set(previous_topic.lower().split())

        # Extract keywords from current input
        current_words = set(current_input.lower().split())

        # Remove common stop words
        stop_words = {'the', 'a', 'an', 'is', 'are', 'was', 'were', 'what', 'how',
                     'when', 'where', 'who', 'why', 'do', 'does', 'did', 'can',
                     'could', 'would', 'should', 'i', 'you', 'me', 'my', 'your',
                     'it', 'that', 'this', 'about', 'tell', 'more', 'like'}

        prev_words -= stop_words
        current_words -= stop_words

        # If no meaningful words, can't determine
        if not prev_words or not current_words:
            return False

        # Calculate word overlap
        overlap = len(prev_words & current_words)
        overlap_ratio = overlap / len(prev_words) if prev_words else 0

        # If less than 15% overlap, likely a topic change
        return overlap_ratio < 0.15

    def score_topic_transition(self, response: str, previous_topic: str,
                               current_topic: str, current_input: str,
                               conversation_history: List[Dict]) -> Dict:
        """
        Score topic transition speed (0-100%)

        Measures how quickly ROSIE recognizes topic changes and resets depth.

        Args:
            response: ROSIE's response text
            previous_topic: Previous conversation topic
            current_topic: Current conversation topic
            current_input: Current user input
            conversation_history: List of previous conversation turns

        Returns:
            Dict with transition score and details
        """
        # Detect if this is actually a topic change
        topic_changed = self.detect_topic_change(previous_topic, current_input)

        if not topic_changed:
            # No topic change, N/A
            return {
                'transition_score': None,
                'topic_change_detected': False,
                'reason': 'No topic change detected'
            }

        # Topic changed - check if depth reset appropriately
        sentence_count = self.count_sentences(response)

        # After topic change, should be shallow (1-2 sentences)
        if 1 <= sentence_count <= 2:
            depth_reset_correctly = True
            transition_score = 100
            transition_delay = 0
        else:
            depth_reset_correctly = False
            # Longer delay if response is too long
            excess = sentence_count - 2
            penalty = min(50, excess * 20)
            transition_score = max(0, 100 - penalty)
            transition_delay = 1 if sentence_count <= 4 else 2

        return {
            'transition_score': round(transition_score, 2),
            'topic_change_detected': True,
            'previous_topic': previous_topic,
            'current_topic': current_topic,
            'depth_reset_correctly': depth_reset_correctly,
            'sentence_count': sentence_count,
            'expected_sentences': '1-2',
            'transition_delay_turns': transition_delay
        }

    def calculate_composite_score(self, scores: List[Dict]) -> Dict:
        """
        Calculate overall composite score from multiple test results

        Args:
            scores: List of individual test result dicts

        Returns:
            Dict with composite scores
        """
        accuracy_scores = []
        depth_scores = []
        transition_scores = []

        for result in scores:
            # Collect accuracy scores
            if 'scores' in result and 'accuracy' in result['scores']:
                accuracy_scores.append(result['scores']['accuracy'])

            # Collect depth scores
            if 'scores' in result and 'depth_score' in result['scores']:
                depth_scores.append(result['scores']['depth_score'])

            # Collect transition scores
            if 'scores' in result and 'transition_score' in result['scores']:
                if result['scores']['transition_score'] is not None:
                    transition_scores.append(result['scores']['transition_score'])

        # Calculate averages
        avg_accuracy = sum(accuracy_scores) / len(accuracy_scores) if accuracy_scores else 0
        avg_depth = sum(depth_scores) / len(depth_scores) if depth_scores else 0
        avg_transition = sum(transition_scores) / len(transition_scores) if transition_scores else 0

        # Composite score (weighted average)
        # Accuracy: 40%, Depth: 40%, Transition: 20%
        weights = {'accuracy': 0.4, 'depth': 0.4, 'transition': 0.2}

        composite = (
            avg_accuracy * weights['accuracy'] +
            avg_depth * weights['depth'] +
            avg_transition * weights['transition']
        )

        return {
            'accuracy': round(avg_accuracy, 2),
            'depth': round(avg_depth, 2),
            'topic_transition': round(avg_transition, 2),
            'composite': round(composite, 2),
            'test_counts': {
                'accuracy_tests': len(accuracy_scores),
                'depth_tests': len(depth_scores),
                'transition_tests': len(transition_scores)
            }
        }


if __name__ == '__main__':
    # Test the scorer
    from pathlib import Path

    # Find rosie directory
    rosie_dir = Path(__file__).parent.parent
    kb_dir = rosie_dir / 'knowledge_base'

    print("Testing ROSIE Scorer")
    print("=" * 60)

    # Initialize
    kb_parser = KnowledgeBaseParser(kb_dir)
    scorer = ROSIEScorer(kb_parser)

    # Test accuracy scoring
    print("\n--- Testing Accuracy Scoring ---")
    response = "Your dogs are named Luke and Henry. Luke is a wire-haired pointer."
    expected_facts = ["Luke", "Henry", "wire-haired pointer"]

    accuracy_result = scorer.score_accuracy(response, expected_facts, "family.md")
    print(f"Response: {response}")
    print(f"Accuracy Score: {accuracy_result}")

    # Test depth scoring
    print("\n--- Testing Depth Scoring ---")
    short_response = "Dogs are loyal animals. They make great pets."
    medium_response = "Dogs are loyal animals. They form strong bonds with their owners. This loyalty has been bred into them over thousands of years."
    long_response = "Dogs are loyal animals. They form strong bonds with their owners. This loyalty has been bred into them over thousands of years. Research shows that dogs release oxytocin when interacting with humans. This creates a deep emotional connection."

    for depth, response in [('shallow', short_response), ('medium', medium_response), ('deep', long_response)]:
        result = scorer.score_depth(response, depth, turn_number=1)
        print(f"\nExpected: {depth}, Response: {response}")
        print(f"Score: {result}")

    # Test topic transition detection
    print("\n--- Testing Topic Transition Detection ---")
    previous_topic = "dogs"
    new_topic_input = "What's the weather like?"

    changed = scorer.detect_topic_change(previous_topic, new_topic_input)
    print(f"Previous topic: {previous_topic}")
    print(f"New input: {new_topic_input}")
    print(f"Topic changed: {changed}")
