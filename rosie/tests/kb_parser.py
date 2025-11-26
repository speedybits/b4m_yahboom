#!/usr/bin/env python3
"""
Knowledge Base Parser for ROSIE Testing

Loads and parses ROSIE's knowledge base markdown files
to validate factual accuracy in responses.
"""

from pathlib import Path
from typing import Dict, List, Set
import re


class KnowledgeBaseParser:
    """Parser for ROSIE's markdown knowledge base"""

    def __init__(self, kb_dir: Path):
        """
        Initialize knowledge base parser

        Args:
            kb_dir: Path to knowledge_base directory
        """
        self.kb_dir = Path(kb_dir)
        self.knowledge = {}  # filename -> content mapping
        self._load_knowledge_base()

    def _load_knowledge_base(self):
        """Load all markdown files from knowledge base"""
        if not self.kb_dir.exists():
            raise FileNotFoundError(f"Knowledge base directory not found: {self.kb_dir}")

        # Load all .md files except README
        for md_file in self.kb_dir.glob("*.md"):
            if md_file.name == "README.md":
                continue

            content = md_file.read_text()
            self.knowledge[md_file.name] = content
            print(f"[KB Parser] Loaded {md_file.name} ({len(content)} chars)")

    def get_file_content(self, filename: str) -> str:
        """
        Get content of a specific knowledge base file

        Args:
            filename: Name of the file (e.g., 'family.md')

        Returns:
            File content as string
        """
        return self.knowledge.get(filename, "")

    def search_facts(self, query: str, files: List[str] = None) -> Dict[str, List[str]]:
        """
        Search for facts related to query across knowledge base

        Args:
            query: Search query
            files: Optional list of specific files to search

        Returns:
            Dict mapping filename to list of matching lines
        """
        results = {}
        query_lower = query.lower()

        # Determine which files to search
        search_files = files if files else list(self.knowledge.keys())

        for filename in search_files:
            if filename not in self.knowledge:
                continue

            content = self.knowledge[filename]
            matching_lines = []

            # Search line by line
            for line in content.split('\n'):
                if query_lower in line.lower():
                    matching_lines.append(line.strip())

            if matching_lines:
                results[filename] = matching_lines

        return results

    def check_facts_in_response(self, response: str, expected_facts: List[str]) -> Dict:
        """
        Check if expected facts appear in response

        Args:
            response: ROSIE's response text
            expected_facts: List of facts that should appear

        Returns:
            Dict with scoring information
        """
        response_lower = response.lower()

        facts_found = []
        facts_missing = []

        for fact in expected_facts:
            # Check for exact match or partial match
            fact_lower = fact.lower()
            if fact_lower in response_lower:
                facts_found.append(fact)
            else:
                # Try fuzzy matching - check if words appear
                fact_words = set(fact_lower.split())
                response_words = set(response_lower.split())
                overlap = len(fact_words & response_words)

                # If majority of words match, consider it found
                if overlap >= len(fact_words) * 0.7:
                    facts_found.append(fact)
                else:
                    facts_missing.append(fact)

        total_facts = len(expected_facts)
        accuracy = (len(facts_found) / total_facts * 100) if total_facts > 0 else 0

        return {
            'facts_found': facts_found,
            'facts_missing': facts_missing,
            'accuracy': round(accuracy, 2),
            'total_expected': total_facts
        }

    def verify_source_used(self, response: str, source_file: str) -> bool:
        """
        Check if response likely used content from expected source file

        Args:
            response: ROSIE's response
            source_file: Expected source file (e.g., 'family.md')

        Returns:
            True if source appears to be used
        """
        if source_file not in self.knowledge:
            return False

        source_content = self.knowledge[source_file]
        response_lower = response.lower()

        # Extract key terms from source (words longer than 4 chars)
        source_words = set()
        for word in re.findall(r'\b\w{5,}\b', source_content.lower()):
            # Skip common words
            if word not in {'there', 'their', 'would', 'could', 'should', 'about', 'which'}:
                source_words.add(word)

        # Check how many source terms appear in response
        matching_words = 0
        for word in source_words:
            if word in response_lower:
                matching_words += 1

        # If at least 20% of unique source terms appear, likely used
        threshold = max(1, len(source_words) * 0.2)
        return matching_words >= threshold

    def get_all_files(self) -> List[str]:
        """Get list of all loaded knowledge base files"""
        return list(self.knowledge.keys())


if __name__ == '__main__':
    # Test the parser
    import sys

    # Find rosie directory (parent of tests/)
    rosie_dir = Path(__file__).parent.parent
    kb_dir = rosie_dir / 'knowledge_base'

    print("Testing Knowledge Base Parser")
    print("=" * 60)

    parser = KnowledgeBaseParser(kb_dir)
    print(f"\nLoaded {len(parser.get_all_files())} knowledge base files")

    # Test searching
    print("\n--- Testing search for 'dog' ---")
    results = parser.search_facts('dog')
    for filename, lines in results.items():
        print(f"\n{filename}:")
        for line in lines[:3]:  # Show first 3 matches
            print(f"  - {line}")

    # Test fact checking
    print("\n--- Testing fact checking ---")
    test_response = "Your dogs are named Luke and Henry. Luke is a 14 year old wire-haired pointer."
    expected_facts = ["Luke", "Henry", "wire-haired pointer"]

    result = parser.check_facts_in_response(test_response, expected_facts)
    print(f"Response: {test_response}")
    print(f"Expected facts: {expected_facts}")
    print(f"Results: {result}")
