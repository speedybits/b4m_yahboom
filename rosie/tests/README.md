# ROSIE Conversation Testing Framework

Programmatic testing framework for analyzing and improving ROSIE's conversation quality.

## Overview

This framework allows Claude (or humans) to:
1. Run automated conversation tests against ROSIE
2. Score responses based on accuracy, depth, and topic transitions
3. Iterate on prompt improvements with measurable results
4. Track conversation quality over time

## Components

### Core Files

- **`test_conversations.py`** - Main test runner that spawns ROSIE and executes tests
- **`kb_parser.py`** - Parses knowledge base to verify factual accuracy
- **`scoring.py`** - Scores responses (accuracy, depth, transitions)
- **`test_scenarios.json`** - Test scenarios with expected behaviors
- **`run_tests.sh`** - Helper script for running tests

### Test Scenarios

Located in `test_scenarios.json`, organized by:
- **Factual Accuracy** - Tests knowledge base retrieval
- **Conversation Depth** - Tests appropriate response length based on turn number
- **Topic Transitions** - Tests quick depth reset when topic changes
- **Mixed** - Combined factual and conversational tests

## Scoring Metrics

### 1. Accuracy Score (0-100%)
Measures factual correctness:
- ✅ **100%**: All expected facts present, correct source used
- ⚠️ **50-80%**: Some facts missing or wrong source
- ❌ **0%**: Incorrect facts or "I don't know" when facts are available

### 2. Depth Score (0-100%)
Measures appropriate response length:
- **Shallow** (turns 1-2): 1-2 sentences
- **Medium** (turns 3-4): 2-3 sentences
- **Deep** (turns 5+): 3-5 sentences with reasoning

### 3. Topic Transition Score (0-100%)
Measures speed of depth reset on topic change:
- ✅ **100%**: Immediate shallow response on new topic
- ⚠️ **50%**: Takes 1-2 turns to recognize change
- ❌ **0%**: Continues deep responses on new topic

### 4. Composite Score (0-100%)
Weighted average:
- Accuracy: 40%
- Depth: 40%
- Transition: 20%

## Usage

### Quick Start

```bash
# Run all test scenarios
./run_tests.sh --all

# Run specific scenario
./run_tests.sh --scenario "Factual Accuracy - Family Facts"

# View results
./run_tests.sh --view test_results.json

# Compare two test runs
./run_tests.sh --compare before.json after.json
```

### Python Direct Usage

```bash
# Run all tests
python3 test_conversations.py --all --output results.json

# Run specific scenario
python3 test_conversations.py --scenario "Conversation Depth - Single Topic"

# View scores
cat results.json | jq '.overall_scores'
```

## For Claude: Iterative Prompt Improvement Workflow

### 1. Baseline Testing
```bash
cd rosie/tests
./run_tests.sh --all baseline_results.json
./run_tests.sh --view baseline_results.json
```

### 2. Identify Issues
```bash
# Find low-scoring tests
cat baseline_results.json | jq '.results[] | select(.scores.accuracy < 80)'

# Find depth issues
cat baseline_results.json | jq '.results[] | select(.scores.depth_appropriate == false)'

# Find transition issues
cat baseline_results.json | jq '.results[] | select(.scores.transition_score < 80)'
```

### 3. Modify Prompts
Edit prompts in `/home/mike/projects/b4m_yahboom/rosie/src/rosie_conversation.py`:

**Factual Mode Prompt** (lines ~1692-1720):
- Controls fact extraction behavior
- Temperature: 0.1 (low for accuracy)
- Emphasizes knowledge base usage

**Conversational Mode Prompt** (lines ~1730+):
- Controls creative/conversational responses
- Temperature: 0.7 (higher for variety)
- Handles greetings, opinions, etc.

**Depth Analysis** (lines ~1448-1560):
- `_analyze_conversation_depth()` function
- Controls shallow/medium/deep classification
- Adjust thresholds for turn-based depth changes

### 4. Re-test
```bash
./run_tests.sh --all improved_results.json
```

### 5. Compare Results
```bash
./run_tests.sh --compare baseline_results.json improved_results.json
```

### 6. Iterate
Repeat steps 2-5 until scores meet targets:
- Accuracy: > 90%
- Depth: > 85%
- Transition: > 80%
- Composite: > 85%

## Example Output

```json
{
  "test_run": "2025-11-26T07:00:00",
  "overall_scores": {
    "accuracy": 92.5,
    "depth": 87.3,
    "topic_transition": 95.0,
    "composite": 90.2
  },
  "results": [
    {
      "scenario": "Factual Accuracy - Family Facts",
      "turn": 1,
      "input": "What are my dogs names?",
      "response": "According to my knowledge base, you have two dogs named Luke and Henry.",
      "scores": {
        "accuracy": 100,
        "facts_found": ["Luke", "Henry"],
        "facts_missing": [],
        "source_verified": true
      }
    }
  ]
}
```

## Adding New Test Scenarios

Edit `test_scenarios.json`:

```json
{
  "name": "My New Test",
  "test_type": "accuracy",
  "conversations": [
    {
      "input": "Test question?",
      "expected_facts": ["fact1", "fact2"],
      "expected_source": "family.md",
      "scoring": {
        "fact_match_required": true,
        "min_accuracy": 90
      }
    }
  ]
}
```

## Prompt Locations in Code

All prompts are in `rosie/src/rosie_conversation.py`:

| Line Range | Component | Purpose |
|-----------|-----------|---------|
| 1448-1560 | `_analyze_conversation_depth()` | Shallow/medium/deep classification |
| 1567-1850 | `_ollama_response()` | Main response generation |
| 1692-1720 | Factual mode prompt | Low-temp fact extraction |
| 1730-1800 | Conversational mode prompt | Higher-temp responses |

## Tips for Prompt Engineering

1. **Start with accuracy** - Get facts right first
2. **Then depth** - Tune response length appropriateness
3. **Finally transitions** - Optimize topic change recognition
4. **Test frequently** - Run tests after each change
5. **Track progress** - Save results with descriptive names
6. **Look for patterns** - Similar failures may indicate systemic issues

## Troubleshooting

### ROSIE Won't Start
```bash
# Clean up hung processes
killall -9 python3
sleep 2
./run_tests.sh --all
```

### Tests Timeout
- Increase timeout in `test_conversations.py` line ~175
- Check ROSIE logs for initialization issues

### Scoring Seems Off
- Verify knowledge base content
- Check expected_facts match actual KB content
- Adjust fuzzy matching threshold in `kb_parser.py`

## Future Enhancements

- [ ] HTML report generation
- [ ] Trend visualization over time
- [ ] A/B prompt testing
- [ ] Performance benchmarking (tokens/second)
- [ ] Cost tracking (if using paid LLMs)
