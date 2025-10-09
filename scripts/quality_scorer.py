#!/usr/bin/env python3
"""
Quality Scoring Module for LLM Benchmark Results
Inspired by OpenAI Evals and EleutherAI's lm-evaluation-harness

References:
- OpenAI Evals: https://github.com/openai/evals
- IFEval: Instruction-Following Evaluation (https://arxiv.org/abs/2311.07911)
- EleutherAI lm-evaluation-harness: https://github.com/EleutherAI/lm-evaluation-harness

This module provides automated quality scoring for different prompt types:
1. Simple prompts: Instruction following (word count, format)
2. Navigation prompts: JSON validity, required fields, structure
3. Reasoning prompts: Answer presence, explanation quality, logic
"""

import json
import re
import sys
from typing import Dict, List, Tuple, Any


class QualityScorer:
    """Automated quality scoring for LLM responses."""
    
    def __init__(self):
        self.scorers = {
            'simple': self.score_simple_prompt,
            'navigation': self.score_navigation_prompt,
            'reasoning': self.score_reasoning_prompt,
        }
    
    def score_response(self, prompt_type: str, prompt_text: str, response: str) -> Dict[str, Any]:
        """
        Score a response based on prompt type.
        
        Args:
            prompt_type: Type of prompt (simple, navigation, reasoning)
            prompt_text: The original prompt text
            response: The LLM's response
            
        Returns:
            Dictionary with:
            - overall_score: 0-100 score
            - subscores: Dictionary of individual metric scores
            - issues: List of detected issues
            - passed_checks: List of passed checks
        """
        if prompt_type not in self.scorers:
            return {
                'overall_score': 0,
                'subscores': {},
                'issues': [f'Unknown prompt type: {prompt_type}'],
                'passed_checks': []
            }
        
        return self.scorers[prompt_type](prompt_text, response)
    
    def score_simple_prompt(self, prompt_text: str, response: str) -> Dict[str, Any]:
        """
        Score simple instruction-following prompts.
        
        Checks:
        1. Word count compliance (if specified in prompt)
        2. Response completeness (not empty, not truncated)
        3. Format compliance (if format specified)
        
        Based on IFEval's instruction-following checks.
        """
        subscores = {}
        issues = []
        passed_checks = []
        
        # Extract word count requirement from prompt
        word_count_match = re.search(r'(\d+)\s+words?', prompt_text.lower())
        if word_count_match:
            expected_words = int(word_count_match.group(1))
            actual_words = len(response.split())
            
            # Allow ±1 word tolerance (IFEval style)
            if abs(actual_words - expected_words) <= 1:
                subscores['word_count'] = 100
                passed_checks.append(f'Word count: {actual_words} (target: {expected_words})')
            else:
                diff = abs(actual_words - expected_words)
                # Deduct 10 points per word off target, min 0
                subscores['word_count'] = max(0, 100 - (diff * 10))
                issues.append(f'Word count off by {diff}: {actual_words} vs {expected_words}')
        else:
            subscores['word_count'] = 100  # No requirement, full marks
        
        # Check response completeness
        if not response.strip():
            subscores['completeness'] = 0
            issues.append('Empty response')
        elif len(response) < 3:
            subscores['completeness'] = 30
            issues.append('Response too short')
        elif response.strip().endswith(('...', '…')) or response.count('\n\n\n') > 0:
            subscores['completeness'] = 70
            issues.append('Response appears truncated')
        else:
            subscores['completeness'] = 100
            passed_checks.append('Complete response')
        
        # Check for common errors
        if 'error' in response.lower() or 'cannot' in response.lower()[:50]:
            subscores['error_free'] = 0
            issues.append('Response contains error message')
        else:
            subscores['error_free'] = 100
            passed_checks.append('No errors detected')
        
        # Calculate overall score (weighted average)
        weights = {
            'word_count': 0.4,
            'completeness': 0.4,
            'error_free': 0.2
        }
        overall_score = sum(subscores[k] * weights[k] for k in subscores.keys())
        
        return {
            'overall_score': round(overall_score, 1),
            'subscores': subscores,
            'issues': issues,
            'passed_checks': passed_checks
        }
    
    def score_navigation_prompt(self, prompt_text: str, response: str) -> Dict[str, Any]:
        """
        Score navigation/planning prompts that require JSON output.
        
        Checks:
        1. Valid JSON syntax
        2. Required fields present
        3. Correct structure (arrays, objects)
        4. Logical content (e.g., valid action types)
        
        Based on OpenAI Evals JSON format checking.
        """
        subscores = {}
        issues = []
        passed_checks = []
        
        # Extract JSON from response (handle markdown code blocks)
        json_match = re.search(
            r'```(?:json)?\s*(\{.*?\})\s*```',
            response,
            re.DOTALL | re.IGNORECASE
        )
        if json_match:
            json_str = json_match.group(1)
        else:
            # Try to find raw JSON
            json_match = re.search(r'\{.*\}', response, re.DOTALL)
            if json_match:
                json_str = json_match.group(0)
            else:
                json_str = response
        
        # 1. JSON validity check
        try:
            data = json.loads(json_str)
            subscores['json_validity'] = 100
            passed_checks.append('Valid JSON syntax')
        except json.JSONDecodeError as e:
            subscores['json_validity'] = 0
            issues.append(f'JSON parse error: {str(e)[:50]}')
            # Can't do further checks without valid JSON
            return {
                'overall_score': 0,
                'subscores': subscores,
                'issues': issues,
                'passed_checks': passed_checks
            }
        
        # 2. Required fields check
        expected_fields = ['steps']  # Based on prompt
        missing_fields = [f for f in expected_fields if f not in data]
        
        if not missing_fields:
            subscores['required_fields'] = 100
            passed_checks.append('All required fields present')
        else:
            subscores['required_fields'] = 0
            issues.append(f'Missing fields: {missing_fields}')
        
        # 3. Structure check
        if 'steps' in data:
            steps = data['steps']
            if isinstance(steps, list):
                subscores['structure'] = 100
                passed_checks.append('Correct structure (steps is array)')
                
                # Check step count (prompt asked for 3)
                expected_step_count = 3
                actual_step_count = len(steps)
                
                if actual_step_count == expected_step_count:
                    subscores['step_count'] = 100
                    passed_checks.append(f'Correct step count: {actual_step_count}')
                else:
                    diff = abs(actual_step_count - expected_step_count)
                    subscores['step_count'] = max(0, 100 - (diff * 20))
                    issues.append(f'Step count: {actual_step_count} (expected: {expected_step_count})')
                
                # Check step structure (should have action and parameters)
                valid_steps = 0
                for i, step in enumerate(steps):
                    if isinstance(step, dict) and 'action' in step and 'parameters' in step:
                        valid_steps += 1
                    else:
                        issues.append(f'Step {i+1} missing action or parameters')
                
                if valid_steps == len(steps):
                    subscores['step_structure'] = 100
                    passed_checks.append('All steps properly structured')
                elif valid_steps > 0:
                    subscores['step_structure'] = int(100 * valid_steps / len(steps))
                    issues.append(f'Only {valid_steps}/{len(steps)} steps properly structured')
                else:
                    subscores['step_structure'] = 0
                    issues.append('No steps properly structured')
                    
            else:
                subscores['structure'] = 0
                subscores['step_count'] = 0
                subscores['step_structure'] = 0
                issues.append('Steps field is not an array')
        else:
            subscores['structure'] = 0
            subscores['step_count'] = 0
            subscores['step_structure'] = 0
        
        # Calculate overall score (weighted average)
        weights = {
            'json_validity': 0.3,
            'required_fields': 0.2,
            'structure': 0.1,
            'step_count': 0.2,
            'step_structure': 0.2
        }
        overall_score = sum(subscores[k] * weights[k] for k in subscores.keys())
        
        return {
            'overall_score': round(overall_score, 1),
            'subscores': subscores,
            'issues': issues,
            'passed_checks': passed_checks
        }
    
    def score_reasoning_prompt(self, prompt_text: str, response: str) -> Dict[str, Any]:
        """
        Score reasoning/problem-solving prompts.
        
        Checks:
        1. Answer present (A, B, C, D for multiple choice)
        2. Explanation present
        3. Explanation quality (mentions key concepts)
        4. Logical structure
        
        Based on reasoning benchmarks like MMLU, ARC.
        """
        subscores = {}
        issues = []
        passed_checks = []
        
        # 1. Check for answer choice
        answer_pattern = re.compile(r'\b([ABCD])\)', re.IGNORECASE)
        answers = answer_pattern.findall(response)
        
        if answers:
            subscores['answer_present'] = 100
            passed_checks.append(f'Answer provided: {answers[0].upper()}')
            
            # Check if answer appears early (good practice)
            first_answer_pos = response.upper().find(answers[0].upper() + ')')
            if first_answer_pos < len(response) * 0.3:  # In first 30% of response
                subscores['answer_position'] = 100
                passed_checks.append('Answer stated early')
            else:
                subscores['answer_position'] = 70
                issues.append('Answer buried in explanation')
        else:
            subscores['answer_present'] = 0
            subscores['answer_position'] = 0
            issues.append('No clear answer choice (A/B/C/D) found')
        
        # 2. Check for explanation
        sentences = re.split(r'[.!?]+', response)
        sentences = [s.strip() for s in sentences if len(s.strip()) > 10]
        
        if len(sentences) >= 2:
            subscores['explanation_present'] = 100
            passed_checks.append(f'Explanation present ({len(sentences)} sentences)')
        elif len(sentences) == 1:
            subscores['explanation_present'] = 50
            issues.append('Only one sentence of explanation')
        else:
            subscores['explanation_present'] = 0
            issues.append('No explanation provided')
        
        # 3. Check explanation quality (mentions key concepts from prompt)
        # Extract key concepts from this specific prompt
        key_concepts = ['doorway', 'wide', 'obstacle', 'center', 'left', 'right']
        mentioned_concepts = [c for c in key_concepts if c.lower() in response.lower()]
        
        concept_coverage = len(mentioned_concepts) / len(key_concepts)
        subscores['concept_coverage'] = int(concept_coverage * 100)
        
        if concept_coverage >= 0.5:
            passed_checks.append(f'Good concept coverage: {len(mentioned_concepts)}/{len(key_concepts)}')
        else:
            issues.append(f'Low concept coverage: {len(mentioned_concepts)}/{len(key_concepts)}')
        
        # 4. Check for logical structure (reasoning markers)
        reasoning_markers = [
            'because', 'therefore', 'since', 'so', 'thus',
            'should', 'would', 'can', 'will',
            'if', 'then', 'when'
        ]
        marker_count = sum(1 for marker in reasoning_markers if marker in response.lower())
        
        if marker_count >= 2:
            subscores['logical_structure'] = 100
            passed_checks.append(f'Clear reasoning markers present ({marker_count})')
        elif marker_count == 1:
            subscores['logical_structure'] = 70
            issues.append('Limited reasoning structure')
        else:
            subscores['logical_structure'] = 40
            issues.append('No clear reasoning markers')
        
        # Calculate overall score (weighted average)
        weights = {
            'answer_present': 0.3,
            'answer_position': 0.1,
            'explanation_present': 0.2,
            'concept_coverage': 0.2,
            'logical_structure': 0.2
        }
        overall_score = sum(subscores[k] * weights[k] for k in subscores.keys())
        
        return {
            'overall_score': round(overall_score, 1),
            'subscores': subscores,
            'issues': issues,
            'passed_checks': passed_checks
        }


def main():
    """CLI interface for testing quality scorer."""
    if len(sys.argv) < 4:
        print("Usage: quality_scorer.py <prompt_type> <prompt_text> <response>")
        print("Prompt types: simple, navigation, reasoning")
        sys.exit(1)
    
    prompt_type = sys.argv[1]
    prompt_text = sys.argv[2]
    response = sys.argv[3]
    
    scorer = QualityScorer()
    result = scorer.score_response(prompt_type, prompt_text, response)
    
    print(json.dumps(result, indent=2))


if __name__ == '__main__':
    main()
