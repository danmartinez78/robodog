#!/usr/bin/env python3
"""Test script to verify LLM backend validation works.

This tests the validation logic without needing a full ROS node.
"""

import requests
import os
import sys

def test_ollama_validation(base_url: str, model: str):
    """Test Ollama validation logic."""
    print("=" * 60)
    print("🔍 TESTING OLLAMA BACKEND VALIDATION")
    print("=" * 60)
    print(f"URL: {base_url}")
    print(f"Model: {model}")
    
    # Step 1: Check if Ollama service is responding
    try:
        print("\n1. Checking Ollama service...")
        response = requests.get(
            f"{base_url}/api/tags",
            timeout=5
        )
        
        if response.status_code != 200:
            print(f"❌ Ollama service returned status {response.status_code}")
            return False
        
        print("✅ Ollama service responding")
        
    except requests.exceptions.Timeout:
        print(f"❌ Timeout connecting to Ollama at {base_url}")
        return False
        
    except requests.exceptions.ConnectionError as e:
        print(f"❌ Cannot connect to Ollama at {base_url}")
        print(f"   Error: {e}")
        return False
    
    # Step 2: Check if model is available
    try:
        print("\n2. Checking model availability...")
        models_data = response.json()
        available_models = [m.get('name', '') for m in models_data.get('models', [])]
        
        print(f"   Available models: {', '.join(available_models[:5])}")
        if len(available_models) > 5:
            print(f"   ... and {len(available_models) - 5} more")
        
        if model not in available_models:
            print(f"❌ Model '{model}' not found in Ollama")
            return False
        
        print(f"✅ Model '{model}' available")
        
    except Exception as e:
        print(f"❌ Failed to parse Ollama models list: {e}")
        return False
    
    # Step 3: Send a test prompt
    try:
        print("\n3. Sending test prompt...")
        test_response = requests.post(
            f"{base_url}/api/generate",
            json={
                "model": model,
                "prompt": "Say OK",
                "stream": False
            },
            timeout=30
        )
        
        if test_response.status_code != 200:
            print(f"❌ Test prompt failed with status {test_response.status_code}")
            return False
        
        response_data = test_response.json()
        response_text = response_data.get('response', '').strip()
        
        print(f"✅ Test prompt succeeded")
        print(f"   Response: '{response_text}'")
        
    except requests.exceptions.Timeout:
        print(f"❌ Timeout waiting for test prompt response (>30s)")
        return False
        
    except Exception as e:
        print(f"❌ Test prompt failed: {e}")
        return False
    
    # All checks passed
    print("\n" + "=" * 60)
    print("✅ Ollama backend validation PASSED")
    print("=" * 60)
    return True


def test_openai_validation(model: str):
    """Test OpenAI validation logic."""
    from openai import OpenAI
    
    print("=" * 60)
    print("🔍 TESTING OPENAI BACKEND VALIDATION")
    print("=" * 60)
    
    # Check API key
    api_key = os.getenv("OPENAI_API_KEY")
    if not api_key:
        print("❌ OPENAI_API_KEY environment variable not set")
        return False
    
    print("✅ OPENAI_API_KEY found")
    print(f"Model: {model}")
    
    # Send test prompt
    try:
        print("\nSending test prompt...")
        
        client = OpenAI(api_key=api_key)
        
        response = client.chat.completions.create(
            model=model,
            messages=[{"role": "user", "content": "Say OK"}],
            max_tokens=10,
            timeout=30
        )
        
        response_text = response.choices[0].message.content.strip()
        print(f"✅ Test prompt succeeded")
        print(f"   Response: '{response_text}'")
        
    except Exception as e:
        print(f"❌ OpenAI test prompt failed: {e}")
        return False
    
    # All checks passed
    print("\n" + "=" * 60)
    print("✅ OpenAI backend validation PASSED")
    print("=" * 60)
    return True


if __name__ == "__main__":
    # Test Ollama (Thor configuration from config files)
    THOR_IP = os.getenv("THOR_IP", "192.168.50.10")
    OLLAMA_URL = f"http://{THOR_IP}:11434"
    OLLAMA_MODEL = "qwen2.5-coder:32b"
    
    print("\n🧪 Testing Ollama Backend Validation")
    print("-" * 60)
    ollama_ok = test_ollama_validation(OLLAMA_URL, OLLAMA_MODEL)
    
    # Test OpenAI (if API key is set)
    if os.getenv("OPENAI_API_KEY"):
        print("\n🧪 Testing OpenAI Backend Validation")
        print("-" * 60)
        openai_ok = test_openai_validation("gpt-4-turbo")
    else:
        print("\n⏭️  Skipping OpenAI test (OPENAI_API_KEY not set)")
        openai_ok = None
    
    # Summary
    print("\n" + "=" * 60)
    print("📊 VALIDATION TEST SUMMARY")
    print("=" * 60)
    print(f"Ollama: {'✅ PASS' if ollama_ok else '❌ FAIL'}")
    if openai_ok is not None:
        print(f"OpenAI: {'✅ PASS' if openai_ok else '❌ FAIL'}")
    else:
        print("OpenAI: ⏭️  SKIPPED")
    print("=" * 60)
    
    # Exit with error if any test failed
    if not ollama_ok or (openai_ok is not None and not openai_ok):
        sys.exit(1)
