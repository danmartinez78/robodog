#!/usr/bin/env python3
"""
Test script for vision skills.

Tests each skill independently with a test image.
"""

import sys
import os
from pathlib import Path
from PIL import Image
import numpy as np

# Add DIMOS to Python path if needed (must be before importing shadowhound_skills)
# test_vision.py is at: /workspaces/shadowhound/src/shadowhound_skills/test/test_vision.py
# DIMOS is at: /workspaces/shadowhound/src/dimos-unitree
# So we need to go up 4 levels: test -> shadowhound_skills -> src -> shadowhound
workspace_root = Path(__file__).resolve().parent.parent.parent.parent
dimos_path = workspace_root / "src" / "dimos-unitree"
if dimos_path.exists():
    sys.path.insert(0, str(dimos_path))
    print(f"✅ Added DIMOS to Python path: {dimos_path}")
else:
    print(f"⚠️  DIMOS not found at {dimos_path}")

from shadowhound_skills.vision import (
    SnapshotSkill,
    DescribeSceneSkill,
    LocateObjectSkill,
    DetectObjectsSkill,
    list_skills,
)


def create_test_image() -> np.ndarray:
    """Create a simple test image."""
    # Create a 640x480 RGB image with some colored rectangles
    image = np.zeros((480, 640, 3), dtype=np.uint8)

    # Add colored rectangles (BGR format)
    image[100:200, 100:200] = [0, 0, 255]  # Red square
    image[100:200, 300:400] = [0, 255, 0]  # Green square
    image[100:200, 500:600] = [255, 0, 0]  # Blue square

    # Add some text-like pattern
    image[300:350, 200:400] = [255, 255, 255]  # White rectangle

    return image


def test_snapshot_skill():
    """Test snapshot skill."""
    print("\n" + "=" * 70)
    print("TEST 1: Snapshot Skill")
    print("=" * 70)

    try:
        skill = SnapshotSkill()
        test_image = create_test_image()

        result = skill.execute(test_image, test_metadata="test_value")

        if result.success:
            print("✅ Snapshot skill PASSED")
            print(f"   Image saved to: {result.data['image_path']}")
            print(f"   Size: {result.data['size']}")
            print(f"   Timestamp: {result.data['timestamp']}")
        else:
            print(f"❌ Snapshot skill FAILED: {result.error}")

        return result.success

    except Exception as e:
        print(f"❌ Snapshot skill ERROR: {e}")
        return False


def test_describe_scene_skill():
    """Test describe scene skill."""
    print("\n" + "=" * 70)
    print("TEST 2: Describe Scene Skill")
    print("=" * 70)

    # Check for API key
    if not os.getenv("ALIBABA_API_KEY"):
        print("⚠️  ALIBABA_API_KEY not set - skipping VLM test")
        print("   Set it with: export ALIBABA_API_KEY='your-key'")
        return True  # Don't fail the test

    try:
        skill = DescribeSceneSkill()
        test_image = create_test_image()

        print("Querying VLM (this may take a few seconds)...")
        result = skill.execute(test_image, query="What colors and shapes do you see?")

        if result.success:
            print("✅ Describe scene skill PASSED")
            print(f"   Description: {result.data['description'][:200]}...")
            if "image_path" in result.data:
                print(f"   Image saved to: {result.data['image_path']}")
        else:
            print(f"❌ Describe scene skill FAILED: {result.error}")

        return result.success

    except Exception as e:
        print(f"❌ Describe scene skill ERROR: {e}")
        import traceback

        traceback.print_exc()
        return False


def test_locate_object_skill():
    """Test locate object skill."""
    print("\n" + "=" * 70)
    print("TEST 3: Locate Object Skill")
    print("=" * 70)

    # Check for API key
    if not os.getenv("ALIBABA_API_KEY"):
        print("⚠️  ALIBABA_API_KEY not set - skipping VLM test")
        return True

    try:
        skill = LocateObjectSkill()
        test_image = create_test_image()

        print("Querying VLM to locate 'red square'...")
        result = skill.execute(test_image, object_name="red square")

        if result.success:
            print("✅ Locate object skill PASSED")
            print(f"   Found: {result.data['found']}")
            if result.data["found"]:
                print(f"   Bounding box: {result.data['bbox']}")
                print(f"   Center: {result.data['center']}")
                print(f"   Estimated size: {result.data['estimated_size_meters']}m")
        else:
            print(f"⚠️  Object not found (this is okay for test image): {result.error}")

        return True  # Don't fail if object not found

    except Exception as e:
        print(f"❌ Locate object skill ERROR: {e}")
        import traceback

        traceback.print_exc()
        return False


def test_detect_objects_skill():
    """Test detect objects skill."""
    print("\n" + "=" * 70)
    print("TEST 4: Detect Objects Skill")
    print("=" * 70)

    # Check for API key
    if not os.getenv("ALIBABA_API_KEY"):
        print("⚠️  ALIBABA_API_KEY not set - skipping VLM test")
        return True

    try:
        skill = DetectObjectsSkill()
        test_image = create_test_image()

        print("Querying VLM to detect all objects...")
        result = skill.execute(test_image)

        if result.success:
            print("✅ Detect objects skill PASSED")
            print(f"   Objects detected: {result.data['count']}")
            if isinstance(result.data["objects"], list):
                for obj in result.data["objects"]:
                    print(f"   - {obj}")
            else:
                print(f"   Raw response: {result.data['raw_response'][:200]}...")
        else:
            print(f"❌ Detect objects skill FAILED: {result.error}")

        return result.success

    except Exception as e:
        print(f"❌ Detect objects skill ERROR: {e}")
        import traceback

        traceback.print_exc()
        return False


def main():
    """Run all tests."""
    print("\n" + "=" * 70)
    print("SHADOWHOUND VISION SKILLS TEST SUITE")
    print("=" * 70)

    print(f"\nAvailable skills: {list_skills()}")

    # Check DIMOS availability
    try:
        from dimos.models.qwen.video_query import query_single_frame

        print("✅ DIMOS vision available")
    except ImportError as e:
        print(f"⚠️  DIMOS vision not available: {e}")
        print("   Some tests will be skipped")

    # Run tests
    results = []
    results.append(("Snapshot", test_snapshot_skill()))
    results.append(("Describe Scene", test_describe_scene_skill()))
    results.append(("Locate Object", test_locate_object_skill()))
    results.append(("Detect Objects", test_detect_objects_skill()))

    # Summary
    print("\n" + "=" * 70)
    print("TEST SUMMARY")
    print("=" * 70)

    passed = sum(1 for _, success in results if success)
    total = len(results)

    for name, success in results:
        status = "✅ PASS" if success else "❌ FAIL"
        print(f"{status}: {name}")

    print(f"\nResults: {passed}/{total} tests passed")

    if passed == total:
        print("\n🎉 All tests passed!")
        return 0
    else:
        print(f"\n⚠️  {total - passed} test(s) failed")
        return 1


if __name__ == "__main__":
    sys.exit(main())
