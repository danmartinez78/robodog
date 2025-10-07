#!/usr/bin/env python3
"""
Vision skills for ShadowHound using DIMOS Qwen VLM.

These skills leverage DIMOS's existing vision infrastructure:
- Qwen VLM for scene understanding and object detection
- Simple, synchronous API for easy testing and integration
- Structured output for agent consumption
"""

import os
import json
import logging
from datetime import datetime
from pathlib import Path
from typing import Optional, Dict, Any, Tuple
from dataclasses import dataclass
from PIL import Image
import numpy as np

# DIMOS vision imports - optional for SnapshotSkill, required for VLM skills
try:
    from dimos.models.qwen.video_query import (
        query_single_frame,
        get_bbox_from_qwen_frame
    )
    DIMOS_VISION_AVAILABLE = True
except ImportError as e:
    DIMOS_VISION_AVAILABLE = False
    DIMOS_VISION_ERROR = str(e)

logger = logging.getLogger(__name__)


@dataclass
class SkillResult:
    """Result from skill execution."""
    success: bool
    data: Optional[Dict[str, Any]] = None
    error: Optional[str] = None
    telemetry: Optional[Dict[str, Any]] = None


class VisionSkillBase:
    """Base class for vision skills."""
    
    def __init__(self, image_dir: Optional[Path] = None, require_dimos: bool = False):
        """
        Initialize vision skill.
        
        Args:
            image_dir: Directory to save captured images (default: /tmp/shadowhound/images)
            require_dimos: Whether this skill requires DIMOS vision APIs
        """
        if require_dimos and not DIMOS_VISION_AVAILABLE:
            raise ImportError(f"DIMOS vision not available: {DIMOS_VISION_ERROR}")
        
        self.image_dir = image_dir or Path("/tmp/shadowhound/images")
        self.image_dir.mkdir(parents=True, exist_ok=True)
        
        # Check for API key (only warn for VLM skills, don't fail)
        self.api_key = os.getenv("ALIBABA_API_KEY")
        if require_dimos and not self.api_key:
            logger.warning("ALIBABA_API_KEY not set. VLM skills may not function.")
    
    def _save_image(self, image: Image.Image, prefix: str = "frame") -> Path:
        """Save image with timestamp."""
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S_%f")
        filename = f"{prefix}_{timestamp}.jpg"
        path = self.image_dir / filename
        image.save(path, "JPEG", quality=85)
        logger.info(f"Saved image: {path}")
        return path
    
    def _numpy_to_pil(self, image: np.ndarray) -> Image.Image:
        """Convert numpy array to PIL Image."""
        if isinstance(image, Image.Image):
            return image
        # Convert BGR to RGB if needed
        if image.shape[-1] == 3:
            image = image[:, :, ::-1]  # BGR -> RGB
        return Image.fromarray(image)


class SnapshotSkill(VisionSkillBase):
    """Capture and save current camera frame. Does not require DIMOS."""
    
    def __init__(self, image_dir: Optional[Path] = None):
        # Snapshot doesn't need DIMOS, just PIL/numpy
        super().__init__(image_dir=image_dir, require_dimos=False)
    
    def execute(self, image: np.ndarray, **kwargs) -> SkillResult:
        """
        Capture current frame and save with metadata.
        
        Args:
            image: Current camera frame (numpy array or PIL Image)
            **kwargs: Optional metadata (pose, timestamp, etc.)
            
        Returns:
            SkillResult with image path and metadata
        """
        try:
            # Convert to PIL if needed
            pil_image = self._numpy_to_pil(image)
            
            # Save image
            image_path = self._save_image(pil_image, prefix="snapshot")
            
            # Collect metadata
            metadata = {
                "image_path": str(image_path),
                "timestamp": datetime.now().isoformat(),
                "size": pil_image.size,
                "format": pil_image.format or "JPEG"
            }
            
            # Add any additional kwargs as metadata
            for key, value in kwargs.items():
                if key not in metadata:
                    metadata[key] = value
            
            return SkillResult(
                success=True,
                data=metadata,
                telemetry={"image_size_bytes": image_path.stat().st_size}
            )
            
        except Exception as e:
            logger.error(f"Snapshot skill failed: {e}")
            return SkillResult(
                success=False,
                error=f"Failed to capture snapshot: {str(e)}"
            )


class DescribeSceneSkill(VisionSkillBase):
    """Use VLM to describe what's in the image. Requires DIMOS."""
    
    def __init__(self, image_dir: Optional[Path] = None):
        super().__init__(image_dir=image_dir, require_dimos=True)
    
    def execute(
        self,
        image: np.ndarray,
        query: Optional[str] = None,
        save_image: bool = True,
        **kwargs
    ) -> SkillResult:
        """
        Get VLM description of the scene.
        
        Args:
            image: Current camera frame
            query: Custom query (default: "Describe what you see in detail")
            save_image: Whether to save the image
            **kwargs: Additional parameters
            
        Returns:
            SkillResult with VLM description
        """
        try:
            # Convert to PIL
            pil_image = self._numpy_to_pil(image)
            
            # Save image if requested
            image_path = None
            if save_image:
                image_path = self._save_image(pil_image, prefix="describe")
            
            # Default query
            if query is None:
                query = (
                    "Describe what you see in this image in detail. "
                    "Include: main objects, their locations (left/right/center), "
                    "colors, and any notable features or activities."
                )
            
            # Query VLM
            logger.info(f"Querying VLM: {query[:50]}...")
            response = query_single_frame(
                pil_image,
                query=query,
                api_key=self.api_key
            )
            
            result_data = {
                "description": response,
                "query": query,
            }
            
            if image_path:
                result_data["image_path"] = str(image_path)
            
            return SkillResult(
                success=True,
                data=result_data,
                telemetry={
                    "query_length": len(query),
                    "response_length": len(response)
                }
            )
            
        except Exception as e:
            logger.error(f"Describe scene skill failed: {e}")
            return SkillResult(
                success=False,
                error=f"Failed to describe scene: {str(e)}"
            )


class LocateObjectSkill(VisionSkillBase):
    """Find specific object and return bounding box. Requires DIMOS."""
    
    def __init__(self, image_dir: Optional[Path] = None):
        super().__init__(image_dir=image_dir, require_dimos=True)
    
    def execute(
        self,
        image: np.ndarray,
        object_name: str,
        save_image: bool = True,
        **kwargs
    ) -> SkillResult:
        """
        Locate specific object in the scene.
        
        Args:
            image: Current camera frame
            object_name: Name of object to locate (e.g., "person", "chair")
            save_image: Whether to save the image
            **kwargs: Additional parameters
            
        Returns:
            SkillResult with bounding box if found
        """
        try:
            # Convert to PIL
            pil_image = self._numpy_to_pil(image)
            
            # Save image if requested
            image_path = None
            if save_image:
                image_path = self._save_image(pil_image, prefix=f"locate_{object_name}")
            
            # Get bounding box from Qwen
            logger.info(f"Locating object: {object_name}")
            bbox = get_bbox_from_qwen_frame(
                pil_image,
                object_name=object_name,
                api_key=self.api_key
            )
            
            # Process bbox result
            found = bbox is not None and len(bbox) == 4
            
            result_data = {
                "object_name": object_name,
                "found": found,
            }
            
            if found:
                x1, y1, x2, y2 = bbox
                result_data.update({
                    "bbox": [int(x1), int(y1), int(x2), int(y2)],
                    "center": [int((x1 + x2) / 2), int((y1 + y2) / 2)],
                    "width": int(x2 - x1),
                    "height": int(y2 - y1),
                    # Simple distance estimation (assumes object is ~0.5m tall)
                    "estimated_size_meters": 0.5
                })
            
            if image_path:
                result_data["image_path"] = str(image_path)
            
            return SkillResult(
                success=True,
                data=result_data,
                telemetry={"bbox_found": found}
            )
            
        except Exception as e:
            logger.error(f"Locate object skill failed: {e}")
            return SkillResult(
                success=False,
                error=f"Failed to locate object: {str(e)}"
            )


class DetectObjectsSkill(VisionSkillBase):
    """Detect all prominent objects in the scene. Requires DIMOS."""
    
    def __init__(self, image_dir: Optional[Path] = None):
        super().__init__(image_dir=image_dir, require_dimos=True)
    
    def execute(
        self,
        image: np.ndarray,
        save_image: bool = True,
        **kwargs
    ) -> SkillResult:
        """
        Detect all prominent objects in the scene.
        
        Args:
            image: Current camera frame
            save_image: Whether to save the image
            **kwargs: Additional parameters
            
        Returns:
            SkillResult with list of detected objects
        """
        try:
            # Convert to PIL
            pil_image = self._numpy_to_pil(image)
            
            # Save image if requested
            image_path = None
            if save_image:
                image_path = self._save_image(pil_image, prefix="detect")
            
            # Query VLM for objects
            query = (
                "List all prominent objects you see in this image. "
                "Format your response as a JSON array like: "
                '["person", "chair", "table", "laptop"]'
            )
            
            logger.info("Detecting objects in scene...")
            response = query_single_frame(
                pil_image,
                query=query,
                api_key=self.api_key
            )
            
            # Try to parse as JSON
            objects = []
            try:
                # Try to extract JSON array from response
                start_idx = response.find('[')
                end_idx = response.rfind(']')
                if start_idx != -1 and end_idx != -1:
                    json_str = response[start_idx:end_idx+1]
                    objects = json.loads(json_str)
            except (json.JSONDecodeError, ValueError):
                # If parsing fails, split by common separators
                logger.warning("Failed to parse JSON, falling back to text parsing")
                objects = [
                    obj.strip() 
                    for obj in response.replace(',', '\n').split('\n') 
                    if obj.strip() and not obj.strip().startswith('[') and not obj.strip().endswith(']')
                ]
            
            result_data = {
                "objects": objects,
                "count": len(objects),
                "raw_response": response
            }
            
            if image_path:
                result_data["image_path"] = str(image_path)
            
            return SkillResult(
                success=True,
                data=result_data,
                telemetry={
                    "object_count": len(objects),
                    "response_length": len(response)
                }
            )
            
        except Exception as e:
            logger.error(f"Detect objects skill failed: {e}")
            return SkillResult(
                success=False,
                error=f"Failed to detect objects: {str(e)}"
            )


# Registry of all vision skills
VISION_SKILLS = {
    "vision.snapshot": SnapshotSkill,
    "vision.describe_scene": DescribeSceneSkill,
    "vision.locate_object": LocateObjectSkill,
    "vision.detect_objects": DetectObjectsSkill,
}


def get_skill(name: str) -> Optional[VisionSkillBase]:
    """Get skill by name."""
    skill_class = VISION_SKILLS.get(name)
    if skill_class:
        try:
            return skill_class()
        except ImportError as e:
            logger.error(f"Failed to initialize skill {name}: {e}")
            return None
    return None


def list_skills() -> list:
    """List all available skills."""
    return list(VISION_SKILLS.keys())
