"""
Minimal HTTP client for UR15 joint-space model served by serve_batch.py.

Usage:
    from workspace.yaobo.eaivis.ur15_client import predict

    action = predict(
        url="http://localhost:5500/api/inference",
        images=[img_np],                      # list of (H,W,3) uint8
        state={
            "ROBOT_RIGHT_JOINTS": np.zeros(6),
            "ROBOT_RIGHT_GRIPPER": np.zeros(1),
        },
        instruction="pick up the eraser",
    )
    # action = {
    #     "ROBOT_RIGHT_JOINTS": np.ndarray (chunk_size, 6),
    #     "ROBOT_RIGHT_GRIPPER": np.ndarray (chunk_size, 1),
    # }
"""

import io
import json
import numpy as np
import requests
from PIL import Image


def predict(
    url: str,
    images: list,
    state: dict,
    instruction: str,
    has_left: bool = False,
    has_right: bool = True,
    has_progress: bool = False,
) -> dict:
    """
    Send images + state to the serve API and return predicted action chunk.

    Args:
        url:         Serve endpoint, e.g. "http://localhost:5500/api/inference"
        images:      List of np.ndarray (H, W, 3) uint8 RGB images.
        state:       Dict of state arrays, e.g.:
                       {"ROBOT_RIGHT_JOINTS": np.ndarray (6,),
                        "ROBOT_RIGHT_GRIPPER": np.ndarray (1,)}
        instruction: Task description string (raw, without prefix).
        has_left:    Whether the model outputs left-arm actions.
        has_right:   Whether the model outputs right-arm actions.
        has_progress: Whether the model outputs progress.

    Returns:
        Dict of action arrays, e.g.:
          {"ROBOT_RIGHT_JOINTS": np.ndarray (chunk_size, 6),
           "ROBOT_RIGHT_GRIPPER": np.ndarray (chunk_size, 1)}
    """
    # --- Encode images as JPEG multipart files ---
    files = []
    for i, img_np in enumerate(images):
        img_pil = Image.fromarray(img_np)
        buf = io.BytesIO()
        img_pil.save(buf, format="JPEG")
        buf.seek(0)
        files.append((f"image_{i}", (f"image_{i}.jpg", buf, "image/jpeg")))

    # --- JSON payload ---
    state_serializable = {k: np.asarray(v).tolist() for k, v in state.items()}

    json_payload = {
        "task_description": instruction,
        "state": state_serializable,
        "use_state": True,
        "has_left": has_left,
        "has_right": has_right,
        "has_progress": has_progress,
    }

    json_buf = io.BytesIO(json.dumps(json_payload).encode("utf-8"))
    files.append(("json", ("json", json_buf, "application/json")))

    # --- Send request ---
    resp = requests.post(url, files=files, timeout=30)
    resp.raise_for_status()
    result = resp.json()

    # --- Convert lists back to numpy ---
    action = {}
    for key, value in result.items():
        action[key] = np.array(value, dtype=np.float32)

    return action


# ---------------------------------------------------------------------------
# Quick test
# ---------------------------------------------------------------------------
if __name__ == "__main__":
    import argparse

    parser = argparse.ArgumentParser(description="Test UR15 client")
    parser.add_argument("--url", type=str,
                        default="http://10.190.172.212:4500/api/inference")
    parser.add_argument("--instruction", type=str,
                        default="pick up the eraser")
    args = parser.parse_args()

    # Dummy inputs
    dummy_image = np.zeros((540, 960, 3), dtype=np.uint8)
    dummy_state = {
        "ROBOT_RIGHT_JOINTS": np.zeros(6, dtype=np.float32),
    }

    print(f"Sending request to {args.url} ...")
    action = predict(args.url, [dummy_image], dummy_state, args.instruction)

    for k, v in action.items():
        print(f"  {k}: shape={v.shape}, dtype={v.dtype}")
        print(f"    first step: {v[0]}")
