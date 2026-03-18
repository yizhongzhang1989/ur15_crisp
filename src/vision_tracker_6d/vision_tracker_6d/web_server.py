"""Web server for Vision Tracker 6D.

Provides a Flask REST API and single-page web UI for monitoring and
controlling the chessboard pose tracker.  The backend detection loop
runs regardless of whether any browser is connected.
"""

import glob
import json
import os
import sys
import threading
import time

import cv2
import numpy as np
from flask import Flask, Response, jsonify, request, send_from_directory

from common.workspace import get_workspace_root
from .tracker_node import VisionTracker

# Add camera_calibration_toolkit to path so we can import its modules
def _find_toolkit_dir():
    """Locate the camera_calibration_toolkit directory."""
    # Try relative to source tree
    src_dir = os.path.join(
        os.path.dirname(os.path.abspath(__file__)),
        "..", "ThirdParty", "camera_calibration_toolkit",
    )
    if os.path.isdir(src_dir):
        return os.path.normpath(src_dir)
    # Try via workspace root
    try:
        ws = get_workspace_root()
        ws_dir = os.path.join(
            ws, "src", "vision_tracker_6d",
            "ThirdParty", "camera_calibration_toolkit",
        )
        if os.path.isdir(ws_dir):
            return ws_dir
    except Exception:
        pass
    return None

_TOOLKIT_DIR = _find_toolkit_dir()
if _TOOLKIT_DIR and _TOOLKIT_DIR not in sys.path:
    sys.path.insert(0, _TOOLKIT_DIR)


class VisionTrackerWebServer:
    """Combines a VisionTracker backend with a Flask web server."""

    def __init__(self):
        self.tracker = VisionTracker()
        self._calib_running = False
        self._calib_result = None
        self._calib_error = None
        self.app = self._create_app()

    # ------------------------------------------------------------------
    # Flask app factory
    # ------------------------------------------------------------------

    def _create_app(self) -> Flask:
        static_dir = self._find_static_dir()
        app = Flask(__name__, static_folder=static_dir)

        # --- Pages -----------------------------------------------------------

        @app.route("/")
        def index():
            resp = send_from_directory(static_dir, "index.html")
            resp.headers["Cache-Control"] = "no-cache, no-store, must-revalidate"
            return resp

        # --- Status -----------------------------------------------------------

        @app.route("/api/status")
        def status():
            return jsonify(self.tracker.get_status())

        # --- Tracking control -------------------------------------------------

        @app.route("/api/tracking", methods=["POST"])
        def set_tracking():
            data = request.get_json(force=True)
            enabled = bool(data.get("enabled", True))
            self.tracker.set_tracking_enabled(enabled)
            return jsonify({"tracking_enabled": enabled})

        # --- Camera list ------------------------------------------------------

        @app.route("/api/cameras")
        def cameras():
            s = self.tracker.get_status()
            return jsonify(s["cameras"])

        # --- MJPEG video feed -------------------------------------------------

        @app.route("/api/video_feed/<camera_name>")
        def video_feed(camera_name):
            """MJPEG stream of annotated frames for a camera."""
            def generate():
                while True:
                    frame = self.tracker.get_latest_annotated_frame(camera_name)
                    if frame is not None:
                        ret, jpeg = cv2.imencode(
                            ".jpg", frame, [cv2.IMWRITE_JPEG_QUALITY, 80]
                        )
                        if ret:
                            yield (
                                b"--frame\r\n"
                                b"Content-Type: image/jpeg\r\n\r\n"
                                + jpeg.tobytes()
                                + b"\r\n"
                            )
                    time.sleep(0.033)  # ~30 fps cap

            resp = Response(
                generate(),
                mimetype="multipart/x-mixed-replace; boundary=frame",
            )
            resp.headers["Cache-Control"] = "no-cache, no-store, must-revalidate"
            resp.headers["Pragma"] = "no-cache"
            resp.headers["Access-Control-Allow-Origin"] = "*"
            return resp

        # --- Snapshot ---------------------------------------------------------

        @app.route("/api/snapshot/<camera_name>")
        def snapshot(camera_name):
            """Single JPEG snapshot of the latest annotated frame."""
            frame = self.tracker.get_latest_annotated_frame(camera_name)
            if frame is None:
                return jsonify({"error": "no frame available"}), 404
            ret, jpeg = cv2.imencode(".jpg", frame)
            if not ret:
                return jsonify({"error": "encode failed"}), 500
            return Response(jpeg.tobytes(), mimetype="image/jpeg")

        # --- SSE stream -------------------------------------------------------

        @app.route("/api/stream")
        def stream():
            """Server-Sent Events stream of tracker status at ~10 Hz."""
            def generate():
                while True:
                    data = self.tracker.get_status()
                    data["calib_image_count"] = self._count_calib_images()
                    yield f"data: {json.dumps(data)}\n\n"
                    time.sleep(0.1)

            return Response(generate(), mimetype="text/event-stream")

        # --- Intrinsic calibration --------------------------------------------

        @app.route("/api/calib/capture", methods=["POST"])
        def calib_capture():
            """Capture current frame and save as calibration image."""
            data = request.get_json(force=True) if request.data else {}
            camera_name = data.get("camera")
            # Default to first camera
            if not camera_name:
                names = self.tracker.get_camera_names()
                camera_name = names[0] if names else None
            if not camera_name:
                return jsonify({"error": "no camera available"}), 404

            frame = self.tracker.get_latest_raw_frame(camera_name)
            if frame is None:
                return jsonify({"error": "no frame available"}), 404

            img_dir = self._calib_images_dir()
            os.makedirs(img_dir, exist_ok=True)
            idx = len(glob.glob(os.path.join(img_dir, "*.png")))
            filename = f"calib_{idx:04d}.png"
            filepath = os.path.join(img_dir, filename)
            cv2.imwrite(filepath, frame)
            count = len(glob.glob(os.path.join(img_dir, "*.png")))
            return jsonify({"saved": filename, "count": count})

        @app.route("/api/calib/images", methods=["GET"])
        def calib_images_list():
            """List captured calibration images."""
            img_dir = self._calib_images_dir()
            if not os.path.isdir(img_dir):
                return jsonify({"images": [], "count": 0})
            files = sorted(glob.glob(os.path.join(img_dir, "*.png")))
            names = [os.path.basename(f) for f in files]
            return jsonify({"images": names, "count": len(names)})

        @app.route("/api/calib/images/<filename>")
        def calib_image_view(filename):
            """View a captured calibration image."""
            img_dir = self._calib_images_dir()
            safe_name = os.path.basename(filename)
            return send_from_directory(img_dir, safe_name)

        @app.route("/api/calib/images/delete", methods=["POST"])
        def calib_images_delete():
            """Delete all captured calibration images."""
            img_dir = self._calib_images_dir()
            if os.path.isdir(img_dir):
                for f in glob.glob(os.path.join(img_dir, "*.png")):
                    os.remove(f)
            return jsonify({"count": 0})

        @app.route("/api/calib/run", methods=["POST"])
        def calib_run():
            """Run intrinsic calibration on captured images."""
            if self._calib_running:
                return jsonify({"error": "calibration already running"}), 409

            img_dir = self._calib_images_dir()
            images = sorted(glob.glob(os.path.join(img_dir, "*.png")))
            if len(images) < 3:
                return jsonify({"error": f"need at least 3 images, have {len(images)}"}), 400

            # Load pattern config
            pattern_path = self._calib_pattern_path()
            if not os.path.isfile(pattern_path):
                return jsonify({"error": f"pattern config not found: {pattern_path}"}), 404

            # Run in background thread
            self._calib_running = True
            self._calib_result = None
            self._calib_error = None
            t = threading.Thread(
                target=self._run_calibration,
                args=(images, pattern_path),
                daemon=True,
            )
            t.start()
            return jsonify({"status": "started", "image_count": len(images)})

        @app.route("/api/calib/status", methods=["GET"])
        def calib_status():
            """Get calibration status and result."""
            return jsonify({
                "running": self._calib_running,
                "result": self._calib_result,
                "error": self._calib_error,
                "image_count": self._count_calib_images(),
            })

        @app.route("/api/calib/pattern", methods=["GET"])
        def calib_pattern_get():
            """Get the current calibration pattern config."""
            path = self._calib_pattern_path()
            if not os.path.isfile(path):
                return jsonify({"error": "not found"}), 404
            with open(path, "r") as f:
                return jsonify(json.load(f))

        return app

    # ------------------------------------------------------------------
    # Calibration helpers
    # ------------------------------------------------------------------

    def _calib_images_dir(self) -> str:
        return os.path.join(get_workspace_root(), "tmp", "intrinsic_calib_images")

    def _calib_results_dir(self) -> str:
        return os.path.join(get_workspace_root(), "tmp", "intrinsic_calib_results")

    def _calib_pattern_path(self) -> str:
        return os.path.join(get_workspace_root(), "config", "calibration_pattern.json")

    def _count_calib_images(self) -> int:
        d = self._calib_images_dir()
        if not os.path.isdir(d):
            return 0
        return len(glob.glob(os.path.join(d, "*.png")))

    def _run_calibration(self, image_paths: list, pattern_path: str):
        """Run intrinsic calibration in a background thread."""
        try:
            from core.intrinsic_calibration import IntrinsicCalibrator
            from core.calibration_patterns import load_pattern_from_json

            with open(pattern_path, "r") as f:
                pattern_data = json.load(f)
            pattern = load_pattern_from_json(pattern_data)

            calibrator = IntrinsicCalibrator(
                image_paths=image_paths,
                calibration_pattern=pattern,
            )
            result = calibrator.calibrate(verbose=True)

            if result is None:
                self._calib_error = "Calibration failed — not enough valid detections"
                self._calib_running = False
                return

            out_dir = self._calib_results_dir()
            calibrator.generate_calibration_report(out_dir, verbose=True)

            # Save compact result to config/
            cam_mtx = result["camera_matrix"]
            dist = result["distortion_coefficients"]
            image_size = list(calibrator.image_size) if calibrator.image_size else [0, 0]
            compact_result = {
                "camera_matrix": cam_mtx.tolist(),
                "distortion_coefficients": dist.tolist(),
                "image_size": image_size,
                "rms_error": float(result["rms_error"]),
            }
            result_path = os.path.join(
                get_workspace_root(), "config", "intrinsic_calibration_result.json"
            )
            with open(result_path, "w") as f:
                json.dump(compact_result, f, indent=2)

            self._calib_result = {
                "rms_error": round(float(result["rms_error"]), 4),
                "fx": round(float(cam_mtx[0, 0]), 2),
                "fy": round(float(cam_mtx[1, 1]), 2),
                "cx": round(float(cam_mtx[0, 2]), 2),
                "cy": round(float(cam_mtx[1, 2]), 2),
                "dist_coeffs": [round(float(c), 6) for c in dist],
                "image_size": image_size,
                "output_dir": out_dir,
                "result_path": result_path,
            }
        except Exception as e:
            self._calib_error = str(e)
        finally:
            self._calib_running = False

    # ------------------------------------------------------------------
    # Helpers
    # ------------------------------------------------------------------

    def _find_static_dir(self) -> str:
        """Find static files — works in dev and installed modes."""
        try:
            from ament_index_python.packages import get_package_share_directory
            installed = os.path.join(
                get_package_share_directory("vision_tracker_6d"), "static"
            )
            if os.path.isdir(installed):
                return installed
        except Exception:
            pass
        here = os.path.dirname(os.path.abspath(__file__))
        return os.path.join(here, "static")

    # ------------------------------------------------------------------
    # Run
    # ------------------------------------------------------------------

    def run(self, host: str = "0.0.0.0", port: int = 8090):
        # Start the backend detection loop
        self.tracker.start()

        # ROS spin in a background thread
        ros_thread = threading.Thread(
            target=self.tracker.spin, daemon=True
        )
        ros_thread.start()

        web_cfg = self.tracker.config.get("web", {})
        port = web_cfg.get("port", port)

        print(f"\n  Vision Tracker 6D Web: http://{host}:{port}\n")
        self.app.run(host=host, port=port, threaded=True)

    def shutdown(self):
        self.tracker.shutdown()


def main():
    server = VisionTrackerWebServer()
    try:
        server.run()
    except KeyboardInterrupt:
        pass
    finally:
        server.shutdown()


if __name__ == "__main__":
    main()
