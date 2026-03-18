"""Web server for Vision Tracker 6D.

Provides a Flask REST API and single-page web UI for monitoring and
controlling the chessboard pose tracker.  The backend detection loop
runs regardless of whether any browser is connected.
"""

import json
import os
import threading
import time

import cv2
from flask import Flask, Response, jsonify, request, send_from_directory

from .tracker_node import VisionTracker


class VisionTrackerWebServer:
    """Combines a VisionTracker backend with a Flask web server."""

    def __init__(self):
        self.tracker = VisionTracker()
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
            return send_from_directory(static_dir, "index.html")

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

        # --- Chessboard config ------------------------------------------------

        @app.route("/api/chessboard", methods=["GET"])
        def get_chessboard():
            s = self.tracker.get_status()
            return jsonify(s["chessboard"])

        @app.route("/api/chessboard", methods=["POST"])
        def set_chessboard():
            data = request.get_json(force=True)
            rows = int(data.get("rows", 7))
            cols = int(data.get("cols", 9))
            square_size = float(data.get("square_size", 0.025))
            self.tracker.update_chessboard(rows, cols, square_size)
            return jsonify({"rows": rows, "cols": cols, "square_size": square_size})

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

            return Response(
                generate(),
                mimetype="multipart/x-mixed-replace; boundary=frame",
            )

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
                    yield f"data: {json.dumps(data)}\n\n"
                    time.sleep(0.1)

            return Response(generate(), mimetype="text/event-stream")

        return app

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
