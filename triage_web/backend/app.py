from flask import Flask, Response, jsonify, send_from_directory
from ros_bridge import ROSBridge, start_ros
import threading
import rclpy
import requests
import os
import time

app = Flask(__name__)
ros_node = None

FRONTEND_DIR = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "frontend"))

# -------------------------
# Frontend routes
# -------------------------
@app.route("/")
def index():
    return send_from_directory(FRONTEND_DIR, "index.html")

@app.route("/assets/<path:filename>")
def assets(filename):
    return send_from_directory(os.path.join(FRONTEND_DIR, "assets"), filename)

# -------------------------
# ROS camera images
# -------------------------
@app.route("/image/<drone>")
def image(drone):
    if ros_node and drone in ros_node.latest_images:
        img_bytes = ros_node.latest_images[drone]
        if img_bytes:
            return Response(img_bytes, mimetype="image/jpeg")
    return "", 204

# -------------------------
# ROS casualty location
# -------------------------
@app.route("/location")
def location():
    if ros_node and ros_node.latest_location:
        data = ros_node.latest_location.copy()
        from datetime import datetime
        data["timestamp"] = datetime.utcnow().isoformat() + "Z"
        return jsonify(data)
    return jsonify({})

# -------------------------
# Operator IP geolocation
# -------------------------
@app.route("/ip_location")
def ip_location():
    try:
        r = requests.get("https://ipapi.co/json/", timeout=2)
        d = r.json()
        return jsonify({
            "lat": d.get("latitude"),
            "lon": d.get("longitude"),
            "city": d.get("city"),
            "region": d.get("region"),
            "country": d.get("country_name"),
            "ip": d.get("ip")
        })
    except Exception:
        return jsonify({})

# -------------------------
# Drone status for panel
# -------------------------
@app.route("/drone_status")
def drone_status():
    if not ros_node:
        return jsonify(["unknown"]*5)
    statuses = []
    for drone in ["drone1","drone2","drone3","drone4","drone5"]:
        img = ros_node.latest_images.get(drone)
        statuses.append("operational" if img else "disconnected")
    return jsonify(statuses)

# -------------------------
# Casualty locations for map
# -------------------------
@app.route("/casualty_locations")
def casualty_locations():
    if ros_node and ros_node.latest_location:
        return jsonify([{
            "x": ros_node.latest_location["lat"],
            "y": ros_node.latest_location["lon"],
            "confidence_radius": 50
        }])
    return jsonify([])

# -------------------------
# Drone telemetry (for map markers)
# -------------------------
@app.route("/drone_telemetry")
def drone_telemetry():
    if not ros_node or not ros_node.latest_location:
        base_lat, base_lon = 20, 0
    else:
        base_lat = ros_node.latest_location["lat"]
        base_lon = ros_node.latest_location["lon"]

    drones = []
    for i in range(5):
        drones.append({
            "id": f"drone{i+1}",
            "lat": base_lat + 0.001*i,
            "lon": base_lon + 0.001*i,
            "autonomy": "auto" if i % 2 == 0 else "manual",
            "signal": "good" if ros_node.latest_images.get(f"drone{i+1}") else "lost",
            "battery": 100 - i*12
        })
    return jsonify(drones)

# -------------------------
# Mission timer
# -------------------------
MISSION_START = time.time()
@app.route("/mission_time")
def mission_time():
    elapsed = time.time() - MISSION_START
    hours, rem = divmod(int(elapsed), 3600)
    minutes, seconds = divmod(rem, 60)
    return jsonify({"hours": hours, "minutes": minutes, "seconds": seconds})

# -------------------------
# Main
# -------------------------
def main():
    global ros_node

    rclpy.init()
    ros_node = ROSBridge()

    ros_thread = threading.Thread(target=start_ros, args=(ros_node,), daemon=True)
    ros_thread.start()

    app.run(host="0.0.0.0", port=8080, debug=False)

if __name__ == "__main__":
    main()
