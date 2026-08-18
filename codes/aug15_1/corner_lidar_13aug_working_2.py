import sys
import time
import math
import threading
import numpy as np
from flask import Flask, render_template_string, jsonify

try:
    from lidar_steering_new import LidarScanner
except ImportError as e:
    print(f"[SYSTEM ERROR] Failed to import lidar_steering_new: {e}")
    sys.exit(1)

# --- CONFIGURATION ---
LIDAR_PORT = "/dev/ttyUSB0"
LIDAR_BAUD = 230400

# Angular Scan Bounds (Limits LiDAR dataset strictly between -100 deg and +100 deg)
SCAN_MIN_ANGLE_DEG = -100.0
SCAN_MAX_ANGLE_DEG = 100.0

FRONT_SCAN_ANGLE_DEG = 15
MAX_SEGMENT_GAP_MM = 150.0  # Max point gap within a single wall

# --- Split-and-Merge line extraction ---
CORNER_SPLIT_DIST_THRESHOLD_MM = 35.0

# L-Shape Detection Thresholds
MIN_WALL_LENGTH_MM = 300.0   # Minimum length for a segment to qualify as a wall
CORNER_CORNER_GAP_MM = 250.0 # Max distance between endpoints to form a vertex
ANGLE_MIN_DEG = 70.0        # Min angle between walls for an L-shape
ANGLE_MAX_DEG = 110.0       # Max angle between walls for an L-shape
CORNER_PROXIMITY_THRESHOLD_MM = 50.0

# --- Custom Corner Detected Flag Thresholds ---
FLAG_FRONT_MIN_MM = 700.0
FLAG_FRONT_MAX_MM = 1050.0
FLAG_CORNER_DIST_MIN_MM = 800.0
FLAG_CORNER_DIST_MAX_MM = 1300.0
FLAG_SUM_LR_MIN_MM = 900.0

app = Flask(__name__)

latest_state = {
    "is_corner": False,
    "corner_detected": False,  # Custom trigger flag
    "fps": 0.0,                # Live LiDAR processing FPS
    "front": 2000.0,
    "left": 2000.0,
    "right": 2000.0,
    "left_avg_side": 0.0,
    "right_avg_side": 0.0,
    "sum_avg_side": 0.0,
    "segments": [],
    "fitted_walls": [],       
    "corners": [],            
    "corner_candidates": []
}
state_lock = threading.Lock()

# --- GEOMETRIC MATH & VECTOR HELPERS ---

def fit_line_vector(points):
    """Fits a best-fit line vector and endpoints to a list of (x,y) points using SVD/PCA."""
    pts = np.array([[p["x"], p["y"]] for p in points])
    centroid = np.mean(pts, axis=0)
    pts_centered = pts - centroid
    _, _, vh = np.linalg.svd(pts_centered)
    direction = vh[0]  # Unit line direction vector (dx, dy)
    
    projections = np.dot(pts_centered, direction)
    min_t, max_t = np.min(projections), np.max(projections)
    
    p_min = centroid + min_t * direction
    p_max = centroid + max_t * direction
    
    start_pt = {"x": float(p_min[0]), "y": float(p_min[1])}
    end_pt = {"x": float(p_max[0]), "y": float(p_max[1])}
    
    return centroid, direction, start_pt, end_pt

def split_segment(points, dist_threshold=CORNER_SPLIT_DIST_THRESHOLD_MM):
    """Recursively splits point sequences into straight sub-segments using Ramer-Douglas-Peucker."""
    if len(points) < 3:
        return [points]

    p_start = np.array([points[0]["x"], points[0]["y"]])
    p_end = np.array([points[-1]["x"], points[-1]["y"]])
    line_vec = p_end - p_start
    line_len = np.linalg.norm(line_vec)

    max_dist = -1.0
    max_idx = -1

    if line_len < 1e-6:
        for i in range(1, len(points) - 1):
            p = np.array([points[i]["x"], points[i]["y"]])
            d = np.linalg.norm(p - p_start)
            if d > max_dist:
                max_dist = d
                max_idx = i
    else:
        line_unit = line_vec / line_len
        for i in range(1, len(points) - 1):
            p = np.array([points[i]["x"], points[i]["y"]])
            proj_len = np.dot(p - p_start, line_unit)
            proj_point = p_start + proj_len * line_unit
            d = np.linalg.norm(p - proj_point)
            if d > max_dist:
                max_dist = d
                max_idx = i

    if max_dist > dist_threshold:
        left = split_segment(points[:max_idx + 1], dist_threshold)
        right = split_segment(points[max_idx:], dist_threshold)
        return left + right
    else:
        return [points]

def build_valid_walls(segments):
    """Splits gap segments and fits parametric straight wall lines."""
    straightened_segments = []
    for seg in segments:
        straightened_segments.extend(split_segment(seg))

    valid_walls = []
    for seg in straightened_segments:
        # Check point count and fitted length
        if len(seg) >= 4:
            centroid, dir_vec, fitted_start, fitted_end = fit_line_vector(seg)
            fitted_len = math.hypot(fitted_end["x"] - fitted_start["x"], fitted_end["y"] - fitted_start["y"])
            if fitted_len >= MIN_WALL_LENGTH_MM:
                valid_walls.append({
                    "points": seg,
                    "start": fitted_start,
                    "end": fitted_end,
                    "dir": dir_vec,
                    "centroid": centroid
                })
    return valid_walls, straightened_segments

def closest_endpoint_pair(w1, w2):
    """Returns closest pair of endpoints between two walls."""
    endpoint_pairs = [
        (w1["end"], w2["start"]),
        (w1["start"], w2["end"]),
        (w1["end"], w2["end"]),
        (w1["start"], w2["start"]),
    ]
    endpoint_distances = [
        math.hypot(pa["x"] - pb["x"], pa["y"] - pb["y"]) for pa, pb in endpoint_pairs
    ]
    min_idx = int(np.argmin(endpoint_distances))
    p1, p2 = endpoint_pairs[min_idx]
    return p1, p2, endpoint_distances[min_idx]

def wall_pair_angle_deg(w1, w2):
    """Orientation-invariant angle (0-90 deg) between two walls."""
    dot_product = np.clip(np.dot(w1["dir"], w2["dir"]), -1.0, 1.0)
    return np.degrees(np.arccos(abs(dot_product)))

def detect_l_shapes(valid_walls):
    """Detects L-shape (~90 deg) corners between wall pairs and calculates distance to robot origin."""
    candidates = []
    all_corners = []

    for i in range(len(valid_walls)):
        for j in range(i + 1, len(valid_walls)):
            w1 = valid_walls[i]
            w2 = valid_walls[j]

            p1, p2, min_dist = closest_endpoint_pair(w1, w2)
            if min_dist > CORNER_CORNER_GAP_MM:
                continue

            angle_deg = wall_pair_angle_deg(w1, w2)
            if not (ANGLE_MIN_DEG <= angle_deg <= ANGLE_MAX_DEG):
                continue

            gap_line = [p1, p2]
            vx = round((p1["x"] + p2["x"]) / 2, 1)
            vy = round((p1["y"] + p2["y"]) / 2, 1)
            
            dist_to_corner = round(math.hypot(vx, vy), 1)

            vertex = {
                "x": vx, 
                "y": vy,
                "dist_to_robot_mm": dist_to_corner
            }
            all_corners.append(vertex)

            far2 = w2["start"] if p2 is w2["end"] else w2["end"]
            boundary_y = vertex["y"]
            point_inner = {"x": vertex["x"], "y": boundary_y}
            point_outer = {"x": far2["x"], "y": boundary_y}
            boundary_line = [point_inner, point_outer]

            confirmed = dist_to_corner <= CORNER_PROXIMITY_THRESHOLD_MM

            candidates.append({
                "vertex": vertex,
                "line": gap_line,
                "boundary_line": boundary_line,
                "distance_mm": round(dist_to_corner, 1),
                "confirmed": confirmed
            })

    return all_corners, candidates

# --- STANDALONE CORNER DETECTION EVALUATOR ---

def evaluate_corner_detection(scan_data):
    """
    Accepts raw LiDAR scan data dictionary {angle_deg: distance_mm}.
    Returns tuple: (corner_detected_flag, metadata_dict)
    """
    if not scan_data:
        return False, {}

    sorted_angles = sorted(scan_data.keys())
    segments = []
    current_segment = []

    # 1. FOV Filtering (-100 deg to +100 deg) and Point Clustering
    for angle_deg in sorted_angles:
        if not (SCAN_MIN_ANGLE_DEG <= angle_deg <= SCAN_MAX_ANGLE_DEG):
            continue

        dist = scan_data[angle_deg]
        if dist > 0:
            rad = math.radians(angle_deg)
            x = round(float(dist * math.sin(rad)), 1)
            y = round(float(dist * math.cos(rad)), 1)
            pt = {"x": x, "y": y}

            if not current_segment:
                current_segment.append(pt)
            else:
                prev_pt = current_segment[-1]
                gap = math.hypot(pt["x"] - prev_pt["x"], pt["y"] - prev_pt["y"])
                if gap <= MAX_SEGMENT_GAP_MM:
                    current_segment.append(pt)
                else:
                    if len(current_segment) > 0:
                        segments.append(current_segment)
                    current_segment = [pt]

    if current_segment:
        segments.append(current_segment)

    # 2. Parametric Wall Model Extraction
    valid_walls, straightened_segments = build_valid_walls(segments)

    # 3. L-Corner Detection
    all_corners, corner_candidates = detect_l_shapes(valid_walls)
    is_corner_present = bool(len(all_corners) > 0)

    # 4. Angle Range Metrics
    front_pts = [scan_data[a] for a in range(-15, 16) if a in scan_data and scan_data[a] > 0]
    left_6_pts = [scan_data[a] for a in range(-93, -86) if a in scan_data and scan_data[a] > 0]
    right_6_pts = [scan_data[a] for a in range(87, 94) if a in scan_data and scan_data[a] > 0]

    avg_front = float(sum(front_pts)/len(front_pts)) if front_pts else 2000.0
    left_avg = float(sum(left_6_pts)/len(left_6_pts)) if left_6_pts else 0.0
    right_avg = float(sum(right_6_pts)/len(right_6_pts)) if right_6_pts else 0.0
    sum_avg = left_avg + right_avg

    # 5. Evaluate Custom Flag Conditions
    cond_front = (FLAG_FRONT_MIN_MM <= avg_front <= FLAG_FRONT_MAX_MM)
    cond_corner_dist = any(
        FLAG_CORNER_DIST_MIN_MM <= c["dist_to_robot_mm"] <= FLAG_CORNER_DIST_MAX_MM 
        for c in all_corners
    ) if all_corners else False
    cond_sum_lr = (sum_avg >= FLAG_SUM_LR_MIN_MM)

    corner_detected_flag = cond_front and cond_corner_dist and cond_sum_lr
    nearest_corner_dist_mm = (
        min(c["dist_to_robot_mm"] for c in all_corners) if all_corners else None
    )

    # Package processed spatial metadata
    metadata = {
        "is_corner": is_corner_present,
        "corner_distance_mm": nearest_corner_dist_mm,  
        "front": round(avg_front, 1),
        "left": round(left_avg, 1),
        "right": round(right_avg, 1),
        "left_avg_side": round(left_avg, 1),
        "right_avg_side": round(right_avg, 1),
        "sum_avg_side": round(sum_avg, 1),
        "segments": straightened_segments,
        "fitted_walls": [[w["start"], w["end"]] for w in valid_walls],
        "corners": all_corners,
        "corner_candidates": corner_candidates
    }
    # print(f"[DEBUG] Corner Detected: {corner_detected_flag}, Front: {int(avg_front)}mm, "
    #   f"Left : {int(left_avg)}mm, Right: {int(right_avg)}mm, LR Sum: {int(sum_avg)}mm, "
    #   f"Nearest Corner Dist: {int(nearest_corner_dist_mm) if nearest_corner_dist_mm is not None else 'N/A'}mm")

    return corner_detected_flag, metadata

# --- BACKGROUND WORKER THREAD ---

def lidar_worker():
    global latest_state
    try:
        lidar_scanner = LidarScanner(port=LIDAR_PORT, baudrate=LIDAR_BAUD)
        lidar_scanner.connect()
        print("[INFO] LiDAR thread connected.")
    except Exception as e:
        print(f"[FATAL] LiDAR connection failed: {e}")
        return

    frame_count = 0
    start_time = time.time()
    fps = 0.0

    try:
        while True:
            scan_data = lidar_scanner.get_scan_data()
            if not scan_data:
                time.sleep(0.01)
                continue

            # Calculate processing FPS over sliding window
            frame_count += 1
            elapsed = time.time() - start_time
            if elapsed >= 0.5:
                fps = round(frame_count / elapsed, 1)
                frame_count = 0
                start_time = time.time()

            # SINGLE FUNCTION CALL: Evaluate corner detection & get metadata
            corner_detected_flag, meta = evaluate_corner_detection(scan_data)

            with state_lock:
                latest_state["corner_detected"] = corner_detected_flag
                latest_state["is_corner"] = meta.get("is_corner", False)
                latest_state["fps"] = fps
                latest_state["front"] = meta.get("front", 2000.0)
                latest_state["left"] = meta.get("left", 2000.0)
                latest_state["right"] = meta.get("right", 2000.0)
                latest_state["left_avg_side"] = meta.get("left_avg_side", 0.0)
                latest_state["right_avg_side"] = meta.get("right_avg_side", 0.0)
                latest_state["sum_avg_side"] = meta.get("sum_avg_side", 0.0)
                latest_state["segments"] = meta.get("segments", [])
                latest_state["fitted_walls"] = meta.get("fitted_walls", [])
                latest_state["corners"] = meta.get("corners", [])
                latest_state["corner_candidates"] = meta.get("corner_candidates", [])

            time.sleep(0.01)
    finally:
        lidar_scanner.disconnect()

# --- FLASK WEB SERVER ---

@app.route('/data')
def get_data():
    with state_lock:
        return jsonify(latest_state)

@app.route('/')
def index():
    return render_template_string('''
    <!DOCTYPE html>
    <html>
    <head>
        <title>LiDAR L-Shape Corner Detection</title>
        <script src="https://cdn.jsdelivr.net/npm/chart.js"></script>
        <style>
            body { font-family: Arial, sans-serif; background: #121212; color: #fff; text-align: center; margin: 20px; }
            .status-box { font-size: 22px; font-weight: bold; padding: 12px; margin: 10px auto; width: 65%; border-radius: 8px; }
            .flag-box { font-size: 20px; font-weight: bold; padding: 10px; margin: 10px auto; width: 65%; border-radius: 8px; transition: all 0.2s ease; }
            .flag-active { background-color: #ff6d00; color: #ffffff; box-shadow: 0 0 12px #ff6d00; }
            .flag-inactive { background-color: #212121; color: #757575; border: 1px solid #424242; }
            .detected { background-color: #d32f2f; color: white; }
            .no-corner { background-color: #388e3c; color: white; }
            .metrics { font-size: 18px; margin-bottom: 20px; }
            .highlight { color: #00e5ff; font-weight: bold; }
            .fps-val { color: #76ff03; font-weight: bold; }
            #chart-container { width: 600px; height: 600px; margin: 0 auto; background: #1e1e1e; padding: 10px; border-radius: 8px; }
        </style>
    </head>
    <body>
        <h2>LiDAR L-Shape Corner Detection</h2>
        
        <div id="status" class="status-box no-corner">INITIALIZING...</div>
        <div id="flag_status" class="flag-box flag-inactive">FLAG: CORNER_DETECTED = FALSE</div>

        <div class="metrics">
            FPS: <span id="fps" class="fps-val">0.0</span> | 
            Front: <span id="front">0</span>mm | 
            Left (-87° to -93°): <span id="left_avg" class="highlight">0</span>mm | 
            Right (87° to 93°): <span id="right_avg" class="highlight">0</span>mm | 
            Sum (L+R): <span id="sum_avg" class="highlight">0</span>mm
        </div>

        <div id="chart-container">
            <canvas id="lidarChart"></canvas>
        </div>

        <script>
            let currentCorners = [];

            const lidarOverlayPlugin = {
                id: 'lidarOverlayPlugin',
                afterDraw(chart) {
                    const { ctx, scales: { x, y } } = chart;
                    const rx = x.getPixelForValue(0);
                    const ry = y.getPixelForValue(0);

                    ctx.save();

                    // 1. Draw Yellow Dashed Line & Distance Text from Robot (0,0) to Corner Vertex
                    if (currentCorners && currentCorners.length > 0) {
                        currentCorners.forEach(corner => {
                            const cx = x.getPixelForValue(corner.x);
                            const cy = y.getPixelForValue(corner.y);

                            ctx.beginPath();
                            ctx.moveTo(rx, ry);
                            ctx.lineTo(cx, cy);
                            ctx.strokeStyle = '#ffd600';
                            ctx.lineWidth = 2;
                            ctx.setLineDash([6, 6]);
                            ctx.stroke();

                            const midX = (rx + cx) / 2;
                            const midY = (ry + cy) / 2;

                            const distText = Math.round(corner.dist_to_robot_mm) + " mm";
                            ctx.font = 'bold 12px Arial';
                            const textWidth = ctx.measureText(distText).width;

                            ctx.fillStyle = 'rgba(0, 0, 0, 0.75)';
                            ctx.fillRect(midX - textWidth / 2 - 4, midY - 10, textWidth + 8, 18);

                            ctx.fillStyle = '#ffd600';
                            ctx.textAlign = 'center';
                            ctx.textBaseline = 'middle';
                            ctx.fillText(distText, midX, midY);
                        });
                    }

                    // 2. Draw Robot Arrow Marker
                    const arrowSize = 16;
                    ctx.setLineDash([]);
                    ctx.fillStyle = '#ff9100';
                    ctx.strokeStyle = '#ffffff';
                    ctx.lineWidth = 2;

                    ctx.beginPath();
                    ctx.moveTo(rx, ry - arrowSize);
                    ctx.lineTo(rx - arrowSize / 1.5, ry + arrowSize / 1.5);
                    ctx.lineTo(rx, ry + arrowSize / 3);
                    ctx.lineTo(rx + arrowSize / 1.5, ry + arrowSize / 1.5);
                    ctx.closePath();
                    ctx.fill();
                    ctx.stroke();

                    ctx.restore();
                }
            };

            const ctx = document.getElementById('lidarChart').getContext('2d');
            const lidarChart = new Chart(ctx, {
                type: 'scatter',
                data: { datasets: [] },
                options: {
                    responsive: true,
                    maintainAspectRatio: false,
                    animation: false,
                    plugins: { legend: { display: false } },
                    scales: {
                        x: { type: 'linear', position: 'bottom', min: -2000, max: 2000, grid: { color: '#333' } },
                        y: { min: -1000, max: 2500, grid: { color: '#333' } }
                    }
                },
                plugins: [lidarOverlayPlugin]
            });

            function updateData() {
                fetch('/data')
                    .then(response => response.json())
                    .then(data => {
                        const statusElem = document.getElementById('status');
                        const flagElem = document.getElementById('flag_status');
                        currentCorners = data.corners || [];

                        if (data.is_corner) {
                            statusElem.innerText = "L-SHAPE CORNER GEOMETRY DETECTED (" + data.corners.length + ")";
                            statusElem.className = "status-box detected";
                        } else {
                            const candidates = data.corner_candidates || [];
                            const nearestCorner = candidates.length > 0 ? Math.min(...candidates.map(c => c.distance_mm)) : null;

                            if (nearestCorner !== null) {
                                statusElem.innerText = "CANDIDATE: corner " + nearestCorner.toFixed(0) + "mm away (need <50mm)";
                            } else {
                                statusElem.innerText = "NO L-CORNER GEOMETRY";
                            }
                            statusElem.className = "status-box no-corner";
                        }

                        // Update Custom Flag Display
                        if (data.corner_detected) {
                            flagElem.innerText = "FLAG: CORNER_DETECTED = TRUE";
                            flagElem.className = "flag-box flag-active";
                        } else {
                            flagElem.innerText = "FLAG: CORNER_DETECTED = FALSE";
                            flagElem.className = "flag-box flag-inactive";
                        }

                        document.getElementById('fps').innerText = data.fps;
                        document.getElementById('front').innerText = data.front;
                        document.getElementById('left_avg').innerText = data.left_avg_side;
                        document.getElementById('right_avg').innerText = data.right_avg_side;
                        document.getElementById('sum_avg').innerText = data.sum_avg_side;

                        // Fitted Straight Wall Lines (Blue Solid Lines Overlaid On Points)
                        const fittedWallDatasets = (data.fitted_walls || []).map(line => ({
                            data: line,
                            showLine: true,
                            borderColor: '#29b6f6',
                            borderWidth: 4,
                            backgroundColor: '#29b6f6',
                            pointRadius: 0
                        }));

                        // Extracted Corner Markers (BIG RED DOTS)
                        const cornerDataset = {
                            label: 'Corners',
                            data: data.corners,
                            backgroundColor: '#ff1744',
                            borderColor: '#ffffff',
                            borderWidth: 3,
                            pointRadius: 10,
                            pointHoverRadius: 12
                        };

                        // Corner Boundary Lines
                        const boundaryDatasets = (data.corner_candidates || []).map(c => ({
                            data: c.boundary_line,
                            showLine: true,
                            borderColor: c.confirmed ? '#00e5ff' : 'rgba(0, 229, 255, 0.5)',
                            borderWidth: c.confirmed ? 6 : 3,
                            borderDash: c.confirmed ? [] : [10, 6],
                            backgroundColor: c.confirmed ? '#00e5ff' : 'rgba(0, 229, 255, 0.5)',
                            pointRadius: 0
                        }));

                        // Raw LiDAR Point Clusters (Light Green Points)
                        const segmentDatasets = data.segments.map(seg => ({
                            data: seg,
                            showLine: false,
                            backgroundColor: '#00e676',
                            pointRadius: 2
                        }));

                        lidarChart.data.datasets = [...fittedWallDatasets, cornerDataset, ...boundaryDatasets, ...segmentDatasets];
                        lidarChart.update();
                    });
            }

            setInterval(updateData, 100);
        </script>
    </body>
    </html>
    ''')

if __name__ == '__main__':
    threading.Thread(target=lidar_worker, daemon=True).start()
    app.run(host='0.0.0.0', port=5000, debug=False)