"""
Analyze formation trajectory data for inter-formation collisions.
Checks whether nodes from different formations come too close.
"""
import json
import math
import os
import glob

DATA_DIR = os.path.dirname(os.path.abspath(__file__))

# Collision threshold in meters — nodes closer than this are considered colliding
COLLISION_THRESHOLD_M = 5.0

def haversine(lat1, lon1, lat2, lon2):
    """Return distance in meters between two lat/lon points."""
    R = 6371000  # Earth radius in meters
    phi1 = math.radians(lat1)
    phi2 = math.radians(lat2)
    dphi = math.radians(lat2 - lat1)
    dlam = math.radians(lon2 - lon1)
    a = math.sin(dphi/2)**2 + math.cos(phi1)*math.cos(phi2)*math.sin(dlam/2)**2
    return R * 2 * math.atan2(math.sqrt(a), math.sqrt(1 - a))

def load_all_files():
    """Load all result JSON files, sorted by name."""
    pattern = os.path.join(DATA_DIR, "result_*.json")
    files = sorted(glob.glob(pattern))
    print(f"Found {len(files)} data files")
    return files

def check_collisions():
    files = load_all_files()

    # Accumulate data per formation: {formation_id: {frame_id: [nodes]}}
    # But to save memory, we process frame by frame across all files

    # Because files are sequential shards, we need to ensure frames align across formations
    # Each file contains all 3 formations for its frame range

    # Strategy: load one file at a time, check each frame within it
    # Since each file has all formations for its frames, we can check within each file

    all_collisions = []  # list of (frame_id, formation_a, node_a, formation_b, node_b, distance)

    # Track overall stats
    min_distances = []  # per-frame minimum inter-formation distances
    formation_pairs = [(1, 2), (1, 3), (2, 3)]  # string keys in data

    for fpath in files:
        print(f"Processing: {os.path.basename(fpath)}")
        with open(fpath, 'r') as f:
            data = json.load(f)

        formations = data["formations"]

        # Get all frame_ids from formation "1" (all formations should have same frames)
        frames_1 = {f["frame_id"]: f for f in formations["1"]}
        frames_2 = {f["frame_id"]: f for f in formations["2"]}
        frames_3 = {f["frame_id"]: f for f in formations["3"]}

        common_frames = sorted(set(frames_1.keys()) & set(frames_2.keys()) & set(frames_3.keys()))

        for frame_id in common_frames:
            frame_min_dist = float('inf')

            nodes_1 = frames_1[frame_id]["nodes"]
            nodes_2 = frames_2[frame_id]["nodes"]
            nodes_3 = frames_3[frame_id]["nodes"]

            formation_nodes = {"1": nodes_1, "2": nodes_2, "3": nodes_3}

            # Check each pair of formations
            for (fa_id, fb_id) in formation_pairs:
                fa_key = str(fa_id)
                fb_key = str(fb_id)
                for na in formation_nodes[fa_key]:
                    for nb in formation_nodes[fb_key]:
                        d = haversine(na["lat"], na["lon"], nb["lat"], nb["lon"])
                        if d < frame_min_dist:
                            frame_min_dist = d
                        if d < COLLISION_THRESHOLD_M:
                            all_collisions.append({
                                "frame_id": frame_id,
                                "formation_a": fa_id,
                                "node_a": na["node_id"],
                                "lat_a": na["lat"],
                                "lon_a": na["lon"],
                                "formation_b": fb_id,
                                "node_b": nb["node_id"],
                                "lat_b": nb["lat"],
                                "lon_b": nb["lon"],
                                "distance_m": round(d, 3),
                            })

            min_distances.append((frame_id, frame_min_dist))

    return all_collisions, min_distances

def main():
    collisions, min_distances = check_collisions()

    print("\n" + "=" * 60)
    print("COLLISION ANALYSIS RESULTS")
    print("=" * 60)

    if collisions:
        print(f"\n!!! FOUND {len(collisions)} COLLISION EVENTS (distance < {COLLISION_THRESHOLD_M}m)")
        print()

        # Group by formation pair
        from collections import Counter
        pair_counts = Counter((c["formation_a"], c["formation_b"]) for c in collisions)
        for (a, b), cnt in sorted(pair_counts.items()):
            print(f"  Formation {a} <-> Formation {b}: {cnt} collision events")

        print()
        print("First 20 collision events:")
        print("-" * 60)
        for c in collisions[:20]:
            print(f"  Frame {c['frame_id']:>5} | F{c['formation_a']}.N{c['node_a']} <-> F{c['formation_b']}.N{c['node_b']} | dist={c['distance_m']}m")

        # Worst collisions (smallest distances)
        worst = sorted(collisions, key=lambda x: x["distance_m"])[:10]
        print(f"\nTop 10 closest approaches (potential worst collisions):")
        print("-" * 60)
        for c in worst:
            print(f"  Frame {c['frame_id']:>5} | F{c['formation_a']}.N{c['node_a']} ({c['lat_a']:.6f},{c['lon_a']:.6f})")
            print(f"           | F{c['formation_b']}.N{c['node_b']} ({c['lat_b']:.6f},{c['lon_b']:.6f})")
            print(f"           | distance = {c['distance_m']}m")
            print()
    else:
        print(f"\n[OK] NO collisions found (threshold = {COLLISION_THRESHOLD_M}m)")

    # Overall statistics
    print("=" * 60)
    print("OVERALL STATISTICS")
    print("=" * 60)

    dists = [d for _, d in min_distances]
    avg_min = sum(dists) / len(dists)
    abs_min = min(dists)
    abs_min_frame = [f for f, d in min_distances if d == abs_min][0]
    abs_max = max(dists)

    # Also compute per-formation-pair statistics
    print(f"\nTotal frames analyzed: {len(dists)}")
    print(f"Average min inter-formation distance: {avg_min:.2f}m")
    print(f"Absolute minimum distance: {abs_min:.2f}m (Frame {abs_min_frame})")
    print(f"Maximum min distance: {abs_max:.2f}m")

    # Distance distribution
    bins = [(0, 1), (1, 3), (3, 5), (5, 10), (10, 20), (20, 50), (50, 100), (100, float('inf'))]
    print(f"\nDistance distribution (min inter-formation per frame):")
    for lo, hi in bins:
        count = sum(1 for _, d in min_distances if lo <= d < hi)
        pct = 100 * count / len(min_distances)
        label = f"{lo}-{hi}m" if hi != float('inf') else f">{lo}m"
        bar = "#" * int(pct / 2)
        print(f"  {label:>10}: {count:>5} frames ({pct:5.1f}%) {bar}")

if __name__ == "__main__":
    main()
