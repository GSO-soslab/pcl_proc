#!/usr/bin/env python3
"""
Offline Voxel Map Builder

Reads a rosbag2 bag and replicates the fls_ism + voxel_log_odds_visualizer
pipeline in pure Python/NumPy — no ROS2 timing constraints.

Usage:
    source /home/tony/auv_ws/install/setup.bash
    python3 src/iceberg_nav/scripts/offline_voxel_mapper.py

Edit config/offline_mapper.yaml before running (bag_path, voxel_resolution, etc.).
"""

import os
import math
import time
import bisect
import glob as glob_mod
import xml.etree.ElementTree as ET

import yaml
import numpy as np
from scipy.spatial.transform import Rotation

import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message

# ── Load config ───────────────────────────────────────────────────────────────

_SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
_YAML_PATH  = os.path.join(_SCRIPT_DIR, '..', 'config', 'offline_mapper.yaml')

with open(_YAML_PATH) as _f:
    _cfg = yaml.safe_load(_f)['offline_mapper']

BAG_PATH          = _cfg['bag_path']
OUTPUT_PCD        = _cfg['output_pcd']
SAVE_NPZ          = bool(_cfg.get('save_npz', False))

SIM               = bool(_cfg.get('sim', False))
IMAGE_TOPIC_FILTER = str(_cfg.get('image_topic_filter', 'raw_image'))
FLS_CHILD_LINK    = str(_cfg.get('fls_child_link', 'fls_link'))

VOXEL_RES      = float(_cfg['voxel_resolution'])
LOGODDS_MIN    = float(_cfg['logodds_min'])
LOGODDS_MAX    = float(_cfg['logodds_max'])
PROB_THRESHOLD = float(_cfg['probability_threshold'])

RANGE_MAX      = float(_cfg['range_max'])
H_FOV_DEG      = float(_cfg['horizontal_fov_deg'])
V_FOV_DEG      = float(_cfg['vertical_fov_deg'])
SPACING_DEG    = float(_cfg['spacing_angle_deg'])

MIN_RANGE      = float(_cfg['threshold_min_range'])
LOWER_BOUND    = int(_cfg['lower_bound_intensity'])
UPPER_BOUND    = int(_cfg['upper_bound_intensity'])
MIN_PROB       = float(_cfg['min_probability'])
MAX_PROB       = float(_cfg['max_probability'])
APERTURE_SIZE  = float(_cfg['aperture_size'])
FREQUENCY      = float(_cfg['frequency'])
SOUND_SPEED    = float(_cfg['sound_speed'])


# ── Transform utilities ───────────────────────────────────────────────────────

def rpy_to_matrix(xyz, rpy):
    """4x4 homogeneous transform from xyz translation and rpy (radians)."""
    T = np.eye(4, dtype=np.float64)
    T[:3, :3] = Rotation.from_euler('xyz', rpy).as_matrix()
    T[:3,  3] = xyz
    return T


def quat_to_matrix(xyz, q_xyzw):
    """4x4 homogeneous transform from xyz translation and quaternion [x,y,z,w]."""
    T = np.eye(4, dtype=np.float64)
    T[:3, :3] = Rotation.from_quat(q_xyzw).as_matrix()
    T[:3,  3] = xyz
    return T


def transform_stamped_to_matrix(ts):
    """Convert geometry_msgs/TransformStamped to 4x4 numpy matrix."""
    t = ts.transform.translation
    q = ts.transform.rotation
    return quat_to_matrix([t.x, t.y, t.z], [q.x, q.y, q.z, q.w])


# ── URDF parser ───────────────────────────────────────────────────────────────

def parse_urdf_fixed_joints(urdf_path, namespace='alpha_rise'):
    """
    Parse URDF and return {(parent_frame, child_frame): 4x4_matrix}
    for every fixed joint, with namespace prepended to each link name.
    """
    root = ET.parse(urdf_path).getroot()
    result = {}
    for joint in root.findall('joint'):
        if joint.get('type') != 'fixed':
            continue
        parent_el = joint.find('parent')
        child_el  = joint.find('child')
        origin_el = joint.find('origin')
        if parent_el is None or child_el is None:
            continue
        parent = f"{namespace}/{parent_el.get('link')}"
        child  = f"{namespace}/{child_el.get('link')}"
        xyz = [0.0, 0.0, 0.0]
        rpy = [0.0, 0.0, 0.0]
        if origin_el is not None:
            xyz = [float(v) for v in origin_el.get('xyz', '0 0 0').split()]
            rpy = [float(v) for v in origin_el.get('rpy', '0 0 0').split()]
        result[(parent, child)] = rpy_to_matrix(xyz, rpy)
    return result


# ── Image preprocessing (replicated from fls_pcl.py) ─────────────────────────

def _anisotropic_diffusion(img, niter=5, kappa=30, gamma=0.1):
    src = img.astype(np.float32)
    for _ in range(niter):
        dN = np.roll(src,  1, axis=0) - src
        dS = np.roll(src, -1, axis=0) - src
        dE = np.roll(src, -1, axis=1) - src
        dW = np.roll(src,  1, axis=1) - src
        src += gamma * (
            np.exp(-(dN / kappa) ** 2) * dN +
            np.exp(-(dS / kappa) ** 2) * dS +
            np.exp(-(dE / kappa) ** 2) * dE +
            np.exp(-(dW / kappa) ** 2) * dW
        )
    return np.clip(src, 0, 255).astype(img.dtype)


def image_preprocess(img):
    """Wedge mask + anisotropic diffusion (replicates fls_pcl.py real-hardware branch)."""
    current = img.copy()
    h, w    = current.shape
    mid     = w // 2

    # Tapered wedge mask (removes near-range centre-column artifacts)
    top_width, bottom_width, bottom_offset = 20, 15, 20
    frac        = np.arange(h, dtype=np.float32) / (h - 1)
    widths      = np.round(top_width + frac * (bottom_width - top_width)).astype(int)
    left_bounds = np.round(
        (mid - top_width) + frac * ((mid - bottom_offset) - (mid - top_width))
    ).astype(int)
    x    = np.arange(w)
    mask = (x[None, :] >= left_bounds[:, None]) & \
           (x[None, :] <  (left_bounds + widths)[:, None])
    current[mask] = 0

    return _anisotropic_diffusion(current, niter=5, kappa=30, gamma=0.1)


def image_preprocess_sim(img):
    """Sim branch (replicates fls_pcl.py sim branch): return image as-is."""
    return img.copy()


# ── PCD writer (matching voxel_log_odds_visualizer.cpp savePCD) ───────────────

def save_pcd(logodds_grid, filename, voxel_res, prob_threshold):
    """Write occupied voxels to ASCII PCD with rgb + occupancy fields."""

    def to_prob(lo):
        return 1.0 / (1.0 + math.exp(-lo))

    valid = [(key, lo) for key, lo in logodds_grid.items()
             if to_prob(lo) >= prob_threshold]
    n = len(valid)

    print(f"\nSaving {n} voxels (prob >= {prob_threshold}) → {filename}")
    os.makedirs(os.path.dirname(os.path.abspath(filename)), exist_ok=True)

    with open(filename, 'w') as f:
        f.write("# .PCD v.7 - Point Cloud Data file format\n")
        f.write("VERSION .7\n")
        f.write("FIELDS x y z rgb occupancy\n")
        f.write("SIZE 4 4 4 4 4\n")
        f.write("TYPE F F F U F\n")
        f.write("COUNT 1 1 1 1 1\n")
        f.write(f"WIDTH {n}\n")
        f.write("HEIGHT 1\n")
        f.write("VIEWPOINT 0 0 0 1 0 0 0\n")
        f.write(f"POINTS {n}\n")
        f.write("DATA ascii\n")

        for (ix, iy, iz), lo in valid:
            prob = to_prob(lo)
            # Voxel centre (floor-based indexing: centre = (idx + 0.5) * res)
            cx = (ix + 0.5) * voxel_res
            cy = (iy + 0.5) * voxel_res
            cz = (iz + 0.5) * voxel_res
            r   = int(prob * 255)
            b   = int((1.0 - prob) * 255)
            rgb = (r << 16) | b          # green = 0
            f.write(f"{cx:.6f} {cy:.6f} {cz:.6f} {rgb} {prob:.6f}\n")

    print(f"Done — {n} voxels written.")


# ── Main ─────────────────────────────────────────────────────────────────────

def main():
    print("=== Offline Voxel Mapper ===")
    print(f"Bag:        {BAG_PATH}")
    print(f"Output:     {OUTPUT_PCD}")
    print(f"Resolution: {VOXEL_RES} m")
    print(f"Sim mode:   {SIM}\n")

    # ── Static transforms from URDF ───────────────────────────────────────────
    from ament_index_python.packages import get_package_share_directory
    urdf_path = os.path.join(
        get_package_share_directory('alpha_rise_description'),
        'urdf', 'base.urdf'
    )
    fixed_joints = parse_urdf_fixed_joints(urdf_path, namespace='alpha_rise')
    print(f"Loaded {len(fixed_joints)} fixed joints from URDF:")
    for (p, c) in fixed_joints:
        print(f"  {p} → {c}")

    T_base_fls = fixed_joints.get(
        ('alpha_rise/base_link', f'alpha_rise/{FLS_CHILD_LINK}')
    )
    if T_base_fls is None:
        print(f"WARNING: {FLS_CHILD_LINK} joint not found in URDF — using hardcoded fallback")
        T_base_fls = rpy_to_matrix([-0.26, -0.13, 0.0], [1.57, 0.0, -1.57])

    # ── Beam pattern: elevation angles + sinc DI weights ─────────────────────
    wavelength = SOUND_SPEED / FREQUENCY
    k          = 2.0 * math.pi / wavelength

    elevation_angles = np.arange(
        -V_FOV_DEG / 2.0,
        V_FOV_DEG / 2.0 + SPACING_DEG,
        SPACING_DEG,
        dtype=np.float32,
    )
    angles_rad = np.deg2rad(elevation_angles)
    cos_a = np.cos(angles_rad)   # (B,)
    sin_a = np.sin(angles_rad)   # (B,)

    temp      = (k * APERTURE_SIZE / 2.0) * np.sin(angles_rad)
    DI        = np.where(np.abs(temp) > 1e-10,
                         np.sin(temp) / temp,
                         np.ones_like(temp))
    beam_probs = np.round(DI.astype(np.float32), 2)   # (B,)

    B = len(elevation_angles)
    print(f"\nElevation: {elevation_angles[0]:.1f}° to {elevation_angles[-1]:.1f}°"
          f"  ({B} angles)")

    # ── Open bag ──────────────────────────────────────────────────────────────
    mcap_files = glob_mod.glob(os.path.join(BAG_PATH, '*.mcap'))
    storage_id = 'mcap' if mcap_files else 'sqlite3'
    print(f"Storage:    {storage_id}")

    storage_opts   = rosbag2_py.StorageOptions(uri=BAG_PATH, storage_id=storage_id)
    converter_opts = rosbag2_py.ConverterOptions('', '')

    reader = rosbag2_py.SequentialReader()
    reader.open(storage_opts, converter_opts)

    type_map = {t.name: t.type for t in reader.get_all_topics_and_types()}
    print(f"\nTopics ({len(type_map)}):")
    for name in sorted(type_map):
        print(f"  {name}: {type_map[name]}")

    # Preload message types (avoids repeated imports in the hot loop)
    TFMessage = get_message('tf2_msgs/msg/TFMessage')
    PingMsg   = get_message('oculus_interfaces/msg/Ping')
    ImageMsg  = get_message('sensor_msgs/msg/Image')

    # ── Processing state ──────────────────────────────────────────────────────
    # Sim mode: bearings are derived from the first image width; max_range set from config.
    # Real mode: both are read from the first Ping message in the bag.
    bearings     = None                          # (n_beams,) float64 radians
    max_range_m  = RANGE_MAX if SIM else None    # float

    # Dynamic TF: odom → base_link, sorted chronologically by bag read order
    tf_times_ns  = []        # list[int]
    tf_matrices  = []        # list[np.ndarray 4x4]

    logodds_grid = {}        # {(ix,iy,iz): float}
    frame_count  = 0
    tf_skip      = 0

    # Sensor-frame geometry cache (rebuilt if image size or bearings change)
    _geom_cache  = None      # (cache_key, sx_all, sy_all)

    t_start = time.time()
    print("\n--- Processing bag ---")

    # ── Single-pass message loop ──────────────────────────────────────────────
    while reader.has_next():
        topic, data, _ts = reader.read_next()

        # ── /tf — dynamic odom → base_link ───────────────────────────────────
        if topic == '/tf':
            msg = deserialize_message(data, TFMessage)
            for ts in msg.transforms:
                if 'odom' in ts.header.frame_id and 'base_link' in ts.child_frame_id:
                    t_ns = ts.header.stamp.sec * 10**9 + ts.header.stamp.nanosec
                    tf_times_ns.append(t_ns)
                    tf_matrices.append(transform_stamped_to_matrix(ts))
            continue

        if topic == '/tf_static':
            # Static transforms already handled from URDF; ignore bag copies
            continue

        # ── Ping — bearings + max range (real hardware only) ─────────────────
        if not SIM and 'ping' in topic and bearings is None:
            msg         = deserialize_message(data, PingMsg)
            bearings    = np.radians(np.array(msg.bearings, dtype=np.float64) * 0.01)
            max_range_m = float(msg.range)
            print(f"Ping: {len(bearings)} beams, max_range={max_range_m:.1f} m")
            continue

        # ── Image — main processing ───────────────────────────────────────────
        if IMAGE_TOPIC_FILTER not in topic:
            continue
        if not SIM and bearings is None:
            continue   # real hardware: wait for first ping

        msg    = deserialize_message(data, ImageMsg)
        n_bins = msg.height       # range bins (rows)
        n_beams_img = msg.width   # azimuth beams (cols)

        # Sim: derive bearings from image width on first image (mirrors fls_pcl.py sim branch)
        if SIM and bearings is None:
            raw      = np.linspace(-3500, 3500, n_beams_img)
            bearings = np.radians(np.array(raw, dtype=np.float64) * 0.01)
            print(f"Sim: derived {len(bearings)} bearings from image width, "
                  f"max_range={max_range_m:.1f} m")

        img = np.frombuffer(bytes(msg.data), dtype=np.uint8).reshape(n_bins, n_beams_img)
        if SIM:
            img = image_preprocess_sim(img)
        else:
            img = image_preprocess(img)

        # ── Pixel → sensor-frame (x, y): cache for fixed image geometry ───────
        cache_key = (n_bins, n_beams_img, len(bearings))
        if _geom_cache is None or _geom_cache[0] != cache_key:
            rows_idx, cols_idx = np.indices((n_bins, n_beams_img))
            rows_f = rows_idx.ravel()
            cols_f = cols_idx.ravel()
            theta       = bearings[cols_f]
            m_per_bin   = max_range_m / n_bins
            r_values    = m_per_bin * (n_bins - rows_f)    # row 0 = max range
            sx_all      = (r_values * np.cos(theta)).astype(np.float32)
            sy_all      = (r_values * np.sin(theta)).astype(np.float32)
            _geom_cache = (cache_key, sx_all, sy_all)

        _, sx_all, sy_all = _geom_cache
        intens_all = img.ravel().astype(np.float32)

        # Keep only non-zero intensity pixels above minimum range
        ranges = np.hypot(sx_all, sy_all)
        valid  = (intens_all > 0) & (ranges >= MIN_RANGE)
        sx     = sx_all[valid]
        sy     = sy_all[valid]
        intens = intens_all[valid]

        if len(sx) == 0:
            frame_count += 1
            continue

        # ── Intensity → probability ───────────────────────────────────────────
        probs              = np.full(len(intens), MIN_PROB, dtype=np.float32)
        mid_mask           = (intens > LOWER_BOUND) & (intens < UPPER_BOUND)
        probs[mid_mask]    = MIN_PROB + (
            (intens[mid_mask] - LOWER_BOUND) / (UPPER_BOUND - LOWER_BOUND)
        ) * (MAX_PROB - MIN_PROB)
        probs[intens >= UPPER_BOUND] = MAX_PROB
        probs = np.round(probs * 10.0) / 10.0   # snap to nearest 0.1

        # ── Elevation expansion: rotate (sx, sy, 0) around Y axis ─────────────
        #   x_r = sx * cos(θ)
        #   y_r = sy
        #   z_r = -sx * sin(θ)
        x_r = sx[None, :] * cos_a[:, None]           # (B, N)
        y_r = sy[None, :] * np.ones((B, 1),           # (B, N)
                                     dtype=np.float32)
        z_r = -sx[None, :] * sin_a[:, None]          # (B, N)

        all_pts   = np.stack((x_r, y_r, z_r), axis=-1).reshape(-1, 3)  # (B*N, 3)
        all_probs = (probs[None, :] * beam_probs[:, None]).reshape(-1)  # (B*N,)

        # Range filter after elevation expansion
        rng2  = np.hypot(all_pts[:, 0], all_pts[:, 1])
        rmask = rng2 >= MIN_RANGE
        all_pts   = all_pts[rmask]
        all_probs = all_probs[rmask]

        if len(all_pts) == 0:
            frame_count += 1
            continue

        # ── TF lookup: closest odom→base_link by timestamp ───────────────────
        img_ns = msg.header.stamp.sec * 10**9 + msg.header.stamp.nanosec

        if not tf_times_ns:
            tf_skip += 1
            frame_count += 1
            continue

        idx = bisect.bisect_left(tf_times_ns, img_ns)
        if idx >= len(tf_times_ns):
            idx = len(tf_times_ns) - 1
        elif idx > 0:
            if (img_ns - tf_times_ns[idx - 1]) < (tf_times_ns[idx] - img_ns):
                idx -= 1

        T_odom_base = tf_matrices[idx]
        T_odom_fls  = T_odom_base @ T_base_fls   # odom ← fls_link

        # ── Transform points: fls_link → odom ────────────────────────────────
        pts_h    = np.hstack((all_pts.astype(np.float64),
                              np.ones((len(all_pts), 1))))    # (M, 4)
        pts_odom = (T_odom_fls @ pts_h.T).T[:, :3]           # (M, 3)

        # ── Depth filter — keep points below -1.0 m in world frame ───────────
        dmask     = pts_odom[:, 2] <= -1.0
        pts_odom  = pts_odom[dmask]
        all_probs = all_probs[dmask]

        if len(pts_odom) == 0:
            frame_count += 1
            continue

        # ── Bayesian log-odds update ──────────────────────────────────────────
        # evidence = log(p / (1-p))   [same as C++: logodds_measurement - logodds(0.5)]
        all_probs = np.clip(all_probs, 1e-6, 1.0 - 1e-6)
        evidence  = np.log(all_probs / (1.0 - all_probs)).astype(np.float64)

        # Voxel indices (floor-based, unbounded dict)
        ix_arr = np.floor(pts_odom[:, 0] / VOXEL_RES).astype(np.int32)
        iy_arr = np.floor(pts_odom[:, 1] / VOXEL_RES).astype(np.int32)
        iz_arr = np.floor(pts_odom[:, 2] / VOXEL_RES).astype(np.int32)

        # Aggregate evidence for duplicate voxel hits within this frame
        keys_arr           = np.stack([ix_arr, iy_arr, iz_arr], axis=1)  # (M, 3)
        u_keys, inverse    = np.unique(keys_arr, axis=0, return_inverse=True)
        agg_ev             = np.bincount(inverse, weights=evidence,
                                         minlength=len(u_keys))

        # Update dict with clamping
        for j in range(len(u_keys)):
            key     = (int(u_keys[j, 0]), int(u_keys[j, 1]), int(u_keys[j, 2]))
            new_val = logodds_grid.get(key, 0.0) + float(agg_ev[j])
            logodds_grid[key] = max(LOGODDS_MIN, min(LOGODDS_MAX, new_val))

        frame_count += 1
        if frame_count % 100 == 0:
            elapsed = time.time() - t_start
            print(f"  Frame {frame_count:5d} | voxels={len(logodds_grid):8d} | "
                  f"t={elapsed:6.1f}s | tf_skip={tf_skip}")

    elapsed = time.time() - t_start
    print(f"\nFinished: {frame_count} frames in {elapsed:.1f}s")
    print(f"TF skipped:  {tf_skip}")
    print(f"Total voxels: {len(logodds_grid)}")

    # ── Save outputs ──────────────────────────────────────────────────────────
    save_pcd(logodds_grid, OUTPUT_PCD, VOXEL_RES, PROB_THRESHOLD)

    if SAVE_NPZ:
        npz_path = OUTPUT_PCD.replace('.pcd', '.npz')
        k_arr    = np.array(list(logodds_grid.keys()),   dtype=np.int32)
        v_arr    = np.array(list(logodds_grid.values()), dtype=np.float32)
        np.savez_compressed(npz_path, keys=k_arr, vals=v_arr,
                            voxel_res=np.float32(VOXEL_RES))
        print(f"NPZ saved: {npz_path}")


if __name__ == '__main__':
    main()
