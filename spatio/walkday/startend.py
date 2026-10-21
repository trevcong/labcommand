"""Raw-scan start-to-end registration. INFORMATION ONLY: never scored.

Did the sensor physically come back to where it started? This answers from
the raw lidar alone, without FAST-LIO: the scans of the end hold are
registered (point-to-plane ICP, many seeds) against the scans of the start
hold, both in the sensor's own frame. The result sits beside FAST-LIO's own
start->end relative pose in walkday.md. Ported from the drift investigation
(spatio-data/derived/drift-investigation/start_end_register.py, geom.py).

    python3 -m spatio.walkday.startend BAG_DIR TRAJECTORY_TUM OUT_DIR --lidar TOPIC
        [--hold S] [--rmin M] [--rmax M]

Hold windows are the first and last `hold` seconds of the trajectory (the walk
checker's rule). Points come straight from the bag; nothing is deskewed, which
is only valid because the cart is parked in both windows. Livox CustomMsg and
PointCloud2 (float32 x/y/z) are supported; the topic is an argument.
Exit 0 written, 2 not computable (reason in the JSON), 3 bad input.
"""

from __future__ import annotations

import argparse
import glob
import os
import struct
import sys

import numpy as np
from scipy.spatial import cKDTree
from scipy.spatial.transform import Rotation

from spatio.manifest.jsonio import write_json_atomic

SCHEMA = "spatio.walkday.startend/1"
NOTE = ("information only; not part of PASS/FAIL. Raw end-hold scans registered to raw "
        "start-hold scans in the sensor frame; FAST-LIO not involved.")


# ---------------------------------------------------------------- decoding
def _cdr_custom_msg(data: bytes):
    """Livox CustomMsg CDR -> (stamp_s, xyz float64 (N,3)). Layout per livox_ros_driver2."""
    b = memoryview(data)[4:]                       # skip encapsulation header
    sec, nsec, flen = struct.unpack_from("<iII", b, 0)
    off = 12 + flen
    off += (-off) % 8
    off += 8                                       # timebase
    (point_num,) = struct.unpack_from("<I", b, off)
    off += 4 + 4                                   # point_num, lidar_id, rsvd[3]
    off += (-off) % 4
    (n,) = struct.unpack_from("<I", b, off)
    off += 4
    if n != point_num:
        raise ValueError(f"CustomMsg point_num {point_num} != sequence length {n}")
    if n == 0:
        return sec + nsec * 1e-9, np.zeros((0, 3))
    need = 20 * (n - 1) + 19
    raw = np.frombuffer(b[off:off + need].tobytes() + b"\x00", dtype=np.uint8).reshape(n, 20)
    xyz = raw[:, 4:16].copy().view("<f4").reshape(n, 3).astype(np.float64)
    return sec + nsec * 1e-9, xyz


def _pointcloud2(msg):
    offs = {f.name: (f.offset, f.datatype) for f in msg.fields}
    n = msg.width * msg.height
    view = np.frombuffer(bytes(msg.data), dtype=np.uint8)[: n * msg.point_step].reshape(n, msg.point_step)
    cols = []
    for name in ("x", "y", "z"):
        o, dt = offs[name]
        if dt != 7:
            raise ValueError(f"PointCloud2 field {name} datatype {dt}, expected FLOAT32")
        cols.append(view[:, o:o + 4].copy().view("<f4").reshape(-1))
    st = msg.header.stamp
    return st.sec + st.nanosec * 1e-9, np.stack(cols, axis=1).astype(np.float64)


def read_scans(bag_dir: str, topic: str, windows):
    """Scans whose header time falls inside any (t0, t1) window -> list of lists of arrays."""
    from mcap.reader import make_reader
    files = sorted(glob.glob(os.path.join(bag_dir, "*.mcap")))
    if not files:
        raise ValueError(f"no .mcap in {bag_dir}")
    out = [[] for _ in windows]
    lo = min(w[0] for w in windows) - 2.0
    hi = max(w[1] for w in windows) + 2.0
    for path in files:
        with open(path, "rb") as f:
            reader = make_reader(f)
            for schema, channel, message in reader.iter_messages(
                    topics=[topic], start_time=int(lo * 1e9), end_time=int(hi * 1e9)):
                if schema.name == "livox_ros_driver2/msg/CustomMsg":
                    t, xyz = _cdr_custom_msg(message.data)
                elif schema.name == "sensor_msgs/msg/PointCloud2":
                    from mcap_ros2.decoder import DecoderFactory
                    dec = DecoderFactory().decoder_for(channel.message_encoding, schema)
                    t, xyz = _pointcloud2(dec(message.data))
                else:
                    raise ValueError(f"{topic} is {schema.name}; only CustomMsg/PointCloud2 supported")
                for k, (t0, t1) in enumerate(windows):
                    if t0 <= t <= t1:
                        out[k].append(xyz)
    return out


# ---------------------------------------------------------------- geometry
def _range(p, rmin, rmax):
    r = np.linalg.norm(p, axis=1)
    return p[np.isfinite(r) & (r >= rmin) & (r <= rmax)]


def voxel(p, size):
    k = np.floor(p / size).astype(np.int64)
    _, first = np.unique(k, axis=0, return_index=True)
    return p[np.sort(first)]


def _T(R, t):
    T = np.eye(4)
    T[:3, :3] = R
    T[:3, 3] = t
    return T


def apply(T, p):
    # numpy 2.0 + macOS Accelerate emits spurious matmul FP warnings on finite data.
    with np.errstate(divide="ignore", over="ignore", invalid="ignore"):
        return p @ T[:3, :3].T + T[:3, 3]


def yaw_deg(T):
    return float(np.degrees(np.arctan2(T[1, 0], T[0, 0])))


def rot_deg(T):
    return float(np.degrees(Rotation.from_matrix(T[:3, :3]).magnitude()))


class Target:
    def __init__(self, p, vox=0.05, k=12, min_planarity=0.3):
        p = voxel(p, vox)
        tree = cKDTree(p)
        _, nn = tree.query(p, k=k)
        c = p[nn] - p[nn].mean(axis=1, keepdims=True)
        w, v = np.linalg.eigh(np.einsum("nki,nkj->nij", c, c) / k)
        planarity = (w[:, 1] - w[:, 0]) / np.maximum(w[:, 2], 1e-12)
        keep = planarity >= min_planarity
        self.p, self.n = p[keep], v[keep, :, 0]
        self.tree = cKDTree(self.p)


def icp(src, tgt, T0, dists=(1.0, 0.5, 0.25, 0.1), iters=15, huber=0.05):
    """Point-to-plane Gauss-Newton ICP; returns T (target <- source) and fit stats."""
    T = T0.copy()
    for dmax in dists:
        for _ in range(iters):
            q = apply(T, src)
            d, j = tgt.tree.query(q, distance_upper_bound=dmax)
            ok = np.isfinite(d)
            if ok.sum() < 50:
                break
            qq, pp, nn = q[ok], tgt.p[j[ok]], tgt.n[j[ok]]
            r = np.einsum("ij,ij->i", qq - pp, nn)
            J = np.hstack([np.cross(qq, nn), nn])
            w = np.where(np.abs(r) <= huber, 1.0, huber / np.maximum(np.abs(r), 1e-12))
            with np.errstate(divide="ignore", over="ignore", invalid="ignore"):
                H, g = J.T @ (J * w[:, None]), J.T @ (w * r)
            if not (np.all(np.isfinite(H)) and np.all(np.isfinite(g))):
                break
            dx = -np.linalg.solve(H + 1e-9 * np.eye(6), g)
            T = _T(Rotation.from_rotvec(dx[:3]).as_matrix(), dx[3:]) @ T
            if np.linalg.norm(dx) < 1e-6:
                break
    q = apply(T, src)
    d, j = tgt.tree.query(q, distance_upper_bound=0.5)
    ok = np.isfinite(d)
    r = np.abs(np.einsum("ij,ij->i", q[ok] - tgt.p[j[ok]], tgt.n[j[ok]]))
    return T, {
        "inlier_frac_10cm": float((r < 0.10).sum() / max(len(q), 1)),
        "inlier_frac_5cm": float((r < 0.05).sum() / max(len(q), 1)),
        "median_abs_p2plane_m": float(np.median(r)) if ok.any() else None,
        "n_src": int(len(q)),
    }


def register(start_pts, end_pts, T_fastlio=None, grid=True):
    """End-hold cloud expressed in the start-hold sensor frame, from many seeds."""
    tgt = Target(start_pts, vox=0.05)
    src = voxel(end_pts, 0.10)
    seeds = {"identity": np.eye(4)}
    if T_fastlio is not None:
        seeds["fastlio"] = T_fastlio
    if grid:
        for yaw in range(-180, 180, 15):
            for dx in (-2, -1, 0, 1, 2):
                for dy in (-2, -1, 0, 1, 2):
                    seeds[f"grid_y{yaw}_x{dx}_y{dy}"] = _T(
                        Rotation.from_euler("z", yaw, degrees=True).as_matrix(), [dx, dy, 0])
    sub = src[:: max(1, len(src) // 3000)]
    coarse = {k: icp(sub, tgt, T0, dists=(1.0, 0.5), iters=8) for k, T0 in seeds.items()}
    best = sorted(coarse, key=lambda k: -coarse[k][1]["inlier_frac_10cm"])[:5]
    results = {}
    for k in set(best) | ({"identity", "fastlio"} & set(seeds)):
        T, st = icp(src, tgt, coarse[k][0])
        results[k] = {"T": T.tolist(), "translation_m": float(np.linalg.norm(T[:3, 3])),
                      "xy_m": float(np.hypot(*T[:2, 3])), "yaw_deg": yaw_deg(T), "rot_deg": rot_deg(T), **st}
    winner = max(results, key=lambda k: (results[k]["inlier_frac_5cm"], -(results[k]["median_abs_p2plane_m"] or 9)))
    return winner, results


def _tum(path):
    rows = [l.split() for l in open(path) if l.strip() and not l.startswith("#")]
    a = np.array(rows, dtype=np.float64)
    return a[:, 0], a[:, 1:4], a[:, 4:8]


def run(bag_dir, tum, lidar, hold=5.0, rmin=1.0, rmax=40.0, grid=True):
    tt, pos, quat = _tum(tum)
    ws, we = (tt[0], tt[0] + hold), (tt[-1] - hold, tt[-1])
    out = {"schema": SCHEMA, "note": NOTE, "lidar_topic": lidar, "start_hold_s": list(ws),
           "end_hold_s": list(we), "range_filter_m": [rmin, rmax], "computed": False}
    if ws[1] >= we[0]:
        out["reason"] = "trajectory shorter than two hold windows"
        return out
    starts, ends = read_scans(bag_dir, lidar, [ws, we])
    out["n_scans_start"], out["n_scans_end"] = len(starts), len(ends)
    if not starts or not ends:
        out["reason"] = "no raw scans inside a hold window"
        return out
    sp = _range(np.vstack(starts), rmin, rmax)
    ep = _range(np.vstack(ends), rmin, rmax)
    if len(sp) < 1000 or len(ep) < 1000:
        out["reason"] = f"too few points in range (start {len(sp)}, end {len(ep)})"
        return out
    Ts = _T(Rotation.from_quat(quat[np.argmin(abs(tt - (ws[0] + ws[1]) / 2))]).as_matrix(),
            pos[np.argmin(abs(tt - (ws[0] + ws[1]) / 2))])
    Te = _T(Rotation.from_quat(quat[np.argmin(abs(tt - (we[0] + we[1]) / 2))]).as_matrix(),
            pos[np.argmin(abs(tt - (we[0] + we[1]) / 2))])
    T_fl = np.linalg.inv(Ts) @ Te
    winner, results = register(sp, ep, T_fl, grid=grid)
    Tw = np.array(results[winner]["T"])
    diff = np.linalg.inv(Tw) @ T_fl
    out.update({
        "computed": True,
        "registration": {k: results[winner][k] for k in
                         ("translation_m", "xy_m", "yaw_deg", "rot_deg", "inlier_frac_5cm",
                          "inlier_frac_10cm", "median_abs_p2plane_m")},
        "winner_seed": winner,
        "fastlio_relative": {"translation_m": float(np.linalg.norm(T_fl[:3, 3])),
                             "xy_m": float(np.hypot(*T_fl[:2, 3])), "yaw_deg": yaw_deg(T_fl),
                             "rot_deg": rot_deg(T_fl)},
        "fastlio_minus_registration": {"translation_m": float(np.linalg.norm(diff[:3, 3])),
                                       "rot_deg": rot_deg(diff)},
        "all_seeds": results,
    })
    return out


def main(argv=None):
    ap = argparse.ArgumentParser(prog="spatio.walkday.startend", description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("bag_dir")
    ap.add_argument("trajectory")
    ap.add_argument("out_dir")
    ap.add_argument("--lidar", required=True)
    ap.add_argument("--hold", type=float, default=5.0)
    ap.add_argument("--rmin", type=float, default=1.0)
    ap.add_argument("--rmax", type=float, default=40.0)
    a = ap.parse_args(argv)
    try:
        out = run(a.bag_dir, a.trajectory, a.lidar, a.hold, a.rmin, a.rmax)
    except (OSError, ValueError) as exc:
        print(f"startend: {exc}", file=sys.stderr)
        return 3
    os.makedirs(a.out_dir, exist_ok=True)
    write_json_atomic(os.path.join(a.out_dir, "startend.json"), out)
    if out["computed"]:
        r, fl = out["registration"], out["fastlio_relative"]
        print(f"raw-scan start->end (info only): {r['translation_m'] * 100:.1f} cm, yaw {r['yaw_deg']:+.2f} deg, "
              f"fit {r['inlier_frac_5cm']:.2f} within 5 cm; FAST-LIO {fl['translation_m'] * 100:.1f} cm, "
              f"yaw {fl['yaw_deg']:+.2f} deg")
        return 0
    print(f"raw-scan start->end not computed: {out['reason']}")
    return 2


if __name__ == "__main__":
    sys.exit(main())
