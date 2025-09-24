#!/usr/bin/env python3
import argparse
import numpy as np

EPS = 1e-12


def rotvec_to_matrix(r):
    r = np.asarray(r, dtype=float).reshape(3)
    theta = np.linalg.norm(r)
    if theta < EPS:
        return np.eye(3)
    k = r / theta
    K = np.array([[0, -k[2], k[1]],
                  [k[2], 0, -k[0]],
                  [-k[1], k[0], 0]], dtype=float)
    R = np.eye(3) + np.sin(theta) * K + (1.0 - np.cos(theta)) * (K @ K)
    return R


def rotvec_to_quat(r):
    """Return quaternion in (x, y, z, w) convention."""
    r = np.asarray(r, dtype=float).reshape(3)
    theta = np.linalg.norm(r)
    if theta < EPS:
        return np.array([0.0, 0.0, 0.0, 1.0])
    axis = r / theta
    half = theta * 0.5
    s = np.sin(half)
    x, y, z = axis * s
    w = np.cos(half)
    return np.array([x, y, z, w])


def canon_rotvec(r):
    """Map axis-angle to the shortest representation (angle ∈ [0, π])."""
    r = np.asarray(r, dtype=float).reshape(3)
    theta = np.linalg.norm(r)
    if theta < EPS:
        return r
    if theta > np.pi + 1e-9:
        r = r * (1.0 - 2.0 * np.pi / theta)
    # Near π, either branch is valid; leave as-is to avoid jitter
    return r


def quat_from_mats(R):
    """Helper: quaternion from rotation matrix (x, y, z, w)."""
    R = np.asarray(R, dtype=float).reshape(3, 3)
    tr = np.trace(R)
    if tr > 0:
        S = np.sqrt(tr + 1.0) * 2.0
        w = 0.25 * S
        x = (R[2, 1] - R[1, 2]) / S
        y = (R[0, 2] - R[2, 0]) / S
        z = (R[1, 0] - R[0, 1]) / S
    else:
        # Find the largest diagonal element
        if (R[0, 0] > R[1, 1]) and (R[0, 0] > R[2, 2]):
            S = np.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2]) * 2.0
            x = 0.25 * S
            y = (R[0, 1] + R[1, 0]) / S
            z = (R[0, 2] + R[2, 0]) / S
            w = (R[2, 1] - R[1, 2]) / S
        elif R[1, 1] > R[2, 2]:
            S = np.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2]) * 2.0
            x = (R[0, 1] + R[1, 0]) / S
            y = 0.25 * S
            z = (R[1, 2] + R[2, 1]) / S
            w = (R[0, 2] - R[2, 0]) / S
        else:
            S = np.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1]) * 2.0
            x = (R[0, 2] + R[2, 0]) / S
            y = (R[1, 2] + R[2, 1]) / S
            z = 0.25 * S
            w = (R[1, 0] - R[0, 1]) / S
    return np.array([x, y, z, w])


def rotation_angle_between(r1, r2, degrees=True):
    """Angle of R1^T R2 using rotvecs."""
    R1 = rotvec_to_matrix(r1)
    R2 = rotvec_to_matrix(r2)
    R = R1.T @ R2
    # Clamp numerical errors
    c = (np.trace(R) - 1.0) * 0.5
    c = max(min(c, 1.0), -1.0)
    ang = np.arccos(c)
    return np.degrees(ang) if degrees else ang


def main():
    ap = argparse.ArgumentParser(description="UR rotvec → rotation matrix / quaternion utilities")
    sub = ap.add_subparsers(dest="cmd", required=True)

    ap_r = sub.add_parser("rotvec", help="Convert a rotation vector to matrix & quaternion")
    ap_r.add_argument("rx", type=float)
    ap_r.add_argument("ry", type=float)
    ap_r.add_argument("rz", type=float)
    ap_r.add_argument("--canon", action="store_true", help="Canonicalize rotvec to angle ≤ π")

    ap_c = sub.add_parser("compare", help="Compare two rotation vectors (angular difference)")
    ap_c.add_argument("rx1", type=float)
    ap_c.add_argument("ry1", type=float)
    ap_c.add_argument("rz1", type=float)
    ap_c.add_argument("rx2", type=float)
    ap_c.add_argument("ry2", type=float)
    ap_c.add_argument("rz2", type=float)

    args = ap.parse_args()

    if args.cmd == "rotvec":
        r = np.array([args.rx, args.ry, args.rz], dtype=float)
        if args.canon:
            r = canon_rotvec(r)
        R = rotvec_to_matrix(r)
        q = rotvec_to_quat(r)
        np.set_printoptions(precision=6, suppress=True)
        print("Rotvec:", r)
        print("Quaternion (x, y, z, w):", q)
        print("Rotation matrix:\n", R)

    elif args.cmd == "compare":
        r1 = np.array([args.rx1, args.ry1, args.rz1], dtype=float)
        r2 = np.array([args.rx2, args.ry2, args.rz2], dtype=float)
        ang_deg = rotation_angle_between(r1, r2, degrees=True)
        print(f"Angular difference: {ang_deg:.6f} degrees")


if __name__ == "__main__":
    main()
