"""
Minimal Working Example: Iterative moveL Visual Servoing

This standalone script demonstrates the core algorithm for iterative moveL-based
visual servoing to converge on a target pose relative to an AprilTag.

Strategy:
1. First moveL: Go directly to target (gain=1.0, no smoothing)
2. Subsequent moveL: Gradual convergence (gain=0.2, momentum smoothing)
"""

import numpy as np


def compute_pose_error(current_pose, target_pose):
    """Compute 6-DOF error vector between current and target poses."""
    return np.array(target_pose) - np.array(current_pose)


def compute_error_magnitude(error):
    """Compute scalar error magnitude from 6-DOF error vector."""
    pos_error = np.linalg.norm(error[:3])
    rot_error = np.linalg.norm(error[3:])
    return np.sqrt(pos_error**2 + rot_error**2)


def add_noise(correction, noise_percent=0.0005):
    """Add uniformly distributed noise (default 0.05%) to correction."""
    noise = np.random.uniform(-noise_percent, noise_percent, 6) * np.abs(correction)
    return correction + noise


def iterative_movel_servo(
    current_pose,
    target_pose,
    first_gain=1.0,
    subsequent_gain=0.2,
    momentum=0.5,
    noise_percent=0.0005,
    max_iterations=50,
    tolerance=0.001,
):
    """
    Iterative moveL visual servoing algorithm.

    Args:
        current_pose: Initial 6-DOF pose [x, y, z, rx, ry, rz]
        target_pose: Target 6-DOF pose [x, y, z, rx, ry, rz]
        first_gain: Gain for first move (1.0 = go directly to target)
        subsequent_gain: Gain for subsequent moves (0.2 = gradual convergence)
        momentum: Momentum smoothing for subsequent moves (0.5 = 50% smoothing)
        noise_percent: Noise level (0.0005 = 0.05%)
        max_iterations: Maximum iterations
        tolerance: Convergence tolerance in meters

    Returns:
        List of poses representing the trajectory
    """
    pose = np.array(current_pose, dtype=float)
    target = np.array(target_pose, dtype=float)
    velocity = np.zeros(6)
    trajectory = [pose.copy()]

    for i in range(max_iterations):
        # Compute error
        error = compute_pose_error(pose, target)
        error_mag = compute_error_magnitude(error)

        # Check convergence
        if error_mag < tolerance:
            print(f"Converged at iteration {i} with error {error_mag:.6f}m")
            break

        # Determine gain and momentum based on iteration
        if i == 0:
            gain = first_gain
            current_momentum = 1.0  # No smoothing for first move
        else:
            gain = subsequent_gain
            current_momentum = momentum

        # Compute correction
        correction = gain * error

        # Add realistic noise
        correction = add_noise(correction, noise_percent)

        # Apply momentum smoothing
        velocity = current_momentum * correction + (1 - current_momentum) * velocity

        # Execute moveL (update pose)
        pose = pose + velocity

        trajectory.append(pose.copy())
        print(f"Iteration {i:2d}: Error = {error_mag:.6f}m, Correction = {np.linalg.norm(correction):.6f}m")

    return trajectory


def main():
    """Run minimal example."""
    # Example poses (x, y, z, rx, ry, rz) in meters and radians
    current_pose = [0.3, 0.2, 0.5, 0.1, -0.2, 0.15]
    target_pose = [0.4, 0.3, 0.55, 0.0, 0.0, 0.0]

    print("=" * 60)
    print("Iterative moveL Visual Servoing - Minimal Example")
    print("=" * 60)
    print(f"Start pose:  {current_pose}")
    print(f"Target pose: {target_pose}")
    print(f"Initial error: {compute_error_magnitude(compute_pose_error(current_pose, target_pose)):.6f}m")
    print("-" * 60)

    trajectory = iterative_movel_servo(
        current_pose=current_pose,
        target_pose=target_pose,
        first_gain=1.0,       # First move goes directly to target
        subsequent_gain=0.2,  # Subsequent moves: gradual convergence
        momentum=0.5,         # 50% momentum smoothing after first move
        noise_percent=0.0005, # 0.05% noise
        max_iterations=30,
        tolerance=0.001,
    )

    print("-" * 60)
    final_error = compute_error_magnitude(compute_pose_error(trajectory[-1], target_pose))
    print(f"Final pose:  {trajectory[-1].tolist()}")
    print(f"Final error: {final_error:.6f}m")
    print(f"Total iterations: {len(trajectory) - 1}")


if __name__ == "__main__":
    main()
