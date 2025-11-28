"""
Minimal Working Example: Iterative moveL Visual Servoing with Gradient Approximation

This standalone script demonstrates the core algorithm for iterative moveL-based
visual servoing to converge on a target pose relative to an AprilTag.

Strategy:
1. First moveL: Go directly to target (gain=1.0, no smoothing) to get in ballpark
2. Subsequent moveL: Use gradient approximation by jogging in ±xyz and ±roll-pitch-yaw
   directions to find optimal correction direction, then apply gradual convergence

The gradient approximation perturbs each of the 6 DOF (x, y, z, roll, pitch, yaw)
in both positive and negative directions to estimate which direction reduces error.
"""

import numpy as np
from scipy.spatial.transform import Rotation


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


def estimate_gradient(current_pose, target_pose, perturbation_size=0.001):
    """
    Estimate gradient by jogging in each of the 6 DOF directions.
    
    Perturbs in ±x, ±y, ±z (position) and ±roll, ±pitch, ±yaw (orientation)
    to approximate the gradient of the error function.
    
    Args:
        current_pose: Current 6-DOF pose [x, y, z, rx, ry, rz]
        target_pose: Target 6-DOF pose [x, y, z, rx, ry, rz]
        perturbation_size: Size of perturbation for gradient estimation
        
    Returns:
        gradient: 6-DOF gradient vector pointing toward error reduction
    """
    current = np.array(current_pose, dtype=float)
    target = np.array(target_pose, dtype=float)
    gradient = np.zeros(6)
    
    # Current error magnitude
    current_error = compute_error_magnitude(compute_pose_error(current, target))
    
    # Perturb each dimension and measure error change
    for dim in range(6):
        # Positive perturbation
        perturbed_pos = current.copy()
        perturbed_pos[dim] += perturbation_size
        error_pos = compute_error_magnitude(compute_pose_error(perturbed_pos, target))
        
        # Negative perturbation
        perturbed_neg = current.copy()
        perturbed_neg[dim] -= perturbation_size
        error_neg = compute_error_magnitude(compute_pose_error(perturbed_neg, target))
        
        # Gradient: direction that decreases error (negative gradient)
        # If positive perturbation reduces error, gradient is positive
        gradient[dim] = (error_neg - error_pos) / (2 * perturbation_size)
    
    return gradient


def iterative_movel_servo_with_gradient(
    current_pose,
    target_pose,
    first_gain=1.0,
    subsequent_gain=0.2,
    momentum=0.5,
    noise_percent=0.0005,
    perturbation_size=0.001,
    max_iterations=50,
    tolerance=0.001,
    use_gradient=True,
):
    """
    Iterative moveL visual servoing with gradient approximation.
    
    For subsequent moves (after the first), estimates gradient by jogging
    in ±xyz and ±roll-pitch-yaw directions before making correction.

    Args:
        current_pose: Initial 6-DOF pose [x, y, z, rx, ry, rz]
        target_pose: Target 6-DOF pose [x, y, z, rx, ry, rz]
        first_gain: Gain for first move (1.0 = go directly to target)
        subsequent_gain: Gain for subsequent moves (0.2 = gradual convergence)
        momentum: Momentum smoothing for subsequent moves (0.5 = 50% smoothing)
        noise_percent: Noise level (0.0005 = 0.05%)
        perturbation_size: Size of perturbation for gradient estimation
        max_iterations: Maximum iterations
        tolerance: Convergence tolerance in meters
        use_gradient: If True, use gradient approximation; if False, use direct error

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

        # First move: go directly to target
        if i == 0:
            gain = first_gain
            current_momentum = 1.0  # No smoothing for first move
            correction = gain * error
        else:
            # Subsequent moves: use gradient approximation
            gain = subsequent_gain
            current_momentum = momentum
            
            if use_gradient:
                # Estimate gradient by jogging in ±xyz and ±roll-pitch-yaw
                gradient = estimate_gradient(pose, target, perturbation_size)
                # Normalize gradient and scale by gain and error magnitude
                grad_norm = np.linalg.norm(gradient)
                if grad_norm > 1e-10:
                    correction = gain * error_mag * (gradient / grad_norm)
                else:
                    correction = gain * error  # Fallback to direct error
            else:
                correction = gain * error

        # Add realistic noise (simulates AprilTag detection uncertainty)
        correction = add_noise(correction, noise_percent)

        # Apply momentum smoothing
        velocity = current_momentum * correction + (1 - current_momentum) * velocity

        # Execute moveL (update pose)
        pose = pose + velocity

        trajectory.append(pose.copy())
        print(f"Iteration {i:2d}: Error = {error_mag:.6f}m, Correction = {np.linalg.norm(correction):.6f}m")

    return trajectory


def main():
    """Run minimal example with gradient approximation."""
    # Example poses (x, y, z, rx, ry, rz) in meters and radians
    current_pose = [0.3, 0.2, 0.5, 0.1, -0.2, 0.15]
    target_pose = [0.4, 0.3, 0.55, 0.0, 0.0, 0.0]

    print("=" * 70)
    print("Iterative moveL Visual Servoing with Gradient Approximation")
    print("=" * 70)
    print(f"Start pose:  {current_pose}")
    print(f"Target pose: {target_pose}")
    print(f"Initial error: {compute_error_magnitude(compute_pose_error(current_pose, target_pose)):.6f}m")
    print("-" * 70)
    print("Strategy:")
    print("  1. First moveL: Go directly to target (gain=1.0)")
    print("  2. Subsequent moveL: Jog ±xyz & ±rpy to estimate gradient, then correct")
    print("-" * 70)

    trajectory = iterative_movel_servo_with_gradient(
        current_pose=current_pose,
        target_pose=target_pose,
        first_gain=1.0,           # First move goes directly to target
        subsequent_gain=0.2,      # Subsequent moves: gradual convergence
        momentum=0.5,             # 50% momentum smoothing after first move
        noise_percent=0.0005,     # 0.05% noise
        perturbation_size=0.001,  # 1mm perturbation for gradient estimation
        max_iterations=30,
        tolerance=0.001,
        use_gradient=True,        # Use gradient approximation
    )

    print("-" * 70)
    final_error = compute_error_magnitude(compute_pose_error(trajectory[-1], target_pose))
    print(f"Final pose:  {trajectory[-1].tolist()}")
    print(f"Final error: {final_error:.6f}m")
    print(f"Total iterations: {len(trajectory) - 1}")


if __name__ == "__main__":
    main()
