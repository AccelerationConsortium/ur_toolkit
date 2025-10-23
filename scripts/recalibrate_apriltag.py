#!/usr/bin/env python3
"""
One-shot AprilTag recalibration helper

Moves the robot to an observation pose, averages a few AprilTag detections,
computes the tag error vs the stored camera_to_tag, transforms that into a
robot TCP correction, and (optionally) applies the correction to the taught
positions (single position or propagated to equipment-linked positions).

Safe defaults: 8 samples, short delays, snapshot of taught positions before write.
"""

import time
import json
from pathlib import Path
from datetime import datetime

import numpy as np

import sys
from pathlib import Path as _Path

# Ensure the repository 'src' directory is on sys.path so package imports work
_project_root = _Path(__file__).parent.parent
_src_path = _project_root / 'src'
if str(_src_path) not in sys.path:
    sys.path.insert(0, str(_src_path))

from ur_toolkit.config_manager import config
from ur_toolkit.robots.ur.ur_controller import URController
from ur_toolkit.visual_servo.visual_servo_engine import VisualServoEngine


def average_tag_detection(engine, tag_id, samples=8, delay=0.15):
    """Capture multiple filtered tag detections and return their average pose."""
    detections = []
    for i in range(samples):
        pose = engine.detection_filter.get_filtered_tag_pose(tag_id)
        if pose is None:
            print(f"⚠️  Detection {i+1}/{samples} failed")
        else:
            detections.append(np.array(pose))
            print(f"   Detected {i+1}/{samples}: {pose}")
        time.sleep(delay)

    if not detections:
        return None

    stacked = np.stack(detections, axis=0)
    mean_pose = np.mean(stacked, axis=0)
    return mean_pose


def run_recalibration(
    position_name: str,
    samples: int = 8,
    apply: bool = False,
    propagate: bool = False,
    move_after_apply: bool = False,
    max_iterations: int = 1,
    verify: bool = False,
    auto_confirm: bool = False,
):
    positions_file = config.resolve_path('taught_positions.yaml')

    print(f"🔧 Recalibration starting for position: {position_name}")
    print(f"📁 Positions file: {positions_file}")

    # Connect to robot
    robot_ip = config.get('robot.ip_address')
    robot = URController(robot_ip)

    # Create engine (it will create detection_filter and pose_history)
    engine = VisualServoEngine(robot, positions_file)

    pos_data = engine._get_position_data(position_name)
    if not pos_data:
        print(f"❌ Position '{position_name}' not found in positions file")
        return False

    # Determine observation info
    if pos_data.get('camera_to_tag'):
        # Direct tag view on this position
        obs_pose_name = position_name
        stored_tag_pose = np.array(pos_data['camera_to_tag'])
        tag_reference = pos_data.get('tag_reference')
        stored_robot_pose = np.array(pos_data.get('coordinates', [0, 0, 0, 0, 0, 0]))
    elif pos_data.get('observation_pose'):
        obs_pose_name = pos_data['observation_pose']
        obs_data = engine._get_position_data(obs_pose_name)
        if not obs_data or not obs_data.get('camera_to_tag'):
            print(f"❌ Observation pose '{obs_pose_name}' has no camera view")
            return False
        stored_tag_pose = np.array(obs_data['camera_to_tag'])
        tag_reference = obs_data.get('tag_reference')
        stored_robot_pose = np.array(obs_data.get('coordinates', [0, 0, 0, 0, 0, 0]))
    else:
        print(f"❌ Position '{position_name}' has no camera_to_tag or observation_pose")
        return False

    # Resolve tag id
    if isinstance(tag_reference, str) and tag_reference.startswith('tag_'):
        tag_id = int(tag_reference.split('_', 1)[1])
    else:
        tag_id = int(tag_reference)

    print(f"🏷️  Using AprilTag {tag_id} (observation pose: {obs_pose_name})")

    # Move robot to observation pose
    target_coords = stored_robot_pose.copy()
    print(f"🤖 Moving to observation pose: {target_coords.tolist()}")
    success = robot.move_to_pose(target_coords)
    if not success:
        print("❌ Failed to move to observation pose")
        return False

    # Give camera a moment to stabilize
    time.sleep(0.5)

    # Average detections
    observed_tag_pose = average_tag_detection(engine, tag_id, samples=samples)
    if observed_tag_pose is None:
        print("❌ No detections collected - aborting")
        return False

    print(f"🔎 Observed tag pose (avg): {observed_tag_pose.tolist()}")
    print(f"🔖 Stored tag pose: {stored_tag_pose.tolist()}")

    # Compute tag error and transform to robot correction
    tag_error = observed_tag_pose - stored_tag_pose
    robot_correction = engine._transform_tag_error_to_robot_correction(tag_error)

    print(f"🔧 Computed robot correction (TCP): {robot_correction.tolist()}")

    # Prepare metrics and snapshot (single metrics file for the run)
    runs_dir = Path('logs') / 'visual_servo_runs'
    runs_dir.mkdir(parents=True, exist_ok=True)
    ts = datetime.now().strftime('%Y%m%d_%H%M%S')
    metrics_file = runs_dir / f"{ts}_recal_{position_name}.json"

    metrics = {
        'timestamp': ts,
        'position': position_name,
        'observation_pose': obs_pose_name,
        'tag_id': tag_id,
        'samples': samples,
        'iterations': [],
        'snapshot': None,
    }

    # Snapshot positions before making any changes
    snap = engine.pose_history.snapshot_positions(f"taught_positions.snapshot.{ts}.yaml")
    if snap:
        metrics['snapshot'] = str(snap)

    confirm_threshold = 0.05  # meters; if correction > this, prompt unless auto_confirm

    # Iterative correction loop
    for iteration in range(1, max_iterations + 1):
        print(f"\n🔁 Recalibration iteration {iteration}/{max_iterations}")

        # Move to observation pose
        target_coords = stored_robot_pose.copy()
        print(f"🤖 Moving to observation pose: {target_coords.tolist()}")
        success = robot.move_to_pose(target_coords)
        if not success:
            print("❌ Failed to move to observation pose")
            break

        time.sleep(0.5)

        # Re-observe
        observed_tag_pose = average_tag_detection(engine, tag_id, samples=samples)
        if observed_tag_pose is None:
            print("❌ No detections collected - aborting")
            break

        tag_error = observed_tag_pose - stored_tag_pose
        robot_correction = engine._transform_tag_error_to_robot_correction(tag_error)

        corr_trans_norm = float(np.linalg.norm(robot_correction[:3]))
        corr_rot_norm = float(np.linalg.norm(robot_correction[3:]))

        iter_record = {
            'iteration': iteration,
            'observed_tag_pose': observed_tag_pose.tolist(),
            'stored_tag_pose': stored_tag_pose.tolist(),
            'tag_error': tag_error.tolist(),
            'robot_correction': robot_correction.tolist(),
            'translation_correction_norm_m': corr_trans_norm,
            'rotation_correction_norm_rad': corr_rot_norm,
        }
        metrics['iterations'].append(iter_record)

        print(f"� Computed robot correction (TCP): {robot_correction.tolist()}")

        # If not applying changes, stop after first iteration (dry-run)
        if not apply:
            print("ℹ️  Dry run (no apply). Review computed correction above.")
            break

        # Confirm large corrections
        if corr_trans_norm > confirm_threshold and not auto_confirm:
            print(f"⚠️  Computed translation correction {corr_trans_norm:.3f} m exceeds threshold {confirm_threshold} m")
            resp = input("Apply and move the robot to the corrected pose? (y/N): ").strip().lower()
            if resp not in ['y', 'yes']:
                print("⏏️  User declined to apply correction")
                break

        # Apply correction to YAML (either the obs pose or the target position)
        print("💾 Applying correction to positions in YAML...")
        if obs_pose_name == position_name:
            new_pose = np.array(stored_robot_pose) + robot_correction
            ok = engine.pose_history.update_position_pose(position_name, new_pose, new_tag_pose=observed_tag_pose)
            if ok and propagate:
                engine.pose_history.update_equipment_positions(position_name, robot_correction)
        else:
            new_obs_pose = np.array(stored_robot_pose) + robot_correction
            ok = engine.pose_history.update_position_pose(obs_pose_name, new_obs_pose, new_tag_pose=observed_tag_pose)
            if ok:
                engine.pose_history.update_equipment_positions(obs_pose_name, robot_correction)

        # Record correction in history
        try:
            orig = np.array(pos_data.get('coordinates', [0, 0, 0, 0, 0, 0]))
            corrected = orig + robot_correction
            engine.pose_history.record_correction(position_name, orig, corrected, stored_tag_pose, observed_tag_pose, iter_record)
        except Exception:
            pass

        # Optionally move the robot to the corrected pose for verification
        if move_after_apply:
            # Target pose depends on which pose was updated
            if obs_pose_name == position_name:
                target_pose = np.array(stored_robot_pose) + robot_correction
            else:
                target_pose = np.array(stored_robot_pose) + robot_correction

            print(f"🚶 Moving robot to corrected pose for verification: {target_pose.tolist()}")
            # Conservative speeds
            try:
                move_ok = robot.move_to_pose(target_pose, speed=0.02, acceleration=0.01)
            except TypeError:
                # Older move_to_pose signature may not accept speed/accel args; call without
                move_ok = robot.move_to_pose(target_pose)

            if not move_ok:
                print("❌ Failed to move to corrected pose (verification)")

            time.sleep(0.5)

            # Re-observe to compute residual
            re_observed = average_tag_detection(engine, tag_id, samples=samples)
            if re_observed is not None:
                residual_tag_error = re_observed - stored_tag_pose
                residual_trans = float(np.linalg.norm(residual_tag_error[:3]))
                residual_rot = float(np.linalg.norm(residual_tag_error[3:]))
                metrics['iterations'][-1].update({
                    'post_move_observed_tag_pose': re_observed.tolist(),
                    'residual_tag_error': residual_tag_error.tolist(),
                    'residual_translation_norm_m': residual_trans,
                    'residual_rotation_norm_rad': residual_rot,
                })

                print(f"🔁 Post-move residual translation: {residual_trans:.4f} m, rotation: {residual_rot:.4f} rad")

                # Check tolerances from engine config
                pos_tol = getattr(engine.config, 'position_tolerance', 0.003)
                rot_tol = getattr(engine.config, 'rotation_tolerance', 0.017)
                if residual_trans < pos_tol and residual_rot < rot_tol:
                    print("✅ Residual within tolerance — recalibration successful")
                    break
                else:
                    print("⚠️  Residual above tolerance — continuing to next iteration (if any)")
            else:
                print("⚠️  Post-move re-detection failed; continuing to next iteration (if any)")

        # Prepare for next iteration: update stored_tag_pose/stored_robot_pose from YAML (reloaded)
        refreshed = engine._get_position_data(obs_pose_name)
        if refreshed and refreshed.get('camera_to_tag'):
            stored_tag_pose = np.array(refreshed['camera_to_tag'])
            stored_robot_pose = np.array(refreshed.get('coordinates', stored_robot_pose.tolist()))

    # Final persist of metrics for this run
    try:
        with open(metrics_file, 'w') as mf:
            json.dump(metrics, mf, indent=2)
        print(f"📝 Metrics written to {metrics_file}")
    except Exception as e:
        print(f"⚠️  Failed to write metrics file: {e}")

    print("✅ Recalibration run completed")
    return True


if __name__ == '__main__':
    import argparse

    parser = argparse.ArgumentParser(description='One-shot AprilTag recalibration helper')
    parser.add_argument('position', help='Name of the taught position to recalibrate')
    parser.add_argument('--samples', type=int, default=8, help='Number of detection samples to average')
    parser.add_argument('--apply', action='store_true', help='Apply correction to taught positions')
    parser.add_argument('--propagate', action='store_true', help='Propagate correction to equipment-linked positions')
    parser.add_argument('--move-after-apply', action='store_true', help='Move robot to corrected pose after applying correction (verification)')
    parser.add_argument('--max-iterations', type=int, default=1, help='Maximum iterations of apply->verify cycle')
    parser.add_argument('--verify', action='store_true', help='Perform verification re-observation after move')
    parser.add_argument('-y', '--yes', action='store_true', dest='yes', help='Auto-confirm apply without prompting')

    args = parser.parse_args()

    run_recalibration(
        args.position,
        samples=args.samples,
        apply=args.apply,
        propagate=args.propagate,
        move_after_apply=args.move_after_apply,
        max_iterations=args.max_iterations,
        verify=args.verify,
        auto_confirm=args.yes,
    )
