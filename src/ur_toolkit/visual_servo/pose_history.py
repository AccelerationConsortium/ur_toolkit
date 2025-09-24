#!/usr/bin/env python3
"""
Pose History Manager
Manages pose correction history and dynamic pose storage for visual servoing
"""

import json
import yaml
import numpy as np
from datetime import datetime
from pathlib import Path
from typing import Dict, List, Optional, Any
import copy


class PoseHistoryManager:
    """Manages pose history and dynamic updates for visual servoing"""

    def __init__(self, positions_file: Path, config):
        """
        Initialize pose history manager

        Args:
            positions_file: Path to taught positions file
            config: Visual servo configuration instance
        """
        self.positions_file = positions_file
        self.config = config
        self.history_file = positions_file.parent / "pose_correction_history.json"

        # Load existing history
        self.correction_history = self._load_history()

    def record_correction(self, position_name: str, original_pose: np.ndarray,
                          corrected_pose: np.ndarray, tag_pose_original: np.ndarray,
                          tag_pose_current: np.ndarray, correction_metrics: Dict[str, Any]):
        """
        Record a pose correction in history

        Args:
            position_name: Name of the position corrected
            original_pose: Original robot pose
            corrected_pose: Corrected robot pose
            tag_pose_original: Original AprilTag pose
            tag_pose_current: Current AprilTag pose
            correction_metrics: Metrics from the correction process
        """
        if not self.config.enable_pose_history:
            return

        timestamp = datetime.now().isoformat()

        # Create correction record
        correction_record = {
            'timestamp': timestamp,
            'position_name': position_name,
            'original_robot_pose': original_pose.tolist(),
            'corrected_robot_pose': corrected_pose.tolist(),
            'original_tag_pose': tag_pose_original.tolist(),
            'current_tag_pose': tag_pose_current.tolist(),
            'correction_delta': (corrected_pose - original_pose).tolist(),
            'tag_delta': (tag_pose_current - tag_pose_original).tolist(),
            'metrics': correction_metrics
        }

        # Add to history
        if position_name not in self.correction_history:
            self.correction_history[position_name] = []

        self.correction_history[position_name].append(correction_record)

        # Limit history size
        max_entries = self.config.max_history_entries
        if len(self.correction_history[position_name]) > max_entries:
            self.correction_history[position_name] = self.correction_history[position_name][-max_entries:]

        # Save to file
        self._save_history()

        print(f"📝 Recorded correction for '{position_name}' at {timestamp}")

    def update_position_pose(self, position_name: str, new_pose: np.ndarray,
                             new_tag_pose: Optional[np.ndarray] = None) -> bool:
        """
        Update stored position with corrected pose

        Args:
            position_name: Name of position to update
            new_pose: New robot pose
            new_tag_pose: New AprilTag pose (optional)

        Returns:
            True if update successful
        """
        try:
            # Load current positions
            positions_data = self._load_positions()

            if position_name not in positions_data.get('positions', {}):
                print(f"❌ Position '{position_name}' not found")
                return False

            # Create backup of original
            original_pose = copy.deepcopy(positions_data['positions'][position_name]['coordinates'])

            # Update the pose
            positions_data['positions'][position_name]['coordinates'] = new_pose.tolist()

            # Update tag pose if provided
            if new_tag_pose is not None:
                positions_data['positions'][position_name]['camera_to_tag'] = new_tag_pose.tolist()

            # Add update metadata
            positions_data['positions'][position_name]['last_visual_servo_update'] = datetime.now().isoformat()

            # Save updated positions
            self._save_positions(positions_data)

            print(f"✅ Updated position '{position_name}' with corrected pose")
            print(f"   Original: {original_pose}")
            print(f"   Updated:  {new_pose.tolist()}")

            return True

        except Exception as e:
            print(f"❌ Failed to update position '{position_name}': {e}")
            return False

    def get_correction_statistics(self, position_name: str) -> Dict[str, Any]:
        """
        Get correction statistics for a position

        Args:
            position_name: Position to analyze

        Returns:
            Dictionary with correction statistics
        """
        if position_name not in self.correction_history:
            return {'correction_count': 0}

        corrections = self.correction_history[position_name]

        if not corrections:
            return {'correction_count': 0}

        # Calculate statistics
        correction_deltas = [np.array(c['correction_delta']) for c in corrections]
        translation_magnitudes = [np.linalg.norm(d[:3]) for d in correction_deltas]
        rotation_magnitudes = [np.linalg.norm(d[3:]) for d in correction_deltas]

        stats = {
            'correction_count': len(corrections),
            'last_correction': corrections[-1]['timestamp'],
            'avg_translation_correction': np.mean(translation_magnitudes),
            'max_translation_correction': np.max(translation_magnitudes),
            'avg_rotation_correction': np.mean(rotation_magnitudes),
            'max_rotation_correction': np.max(rotation_magnitudes),
            'recent_corrections': len([c for c in corrections if self._is_recent(c['timestamp'])])
        }

        return stats

    def _load_positions(self) -> Dict[str, Any]:
        """Load current taught positions"""
        try:
            with open(self.positions_file, 'r') as f:
                return yaml.safe_load(f) or {}
        except Exception as e:
            print(f"❌ Failed to load positions: {e}")
            return {}

    def _save_positions(self, positions_data: Dict[str, Any]):
        """Save updated positions"""
        try:
            with open(self.positions_file, 'w') as f:
                yaml.dump(positions_data, f, default_flow_style=False, sort_keys=False, indent=2)
        except Exception as e:
            print(f"❌ Failed to save positions: {e}")

    def _load_history(self) -> Dict[str, List[Dict]]:
        """Load correction history from file"""
        try:
            if self.history_file.exists():
                with open(self.history_file, 'r') as f:
                    return json.load(f)
        except Exception as e:
            print(f"⚠️  Failed to load history: {e}")

        return {}

    def _save_history(self):
        """Save correction history to file"""
        try:
            with open(self.history_file, 'w') as f:
                json.dump(self.correction_history, f, indent=2)
        except Exception as e:
            print(f"❌ Failed to save history: {e}")

    def _is_recent(self, timestamp_str: str, hours: int = 24) -> bool:
        """Check if timestamp is within recent hours"""
        try:
            timestamp = datetime.fromisoformat(timestamp_str)
            return (datetime.now() - timestamp).total_seconds() < (hours * 3600)
        except Exception:
            return False

    def print_history_summary(self, position_name: Optional[str] = None):
        """Print correction history summary"""
        if position_name:
            if position_name in self.correction_history:
                stats = self.get_correction_statistics(position_name)
                print(f"📊 Correction History for '{position_name}':")
                print(f"   Total corrections: {stats['correction_count']}")
                if stats['correction_count'] > 0:
                    print(f"   Last correction: {stats['last_correction']}")
                    print(f"   Avg translation: {stats['avg_translation_correction']:.4f}m")
                    print(f"   Max translation: {stats['max_translation_correction']:.4f}m")
                    print(f"   Recent corrections (24h): {stats['recent_corrections']}")
            else:
                print(f"📊 No correction history for '{position_name}'")
        else:
            print("📊 Overall Correction History:")
            total_corrections = sum(len(corrections) for corrections in self.correction_history.values())
            print(f"   Positions with corrections: {len(self.correction_history)}")
            print(f"   Total corrections: {total_corrections}")

            for pos_name in self.correction_history:
                stats = self.get_correction_statistics(pos_name)
                print(f"   {pos_name}: {stats['correction_count']} corrections")

    def update_position_tag_association(self, position_name: str, tag_reference: str,
                                        camera_to_tag_transform: list) -> bool:
        """
        Update a position with AprilTag association for visual servoing

        Args:
            position_name: Name of position to update
            tag_reference: Tag reference string (e.g., "tag_123")
            camera_to_tag_transform: 6D pose transform from camera to tag

        Returns:
            True if update successful
        """
        try:
            # Load current positions
            positions_data = self._load_positions()

            if position_name not in positions_data.get('positions', {}):
                print(f"❌ Position '{position_name}' not found")
                return False

            # Update the position with AprilTag association
            positions_data['positions'][position_name]['tag_reference'] = tag_reference
            positions_data['positions'][position_name]['camera_to_tag'] = camera_to_tag_transform
            positions_data['positions'][position_name]['has_apriltag_view'] = True
            positions_data['positions'][position_name]['visual_servo_enabled'] = True
            positions_data['positions'][position_name]['last_tag_association_update'] = datetime.now().isoformat()

            # Save updated positions
            self._save_positions(positions_data)

            print(f"✅ Updated position '{position_name}' with AprilTag association: {tag_reference}")
            return True

        except Exception as e:
            print(f"❌ Failed to update tag association for '{position_name}': {e}")
            return False

    def update_equipment_positions(self, position_name: str, pose_correction: np.ndarray) -> bool:
        """
        Update all positions associated with the same equipment when visual servoing detects an offset

        Args:
            position_name: Name of the position that was corrected
            pose_correction: The pose correction that was applied [x, y, z, rx, ry, rz]

        Returns:
            True if update successful
        """
        try:
            # Load current positions
            positions_data = self._load_positions()

            if position_name not in positions_data.get('positions', {}):
                print(f"❌ Position '{position_name}' not found")
                return False

            # Get the equipment name of the corrected position
            corrected_position = positions_data['positions'][position_name]
            equipment_name = corrected_position.get('equipment_name')

            if not equipment_name:
                print(f"⚠️  Position '{position_name}' has no equipment_name, only updating this position")
                return self.update_position_pose(position_name,
                                                 np.array(corrected_position['coordinates']) + pose_correction)

            print(f"🔧 Updating all positions for equipment: {equipment_name}")

            # Find all positions with the same equipment_name
            updated_positions = []
            for pos_name, pos_data in positions_data['positions'].items():
                if pos_data.get('equipment_name') == equipment_name:
                    # Apply the same correction to this position
                    original_coords = np.array(pos_data['coordinates'])
                    corrected_coords = original_coords + pose_correction

                    # Update the coordinates
                    pos_data['coordinates'] = corrected_coords.tolist()
                    pos_data['last_visual_servo_update'] = datetime.now().isoformat()
                    pos_data['visual_servo_correction_applied'] = pose_correction.tolist()

                    updated_positions.append(pos_name)
                    print(f"   ✅ Updated {pos_name}: {original_coords} → {corrected_coords}")

            # Save the updated positions file
            self._save_positions(positions_data)

            print(f"💾 Successfully updated {len(updated_positions)} positions for equipment '{equipment_name}'")
            print(f"   Updated positions: {updated_positions}")

            return True

        except Exception as e:
            print(f"❌ Failed to update equipment positions: {e}")
            return False
