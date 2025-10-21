import json
import numpy as np

# Load current calibration
with open('src/ur_toolkit/hand_eye_calibration/hand_eye_calibration.json') as f:
    current = json.load(f)

print("=== HAND-EYE CALIBRATION COMPARISON ===\n")

# Previous calibration data (from conversation history)
prev_poses = 8
prev_translation = [-113.73, 11.49, 28.88]  # mm
prev_trans_error = 753.38  # mm
prev_rot_error = 119.06  # degrees

# Current calibration data
curr_poses = current['calibration_info']['poses_collected']
curr_trans = current['hand_eye_transform']['translation_mm']
curr_translation = [curr_trans['x'], curr_trans['y'], curr_trans['z']]
curr_accuracy = current['calibration_accuracy']['residuals']
curr_trans_error = curr_accuracy['average_translation_error_mm']
curr_rot_error = curr_accuracy['average_rotation_error_deg']

print("📅 PREVIOUS CALIBRATION (Sep 26):")
print(f"   Poses collected: {prev_poses}")
print(f"   Translation: [{prev_translation[0]:.1f}, {prev_translation[1]:.1f}, {prev_translation[2]:.1f}] mm")
print(f"   Average errors: {prev_trans_error:.1f}mm translation, {prev_rot_error:.1f}° rotation")
print()

print("📅 CURRENT CALIBRATION (Sep 29):")
print(f"   Poses collected: {curr_poses}")
print(f"   Translation: [{curr_translation[0]:.1f}, {curr_translation[1]:.1f}, {curr_translation[2]:.1f}] mm")
print(f"   Average errors: {curr_trans_error:.1f}mm translation, {curr_rot_error:.1f}° rotation")
print()

# Calculate differences
translation_shift = np.array(curr_translation) - np.array(prev_translation)
total_shift = np.linalg.norm(translation_shift)
error_change_trans = curr_trans_error - prev_trans_error
error_change_rot = curr_rot_error - prev_rot_error

print("🔍 COMPARISON ANALYSIS:")
print(f"   Translation shift: [{translation_shift[0]:.1f}, {translation_shift[1]:.1f}, {translation_shift[2]:.1f}] mm")
print(f"   Total position shift: {total_shift:.1f} mm")
print(f"   Error change: {error_change_trans:+.1f}mm translation, {error_change_rot:+.1f}° rotation")
print()

print("📊 QUALITY ASSESSMENT:")
print(f"   Data quantity: {'✅ Better' if curr_poses > prev_poses else '❌ Same/Worse'} ({curr_poses} vs {prev_poses} poses)")
print(f"   Position stability: {'✅ Good' if total_shift < 50 else '⚠️ Significant shift'} ({total_shift:.1f}mm shift)")
print(f"   Error consistency: {'✅ Consistent' if abs(error_change_trans) < 100 else '⚠️ Variable'} ({error_change_trans:+.1f}mm change)")

# Interpretation
print("\n🎯 INTERPRETATION:")
if total_shift < 30:
    print("   Position difference is small - calibrations are reasonably consistent")
elif total_shift < 60:
    print("   Moderate position difference - some variation but acceptable")
else:
    print("   Large position difference - indicates calibration variability")

if abs(error_change_trans) < 50:
    print("   Error levels are similar - both calibrations have comparable internal consistency")
else:
    print("   Different error levels - may indicate different pose diversity or quality")

print(f"\n💡 RECOMMENDATION:")
if curr_poses > prev_poses and total_shift < 50:
    print("   Current calibration is better: More poses + consistent position")
elif total_shift > 60:
    print("   Consider recalibrating with more rotational diversity for stability")
else:
    print("   Both calibrations are reasonable - use current for most recent data")