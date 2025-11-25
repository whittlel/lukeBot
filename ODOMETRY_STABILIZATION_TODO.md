# Odometry Stabilization Fix - TODO for Tomorrow

**Date Created**: 2025-11-19
**Status**: Ready to implement
**Priority**: HIGH - Required for reliable autonomous navigation

---

## Problem Statement

The hybrid depth/visual odometry system is **unstable** - rapidly switching between DEPTH and VISUAL modes, causing pose discontinuities and unreliable navigation.

### Current Symptoms

1. **Odometry Source Indicator**: Alternating between "DEPTH" (cyan) and "VISUAL" (magenta) frequently
2. **Pose Jumps**: Discontinuous pose updates when switching modes
3. **Display Bug**: Pose showing `(0.19, 0.27, 9.7??)` - degree symbol rendering as "??"
4. **Performance**: ~5 FPS in Mode 3 (Multi-Panel visualization)
5. **Visual Features**: Red dots on object edges (correct but low quality for white/tan environment)

### User Observation

> "the odometry is switching between depth and visual, the pose from the depth seems most accurate"

**Key Finding**: Depth odometry produces accurate poses when it works, but fails on ~50% of frames.

---

## Root Cause Analysis

### Why Depth ICP is Failing

1. **Insufficient Points**: `min_points: 50` threshold too strict
   - Low-texture environment already has sparse depth features
   - Current sample_rate of 30 reduces point cloud density
   - Many frames fall below minimum point threshold

2. **Strict Error Threshold**: `if error > 0.5:` fails too often
   - ICP error can legitimately be 0.5-1.0m in dynamic scenes
   - Threshold too conservative for real-world use

3. **Limited Depth Range**: `max_depth: 3.0m` may be too restrictive
   - Reduces available points for ICP matching
   - User's environment may need longer range

4. **Convergence Issues**: `icp_convergence: 0.01` may be too tight
   - Takes more iterations to converge
   - May timeout before finding good solution

### Why Rapid Mode Switching is Bad

- Each mode switch causes **pose discontinuity**
- Visual features are unreliable in low-texture environment
- Switching back to visual introduces large drift errors
- Creates "jittery" pose estimates that break path planning

---

## Proposed Solution - 5 Phase Fix

### Phase 1: Fix Display Bug ✏️

**File**: `main.py` (around line 297)

**Change**:
```python
# BEFORE:
cv2.putText(frame, f"Pose: ({pose.x:.2f}, {pose.y:.2f}, {pose.theta:.1f}°)",
           (10, 70), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)

# AFTER:
theta_deg = np.degrees(pose.theta)
cv2.putText(frame, f"Pose: ({pose.x:.2f}, {pose.y:.2f}, {theta_deg:.1f}deg)",
           (10, 70), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
```

**Why**: Degree symbol (°) may not render properly on some systems. Use "deg" text instead.

---

### Phase 2: Relax Depth Odometry Constraints ⚙️

**File**: `config/slam_config.yaml`

**Changes**:
```yaml
slam:
  depth_odometry:
    icp_max_iterations: 5             # Keep as is (performance)
    icp_convergence: 0.02             # CHANGE: 0.01 → 0.02 (faster convergence)
    max_correspondence_distance: 0.2  # Keep as is
    sample_rate: 25                   # CHANGE: 30 → 25 (more points)
    min_depth: 0.1                    # Keep as is
    max_depth: 5.0                    # CHANGE: 3.0 → 5.0 (more range)
    min_points: 30                    # CHANGE: 50 → 30 (lower threshold)
    use_imu: true                     # Keep as is
    imu_alpha: 0.7                    # Keep as is
```

**Rationale**:
- `min_points: 30` - Allow ICP with fewer points (still enough for stable registration)
- `max_depth: 5.0m` - Capture more of the environment for point matching
- `sample_rate: 25` - Balance between speed and point density
- `icp_convergence: 0.02` - Relax convergence for faster results

---

### Phase 3: Increase ICP Error Tolerance 📊

**File**: `src/slam/depth_odometry.py` (line 254)

**Change**:
```python
# BEFORE:
if error == float('inf') or error > 0.5:  # High error threshold
    # ICP failed
    self.prev_point_cloud = current_cloud
    return None

# AFTER:
if error == float('inf') or error > 1.0:  # Relaxed error threshold
    # ICP failed
    self.prev_point_cloud = current_cloud
    return None
```

**Why**: Allow slightly higher ICP errors. Real-world motion can have legitimate 0.5-1.0m residual errors.

---

### Phase 4: Add Hysteresis to Prevent Rapid Switching 🔄

**File**: `src/slam/slam_engine.py`

**Step 1**: Add state tracking in `__init__` (around line 30):
```python
def __init__(self, config=None):
    # ... existing code ...

    # Odometry mode switching with hysteresis
    self.depth_failure_count = 0
    self.visual_failure_count = 0
    self.odometry_mode_sticky = 'depth'  # Start with depth
    self.mode_switch_threshold = 3  # Require 3 consecutive failures
```

**Step 2**: Modify hybrid odometry logic in `process_frame` (around line 150):
```python
# BEFORE:
if self.odometry_mode == 'hybrid':
    delta_pose = None
    odometry_source = "none"

    # Try depth first
    if depth_image is not None:
        depth_pose = self.depth_odometry.estimate_pose(depth_image, rgb_image, imu_data)
        if depth_pose is not None:
            delta_pose = depth_pose
            odometry_source = "depth"

    # Fallback to visual if depth failed
    if delta_pose is None:
        visual_pose = self.visual_odometry.estimate_pose(rgb_image, depth_image, imu_data)
        if visual_pose is not None:
            delta_pose = visual_pose
            odometry_source = "visual"

# AFTER:
if self.odometry_mode == 'hybrid':
    delta_pose = None
    odometry_source = "none"

    # Try depth first
    if depth_image is not None:
        depth_pose = self.depth_odometry.estimate_pose(depth_image, rgb_image, imu_data)

        if depth_pose is not None:
            delta_pose = depth_pose
            odometry_source = "depth"
            self.depth_failure_count = 0  # Reset failure counter
            self.odometry_mode_sticky = 'depth'
        else:
            self.depth_failure_count += 1

    # Only fallback to visual after multiple consecutive depth failures
    if delta_pose is None and self.depth_failure_count >= self.mode_switch_threshold:
        visual_pose = self.visual_odometry.estimate_pose(rgb_image, depth_image, imu_data)

        if visual_pose is not None:
            delta_pose = visual_pose
            odometry_source = "visual"
            self.visual_failure_count = 0
            self.odometry_mode_sticky = 'visual'
        else:
            self.visual_failure_count += 1

    # If still no pose but we're in sticky depth mode, keep trying depth
    if delta_pose is None and self.odometry_mode_sticky == 'depth':
        odometry_source = "depth_retry"
```

**Why**: This prevents single-frame failures from switching modes. Requires 3 consecutive failures before switching to visual fallback.

---

### Phase 5: Add Diagnostic Logging 🔍

**File**: `src/slam/depth_odometry.py`

**Add logging in `estimate_pose` method** (around line 236):

```python
def estimate_pose(self, depth_image: np.ndarray,
                 rgb_image: Optional[np.ndarray] = None,
                 imu_data: Optional[dict] = None) -> Optional[RobotPose]:
    """..."""
    try:
        # Convert depth to point cloud
        current_cloud = self.depth_to_point_cloud(depth_image, rgb_image)

        if len(current_cloud) < self.min_points:
            # ADD THIS:
            print(f"[DEPTH] Insufficient points: {len(current_cloud)}/{self.min_points}")
            self.prev_point_cloud = current_cloud
            return RobotPose(0.0, 0.0, 0.0)

        if self.prev_point_cloud is None or len(self.prev_point_cloud) < self.min_points:
            # First frame
            self.prev_point_cloud = current_cloud
            return RobotPose(0.0, 0.0, 0.0)

        # Perform ICP: align current cloud to previous cloud
        R, t, error = self.icp(current_cloud, self.prev_point_cloud)

        # Check if ICP succeeded
        if error == float('inf') or error > 1.0:  # Updated threshold
            # ADD THIS:
            print(f"[DEPTH] ICP error too high: {error:.3f}m (threshold: 1.0m)")
            self.prev_point_cloud = current_cloud
            return None

        # ADD THIS (successful case):
        print(f"[DEPTH] Success: {len(current_cloud)} points, error: {error:.3f}m")

        # ... rest of method ...
```

**Why**: Provides diagnostic information to understand why depth odometry fails.

---

### Phase 6 (Optional): Add Odometry Indicator to Mode 3 📺

**File**: `src/slam/slam_visualizer.py`

**Modify `create_multi_panel_debug_view`** (around line 250):

```python
def create_multi_panel_debug_view(self, rgb_image, depth_image, keypoints,
                                  prev_keypoints, matches, pose,
                                  camera_matrix, odometry_source="unknown"):
    # ... existing code ...

    # ADD: Display odometry source on the combined panel
    odom_color = (0, 255, 255) if odometry_source == "depth" else (255, 0, 255)
    cv2.putText(combined, f"Odometry: {odometry_source.upper()}",
               (10, combined.shape[0] - 10),
               cv2.FONT_HERSHEY_SIMPLEX, 0.7, odom_color, 2)

    return combined
```

**Update call in `slam_engine.py`** (around line 220):
```python
# BEFORE:
slam_viz = self.visualizer.create_multi_panel_debug_view(
    rgb_image, depth_image, current_keypoints,
    self.prev_keypoints, good_matches, self.current_pose,
    self.camera_matrix
)

# AFTER:
slam_viz = self.visualizer.create_multi_panel_debug_view(
    rgb_image, depth_image, current_keypoints,
    self.prev_keypoints, good_matches, self.current_pose,
    self.camera_matrix, self.last_odometry_source
)
```

---

## Testing Procedure

### Step 1: Verify Display Fix
```bash
cd /home/luke/lukeBot
python main.py
```
- Press 'm' until in Mode 1 (SLAM Debug)
- Check pose display: Should show `(0.00, 0.00, 0.0deg)` instead of "??"

### Step 2: Monitor Odometry Stability
- Move robot slowly with 'w'/'s' keys
- Watch odometry indicator (top of screen)
- **Expected**: Should stay "DEPTH" (cyan) for 90%+ of frames
- **Before fix**: Rapidly alternates between DEPTH and VISUAL

### Step 3: Check Diagnostic Logs
- Watch terminal output during movement
- Look for diagnostic messages:
  ```
  [DEPTH] Success: 156 points, error: 0.234m
  [DEPTH] Success: 142 points, error: 0.187m
  ```
- Occasional failures are OK:
  ```
  [DEPTH] Insufficient points: 28/30
  [DEPTH] ICP error too high: 1.234m (threshold: 1.0m)
  ```

### Step 4: Test Mode Switching Hysteresis
- Cover depth camera briefly (force failure)
- **Expected**: Should require 3 consecutive failures before switching to VISUAL
- Uncover camera
- **Expected**: Should switch back to DEPTH after 1 success

### Step 5: Verify Pose Accuracy
- Drive forward 1 meter
- Check pose: `x` should be ~1.0 ± 0.2
- Turn 90° left
- Check pose: `theta` should be ~90deg ± 10deg

---

## Expected Outcomes

### Before Fix
- ⚠️ Odometry switches every 1-3 frames
- ⚠️ Pose jumps/discontinuities
- ⚠️ ~50% depth failure rate
- ⚠️ Unreliable for autonomous navigation

### After Fix
- ✅ Odometry stable on DEPTH for 90%+ of frames
- ✅ Smooth pose updates
- ✅ ~10% depth failure rate
- ✅ Reliable enough for autonomous navigation
- ✅ Hysteresis prevents rapid switching

---

## Alternative Quick Test: Force Depth-Only Mode

If you want to quickly test if relaxed constraints help:

**File**: `config/slam_config.yaml`
```yaml
slam:
  odometry_mode: "depth"  # CHANGE: "hybrid" → "depth"
```

Run robot and observe:
- Should never show "VISUAL" (only uses depth)
- If pose updates smoothly → depth is working well
- If pose stops updating → depth failing too often, need to relax more

---

## Rollback Plan

If changes cause issues, revert:

```bash
cd /home/luke/lukeBot
git diff config/slam_config.yaml
git diff src/slam/depth_odometry.py
git diff src/slam/slam_engine.py
git diff main.py

# Revert specific file:
git checkout config/slam_config.yaml
```

Or restore original values:
- `min_points: 50`
- `max_depth: 3.0`
- `icp_convergence: 0.01`
- `sample_rate: 30`
- Remove hysteresis code

---

## Next Steps After Stabilization

Once depth odometry is stable:

1. **Test Autonomous Navigation**
   - Enable autonomous mode with 'n' key
   - Observe reactive obstacle avoidance
   - Verify depth-based obstacle detection works

2. **Implement AprilTag Detection**
   - Use `tools/generate_apriltags.py` to create markers
   - Print and place 10-15 tags around environment
   - Add AprilTag detection to camera pipeline
   - Use for loop closure and drift correction

3. **Full Sensor Fusion (EKF)**
   - Implement Extended Kalman Filter
   - Fuse: Depth (60%), IMU (20%), Visual (10%), AprilTags (10%)
   - See: `DEPTH_ODOMETRY_GUIDE.md` Phase 3

---

## Files Reference

### Modified Files (this fix)
- `config/slam_config.yaml` - Relaxed depth odometry constraints
- `src/slam/depth_odometry.py` - Increased error tolerance, added logging
- `src/slam/slam_engine.py` - Added hysteresis for mode switching
- `main.py` - Fixed degree symbol display bug
- `src/slam/slam_visualizer.py` - (Optional) Added odometry to Mode 3

### Related Documentation
- `DEPTH_ODOMETRY_GUIDE.md` - Overview of depth-based odometry system
- `tools/generate_apriltags.py` - AprilTag landmark generation
- `config/robot_config.yaml` - Robot motion speeds

---

## Summary

**Problem**: Depth odometry failing 50% of the time, causing unstable mode switching

**Root Cause**: Overly strict constraints (min_points, error threshold, depth range)

**Solution**: Relax constraints + add hysteresis to prevent rapid switching

**Implementation Time**: ~15-20 minutes

**Expected Result**: Stable depth odometry on 90%+ of frames, smooth autonomous navigation

---

**Status**: Ready to implement tomorrow
**Last Updated**: 2025-11-19
