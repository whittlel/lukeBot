# SLAM Debug - Next Steps

## Current Status (2025-11-20)

### What's Working ✓
- Depth odometry ICP: 0.022-0.065m errors with 2600-3200 points
- Robot marker moves on map correctly
- Translation (x, y) working properly
- Map persistence (obstacles don't disappear)
- Performance: ~4-6 FPS on Jetson Nano

### Known Issue ⚠️
**Occupancy map appears to be camera-relative instead of world-relative**
- Map looks like "overwritten" camera view
- Triangle pattern persists
- Map doesn't seem to rotate with robot heading changes
- Black blob near robot removed (was robot chassis < 0.3m)

## Suspected Root Cause

The pose rotation (theta) may not be accumulating correctly, causing map points to be placed in camera frame instead of world frame.

**Evidence:**
- Delta rotations show: `dtheta=6.1deg`, `dtheta=2.4deg` (moderate)
- Delta translations show: `dx=0.005m`, `dy=0.004m` (very small, mm range)
- Fast rotation might cause ICP to lose tracking

## Debug Steps for Tomorrow

### 1. Verify Pose Accumulation
Check if theta is actually accumulating:

```python
# In main.py or slam_engine.py, add logging:
print(f"[POSE] Cumulative: x={pose.x:.3f}, y={pose.y:.3f}, theta={np.degrees(pose.theta):.1f}deg")
```

**Expected:** Theta should increase/decrease continuously as robot rotates
**If broken:** Theta stays near 0 or jumps erratically

### 2. Check update_pose() in depth_odometry.py

Lines 326-334 show the pose update logic:
```python
def update_pose(self, delta_pose: RobotPose) -> RobotPose:
    cos_theta = np.cos(self.prev_pose.theta)
    sin_theta = np.sin(self.prev_pose.theta)

    self.prev_pose.x += delta_pose.x * cos_theta - delta_pose.y * sin_theta
    self.prev_pose.y += delta_pose.x * sin_theta + delta_pose.y * cos_theta
    self.prev_pose.theta += delta_pose.theta
```

**Verify:**
- Is `self.prev_pose.theta` actually updating?
- Print before/after values to confirm accumulation

### 3. Slow Down Rotation for Testing

If fast rotation is the issue, try:
- Reducing turn speed in robot_config.yaml: `default_turn_speed: 15 → 10`
- Or manually turn robot VERY slowly
- Check if map builds correctly with slow rotation

### 4. Check Map Rotation Application

In map_builder.py lines 164-165, verify rotation is applied:
```python
cos_theta = np.cos(pose.theta)
sin_theta = np.sin(pose.theta)
```

Add logging:
```python
if self._map_update_count % 20 == 1:
    print(f"[MAP] Applying rotation: theta={np.degrees(pose.theta):.1f}deg, cos={cos_theta:.3f}, sin={sin_theta:.3f}")
```

**Expected:** theta should vary as robot rotates
**If broken:** theta stays constant (likely 0)

### 5. Alternative: Check if ICP Rotation is Wrong

The rotation extraction (depth_odometry.py:272) might be incorrect:
```python
dtheta = np.arctan2(R[1, 0], R[0, 0])
```

This extracts yaw from rotation matrix. Might need different indices for camera-to-robot transform.

## Quick Wins to Try First

1. **Add cumulative pose logging** - see if theta accumulates
2. **Slow rotation test** - manually rotate VERY slowly, see if map improves
3. **Check Mode 3 visualization** - does the pose text show theta changing?

## Files Modified Today

- `src/slam/depth_odometry.py` - ICP fixes, translation axes, logging
- `src/slam/slam_engine.py` - String matching bug, hysteresis
- `src/slam/map_builder.py` - Grid coordinates, persistence, raycasting
- `src/slam/slam_visualizer.py` - NEW: Multi-panel debug view
- `src/slam/map_overlay_visualizer.py` - NEW: Top-down map view
- `config/slam_config.yaml` - Performance tuning

## Commit

All progress saved in commit: `6628724 - WIP: Fix depth odometry and occupancy mapping`
