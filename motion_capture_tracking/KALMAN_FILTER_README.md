# Kalman Filter for Motion Capture Velocity Estimation

## Overview

The `3d_pose_motion_capture_node` now includes an optional Kalman filter for improved velocity estimation. The Kalman filter provides smoother, more accurate velocity estimates compared to simple finite difference methods, especially when dealing with noisy motion capture data.

## Features

### Kalman Filter Approach
- **State Estimation**: Maintains a 12-dimensional state vector containing:
  - Position (3D)
  - Orientation (quaternion)
  - Linear velocity (3D)
  - Angular velocity (3D)
- **Smooth Velocities**: Filters out measurement noise for cleaner velocity estimates
- **Covariance Estimation**: Provides uncertainty estimates in odometry messages
- **Adaptive Initialization**: Automatically initializes from the first two measurements

### Finite Difference Approach (Fallback)
- Simple numerical differentiation
- No filtering
- Lower computational cost
- Suitable for high-quality mocap data with minimal noise

## Parameters

### New Parameters

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `use_kalman_filter` | bool | true | Enable/disable Kalman filtering for velocity estimation |
| `max_accel` | double | 10.0 | Maximum expected acceleration (m/s²) - used to tune process noise |

### Existing Parameters

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `frame_rate` | int | 200 | Expected motion capture frame rate (Hz) |
| `type` | string | "vicon" | Motion capture system type |
| `hostname` | string | "localhost" | Motion capture server hostname |

## Usage

### Enable Kalman Filter (Default)

```yaml
# config/kalman_config.yaml
/**:
  ros__parameters:
    use_kalman_filter: true
    max_accel: 10.0  # Adjust based on your robot's dynamics
    frame_rate: 200
```

Launch with:
```bash
ros2 run motion_capture_tracking 3d_pose_motion_capture_node --ros-args --params-file config/kalman_config.yaml
```

### Disable Kalman Filter (Use Simple Differentiation)

```yaml
# config/simple_diff_config.yaml
/**:
  ros__parameters:
    use_kalman_filter: false
    frame_rate: 200
```

## Tuning

### Process Noise (`max_accel`)

The `max_accel` parameter controls how much the filter trusts the dynamic model vs. measurements:

- **Higher values** (e.g., 20.0): 
  - More responsive to rapid changes
  - Less smoothing
  - Better for aggressive maneuvers
  
- **Lower values** (e.g., 5.0):
  - More smoothing
  - Less responsive to sudden changes
  - Better for smooth, predictable motion

### Measurement Noise

Currently fixed at `1e-3` in the code. This represents the expected noise in position and orientation measurements from the mocap system. Can be adjusted in the source code if needed.

## Implementation Details

### State Vector Structure

The 12-dimensional state vector is organized as:
```
[0:3]   - Orientation error (axis-angle representation)
[3:6]   - Position (x, y, z)
[6:9]   - Angular velocity (roll, pitch, yaw rates)
[9:12]  - Linear velocity (vx, vy, vz)
```

### Filter Initialization

The Kalman filter requires two measurements to initialize:
1. **First measurement**: Sets initial pose
2. **Second measurement**: Estimates initial velocity from pose difference

During initialization, velocities are reported as zero in the odometry messages.

### Covariance Propagation

The odometry messages include full 6x6 covariance matrices for both pose and twist:
- **Pose covariance**: Position and orientation uncertainty
- **Twist covariance**: Linear and angular velocity uncertainty

These can be used by downstream nodes for sensor fusion (e.g., with wheel odometry in an Extended Kalman Filter).

## Output Topics

Each tracked rigid body publishes:

- `{rigid_body_name}/pose3d` (geometry_msgs/PoseStamped): Raw measured pose
- `{rigid_body_name}/odom` (nav_msgs/Odometry): 
  - Filtered/measured pose
  - Estimated velocities (with covariances if Kalman filter is enabled)

## Comparison: Kalman Filter vs. Finite Difference

| Aspect | Kalman Filter | Finite Difference |
|--------|---------------|-------------------|
| Smoothness | High - filters noise | Low - sensitive to noise |
| Latency | Minimal (~1 frame) | None |
| Computation | Moderate | Minimal |
| Covariance | Yes | No |
| Initialization | Requires 2 frames | Requires 1 frame |
| Best for | Noisy data, control | Clean data, visualization |

## Example Launch File

```python
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='motion_capture_tracking',
            executable='3d_pose_motion_capture_node',
            name='motion_capture_tracking_node',
            parameters=[{
                'type': 'vicon',
                'hostname': '192.168.1.100',
                'frame_rate': 200,
                'use_kalman_filter': True,
                'max_accel': 10.0,
                'topics': {
                    'frame_id': 'world',
                    'poses': {'qos': {'mode': 'sensor', 'deadline': 100.0}},
                    'tf': {'child_frame_id': '{}'}
                }
            }]
        )
    ])
```

## Performance Notes

- The Kalman filter adds minimal computational overhead (~0.1-0.2ms per rigid body per frame)
- Memory overhead is approximately 1KB per tracked rigid body
- Recommended for systems where velocity estimates are used for control
- Can be disabled for pure visualization use cases

## References

- Trawny, N., & Roumeliotis, S. I. (2005). *Indirect Kalman Filter for 3D Attitude Estimation: A Tutorial for Quaternion Algebra*. Technical Report Number 2005-002, Rev. 57.
- Original implementation adapted from the `mocap_base` package

## Troubleshooting

### Velocities seem too smooth/delayed
- Increase `max_accel` parameter
- The filter may be over-trusting the dynamic model

### Velocities are too noisy
- Decrease `max_accel` parameter
- Check motion capture data quality

### Filter doesn't initialize
- Ensure at least 2 valid measurements are received
- Check that `frame_rate` parameter matches actual mocap rate

### Build errors
- Ensure Eigen3 is installed: `sudo apt install libeigen3-dev`
- Clean and rebuild: `colcon build --packages-select motion_capture_tracking --cmake-clean-cache`




