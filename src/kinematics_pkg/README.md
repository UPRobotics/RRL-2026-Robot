# Kinematics Package (C++)

Forward and Inverse Kinematics solver for the RRL-2026 robot arm using Denavit-Hartenberg parameters.

## Features

- **Forward Kinematics (FK)**: Calculates end-effector pose from joint angles
- **Inverse Kinematics (IK)**: Calculates joint angles for desired end-effector pose
- **Numerical IK**: Uses Jacobian-based gradient descent (handles singularities gracefully)
- **Joint limits**: Enforces physical joint constraints
- **ROS2 Integration**: Publishes continuous updates and subscribes to joint states

## Build

```bash
cd ~/RRL-2026-Robot
colcon build --packages-select kinematics_pkg
source install/setup.bash
```

## Usage

### Launch the node

```bash
ros2 launch kinematics_pkg arm_kinematics.launch.py
```

### Topics

#### Publishers
- **`/arm/end_effector_pose`** (geometry_msgs/PoseStamped)
  - Publishes the current end-effector position and orientation
  - Updated at 20 Hz from /joint_states

- **`/arm/inverse_kinematics_solution`** (sensor_msgs/JointState)
  - Publishes IK solution when requested via service

#### Subscribers
- **`/joint_states`** (sensor_msgs/JointState)
  - Reads current joint angles: Cadera, Hombro, Codo, Roll, Pitch
  - Used to compute continuous FK

### Example: Read End-Effector Pose

```bash
ros2 topic echo /arm/end_effector_pose
```

Output:
```yaml
header:
  frame_id: base_link
  stamp: ...
pose:
  position:
    x: 0.523
    y: 0.012
    z: -0.897
  orientation:
    x: 0.0
    y: 0.707
    z: 0.0
    w: 0.707
```

## Parameters

### Denavit-Hartenberg (DH) Parameters

The arm is modeled using standard DH convention:

| Joint | a (m) | alpha (rad) | d (m) | theta_offset (rad) |
|-------|-------|-------------|-------|------------------|
| Cadera | 0.000 | π/2 | 0.000 | 0.000 |
| Hombro | 0.249 | 0.0 | 0.000 | 0.000 |
| Codo | 0.254 | π | 0.000 | 0.000 |
| Roll | 0.000 | π/2 | 0.000 | 0.000 |
| Pitch | 0.000 | 0.0 | 0.000 | 0.000 |

### Joint Limits (radians)

```
Cadera:  [-3.14, 3.05]
Hombro:  [-0.52, 1.40]
Codo:    [-3.23, 0.35]
Roll:    [-3.05, 3.05]
Pitch:   [-1.66, 1.48]
```

## C++ API

### ForwardKinematics

```cpp
#include "kinematics_pkg/forward_kinematics.hpp"

auto fk = std::make_shared<kinematics::ForwardKinematics>();

std::vector<double> angles = {0.0, 0.5, -1.0, 0.0, 0.0};  // in radians
geometry_msgs::msg::Pose pose;

if (fk->computeFK(angles, pose)) {
  std::cout << "x=" << pose.position.x << ", y=" << pose.position.y
            << ", z=" << pose.position.z << std::endl;
}

// Get full transformation matrix
Eigen::Matrix4d T;
fk->getTransformationMatrix(angles, T);
```

### InverseKinematics

```cpp
#include "kinematics_pkg/inverse_kinematics.hpp"

auto fk = std::make_shared<kinematics::ForwardKinematics>();
auto ik = std::make_shared<kinematics::InverseKinematics>(fk);

// Set configuration
kinematics::InverseKinematics::IKConfig config;
config.max_iterations = 100;
config.tolerance = 0.01;  // 1cm
config.step_size = 0.01;

// Desired pose
geometry_msgs::msg::Pose desired;
desired.position.x = 0.5;
desired.position.y = 0.0;
desired.position.z = -0.3;

// Initial guess (optional)
std::vector<double> joint_angles = {0.0, 0.0, 0.0, 0.0, 0.0};

// Solve
if (ik->computeIKWithConfig(desired, joint_angles, config)) {
  std::cout << "Solution found!" << std::endl;
  for (int i = 0; i < 5; ++i) {
    std::cout << "Joint " << i << ": " << joint_angles[i] << std::endl;
  }
} else {
  std::cout << "IK failed or singular" << std::endl;
}
```

## Algorithm Details

### Forward Kinematics
Uses standard DH transformation matrix multiplication:
```
T = T1 * T2 * T3 * T4 * T5
```

Each Ti is computed as:
```
Ti = Rz(θi) * Tz(di) * Tx(ai) * Rx(αi)
```

### Inverse Kinematics
Uses **Jacobian-based numerical method**:
1. Compute current pose using FK
2. Calculate position error
3. Compute numerical Jacobian (3×5 matrix)
4. Compute pseudo-inverse using SVD
5. Update joint angles: `Δq = J† * error * step_size`
6. Repeat until convergence or max iterations

**Convergence criteria**: Position error < tolerance (default 1cm)

## Limitations & Future Work

- IK only controls position (x, y, z), not orientation
- Numerical IK may converge to local minima
- No collision avoidance
- No velocity/acceleration limits

## Dependencies

- **Eigen3**: Linear algebra
- **rclcpp**: ROS2 C++ client library
- **geometry_msgs**: ROS2 messages
- **sensor_msgs**: ROS2 messages
- **tf2**: Transform library

## Testing

```bash
# Monitor topics
ros2 topic echo /arm/end_effector_pose

# Check joint state subscription
ros2 topic list | grep arm

# Verify kinematics
# Move the arm physically and watch end_effector_pose update
```

## References

- DH Parameters: [Denavit–Hartenberg parameters](https://en.wikipedia.org/wiki/Denavit%E2%80%93Hartenberg_parameters)
- IK Method: Jacobian transpose with gradient descent
- URDF: `ensamble_flaca.xacro` in the robot description package

## Author

RRL-2026 Rescue Robot Team

## License

Apache License 2.0
