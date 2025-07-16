# ros2_amcl_lite

A lightweight 2D Particle Filter localization node for ROS 2, inspired by the Adaptive Monte Carlo Localization (AMCL) algorithm. Designed for educational and research use, it demonstrates the core principles of probabilistic localization using Lidar and Odometry.

## Purpose
- Provide a simple, readable C++ implementation of a particle filter for robot localization.
- Serve as a learning tool for understanding AMCL, particle filters, and measurement models in robotics.
- Integrate easily with ROS 2 navigation stacks and custom robots.

## How It Works
**Subscriptions:**
  - `/bcr_bot/scan` (`sensor_msgs/msg/LaserScan`): Lidar data for measurement updates.
  - `/bcr_bot/odom` (`nav_msgs/msg/Odometry`): Odometry for motion model prediction.
  - `/map` (`nav_msgs/msg/OccupancyGrid`): Static map for likelihood field computation.
  - `/initialpose` (`geometry_msgs/msg/PoseWithCovarianceStamped`): Initial pose estimate (e.g., from RViz).
**Publications:**
  - `amcl_lite_pose` (`geometry_msgs/msg/PoseStamped`): Estimated robot pose.
  - `/amcl_lite_particles` (`sensor_msgs/msg/PointCloud2`): All particles for visualization.
- **Algorithm:**
  1. **Prediction:** Applies odometry deltas (with noise) to all particles.
  2. **Update:** Uses the Likelihood Field Model to weight particles based on Lidar scan likelihoods.
  3. **Resampling:** Draws new particles proportional to their weights.
  4. **Estimation:** Publishes the mean pose and all particles.

## Usage
1. Build the package with `colcon build --packages-select ros2_amcl_lite`.
2. Source your workspace: `source install/setup.bash`.
3. Launch your navigation stack, ensuring the node is started and topics are remapped as needed.
4. Set the initial pose in RViz using the "2D Pose Estimate" tool.

## Parameters
- Most parameters are hardcoded for simplicity, but can be extended for flexibility.
- See the code and comments for details on noise, particle count, and measurement model.

## License
Apache-2.0

---

## Project Structure
- `src/ros2_amcl_lite/src/`: C++ source files
- `src/ros2_amcl_lite/include/`: C++ headers
- `src/ros2_amcl_lite/CMakeLists.txt`: Build instructions
- `src/ros2_amcl_lite/package.xml`: ROS 2 package manifest
