# Research Plan

## 1. Hypothesis
Incorporating a dynamic-object-aware sensor model that discounts unexpected Lidar returns from moving obstacles will improve AMCL’s localization accuracy and reduce particle degeneracy in dynamic environments.

## 2. Experimental Setup

### 2.1 Baseline
- Standard ROS2 AMCL (particle filter with fixed sensor model and global resampling).
- Default parameters (particle count, resample threshold, motion & sensor noise).

### 2.2 Proposed Method
- AMCL-Lite with a dynamic-object-aware observation model:
  - Identify and down-weight Lidar beams hitting moving obstacles.
  - Maintain standard motion model and resampling strategy.

## 3. Evaluation Metrics
- Pose RMSE (translation in meters, rotation in radians) over time.
- Time to converge: time until 95% of runs achieve error below threshold.
- Effective Sample Size (ESS) to quantify particle diversity.
- CPU utilization and average particle count.

## 4. Test Scenarios
1. **Static Environment**
   - Controlled indoor map with no moving obstacles.
2. **Dynamic Environment**
   - Same map with 2–3 simulated moving people crossing corridors.
3. **Kidnapped-Robot Trials**
   - Robot is teleported to unknown poses mid-run; measure recovery time.

## 5. Procedure
1. For each scenario, run 10 trials with the baseline and 10 with the proposed method.
2. Record live pose estimates and ground-truth from Gazebo / real-robot logs.
3. Compute metrics offline using analysis scripts (e.g., in `scripts/analysis/`).
4. Perform statistical comparison (paired t-test) on RMSE and recovery times.

## 6. Timeline
- Week 1: Implement dynamic-object detector and weight modifier.
- Week 2: Integrate into AMCL-Lite and validate on static map.
- Week 3: Run dynamic-environment experiments.
- Week 4: Conduct kidnapped-robot trials and analyze results.
- Week 5: Write up findings and refine parameters.

---

# Package Overview

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
