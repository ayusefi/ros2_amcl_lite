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

### 3.1 Primary Metrics
- **Absolute Trajectory Error (ATE)**: RMSE of position error (meters) over entire trajectory
- **Relative Pose Error (RPE)**: Frame-to-frame pose error for local accuracy assessment
- **Angular Error**: RMSE of orientation error (radians) over time
- **Time to Converge**: Time until 95% of particles achieve error below 0.5m threshold

### 3.2 Secondary Metrics
- **Effective Sample Size (ESS)**: Measure of particle diversity (higher = better)
- **Particle Concentration**: Std deviation of particle spread
- **Recovery Time**: Time to relocalize after kidnapping (seconds)
- **Computational Efficiency**: CPU utilization and processing time per scan

### 3.3 Data Collection Format
All metrics logged to CSV files with timestamps:
- `timestamp,ground_truth_x,ground_truth_y,ground_truth_theta,estimated_x,estimated_y,estimated_theta,num_particles,ess,processing_time,dynamic_obstacles_detected`

## 4. Test Scenarios

### 4.1 Scenario A: Static Environment Navigation
- **Map**: Indoor office environment with corridors and rooms
- **Path**: Predefined square path (4m x 4m) with 4 waypoints
- **Duration**: 5 minutes per trial
- **Repetitions**: 10 trials per algorithm
- **Conditions**: No moving obstacles, consistent lighting

### 4.2 Scenario B: Dynamic Environment Navigation  
- **Map**: Same indoor office environment
- **Path**: Same predefined square path
- **Duration**: 5 minutes per trial
- **Repetitions**: 10 trials per algorithm
- **Conditions**: 2-3 simulated people crossing robot path at intervals

### 4.3 Scenario C: Kidnapped Robot Test
- **Map**: Indoor office environment
- **Path**: Initial navigation, then manual relocation
- **Procedure**: 
  1. Robot navigates normally for 2 minutes
  2. Robot teleported to random location (3 different locations)
  3. Measure recovery time and accuracy
- **Repetitions**: 10 trials per algorithm per location
- **Conditions**: Both static and dynamic environments

## 5. Data Collection and Analysis Implementation

### 5.1 Research Data Logger
- **Purpose**: Systematic collection of quantitative metrics for algorithm comparison
- **Data Format**: CSV files with timestamps, pose estimates, ground truth, and performance metrics
- **Metrics Collected**:
  - Absolute Trajectory Error (ATE) and Relative Pose Error (RPE)
  - Effective Sample Size (ESS) and particle diversity
  - Processing time and computational efficiency
  - Dynamic obstacle detection statistics

### 5.2 Automated Experiment Runner
- **Script**: `scripts/run_research_experiments.py`
- **Features**: 
  - Automated execution of multiple trials
  - Scenario-based testing (static, dynamic, kidnapped robot)
  - Algorithm comparison (baseline vs. dynamic-aware)
  - Gazebo integration for ground truth data

### 5.3 Statistical Analysis Tools
- **Script**: `scripts/analyze_research_data.py`
- **Capabilities**:
  - Statistical significance testing (t-tests, Cohen's d)
  - Performance visualization and plotting
  - Comprehensive research report generation
  - Time series analysis and convergence metrics

### 5.4 Experiment Execution
```bash
# Phase 1: Baseline experiments (switch to master branch)
git checkout master
colcon build --packages-select ros2_amcl_lite
python3 scripts/run_research_experiments.py --algorithm amcl_lite_baseline --trials 10

# Phase 2: Dynamic-aware experiments (switch to feature branch)
git checkout feature/dynamic-object-aware-sensor
colcon build --packages-select ros2_amcl_lite
python3 scripts/run_research_experiments.py --algorithm amcl_lite_dynamic --trials 10

# Phase 3: Analysis and reporting
python3 scripts/analyze_research_data.py --data-dir logs/research_data --output-dir analysis_results
```

### 5.5 Expected Outcomes
- **Quantitative Comparison**: RMSE, convergence time, particle diversity metrics
- **Statistical Validation**: p-values, effect sizes, confidence intervals
- **Visual Analysis**: Time series plots, box plots, significance heatmaps
- **Research Report**: Structured findings with conclusions and recommendations

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
