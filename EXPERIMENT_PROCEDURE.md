# AMCL Research Experiment Procedure

## Overview
This document outlines the step-by-step procedure for conducting rigorous experiments to compare the baseline AMCL with the dynamic-object-aware AMCL implementation.

## Prerequisites

### 1. Build the System
```bash
cd /home/abdullah/workspaces/career_sprint/ros2_ws
colcon build --packages-select ros2_amcl_lite
source install/setup.bash
```

### 2. Install Dependencies
```bash
# Install required Python packages for analysis
pip3 install pandas numpy matplotlib seaborn scipy
```

### 3. Prepare Gazebo Environment
- Ensure Gazebo is installed and configured
- Have the BCR bot model and simulation environment ready
- Map service should be available at `/map` topic

## Experiment Design

### Test Scenarios

#### Scenario A: Static Environment Navigation
- **Objective**: Evaluate performance in controlled static environment
- **Duration**: 5 minutes per trial
- **Path**: Square path (4m x 4m) with 4 waypoints
- **Conditions**: No moving obstacles, consistent environment

#### Scenario B: Dynamic Environment Navigation
- **Objective**: Evaluate performance with moving obstacles
- **Duration**: 5 minutes per trial
- **Path**: Same square path as Scenario A
- **Conditions**: 2-3 simulated moving people/obstacles

#### Scenario C: Kidnapped Robot Test
- **Objective**: Evaluate recovery after unexpected relocation
- **Duration**: 3 minutes per trial
- **Procedure**: 
  1. Normal navigation (2 minutes)
  2. Manual robot relocation
  3. Recovery measurement (1 minute)

### Algorithm Variants

#### Baseline AMCL (`amcl_lite_baseline`)
- Standard particle filter implementation
- No dynamic object detection
- `enable_dynamic_detection: false`

#### Dynamic-Aware AMCL (`amcl_lite_dynamic`)
- Enhanced particle filter with dynamic object detection
- Down-weights laser beams hitting moving obstacles
- `enable_dynamic_detection: true`

## Data Collection Procedure

### Phase 1: Baseline Experiments

1. **Switch to master branch** (baseline implementation):
   ```bash
   cd /home/abdullah/workspaces/career_sprint/ros2_ws/src/ros2_amcl_lite
   git checkout master
   cd /home/abdullah/workspaces/career_sprint/ros2_ws
   colcon build --packages-select ros2_amcl_lite
   source install/setup.bash
   ```

2. **Run baseline experiments**:
   ```bash
   # Run all scenarios with baseline algorithm
   python3 src/ros2_amcl_lite/scripts/run_research_experiments.py \
     --algorithm amcl_lite_baseline \
     --trials 10
   ```

3. **Verify data collection**:
   ```bash
   ls -la logs/research_data/
   # Should see CSV files with naming pattern: amcl_research_*_amcl_lite_baseline_*
   ```

### Phase 2: Dynamic-Aware Experiments

1. **Switch to feature branch**:
   ```bash
   cd /home/abdullah/workspaces/career_sprint/ros2_ws/src/ros2_amcl_lite
   git checkout feature/dynamic-object-aware-sensor
   cd /home/abdullah/workspaces/career_sprint/ros2_ws
   colcon build --packages-select ros2_amcl_lite
   source install/setup.bash
   ```

2. **Run dynamic-aware experiments**:
   ```bash
   # Run all scenarios with dynamic-aware algorithm
   python3 src/ros2_amcl_lite/scripts/run_research_experiments.py \
     --algorithm amcl_lite_dynamic \
     --trials 10
   ```

3. **Verify data collection**:
   ```bash
   ls -la logs/research_data/
   # Should see additional CSV files with: amcl_research_*_amcl_lite_dynamic_*
   ```

### Phase 3: Data Analysis

1. **Analyze collected data**:
   ```bash
   python3 src/ros2_amcl_lite/scripts/analyze_research_data.py \
     --data-dir logs/research_data \
     --output-dir analysis_results
   ```

2. **Review results**:
   ```bash
   # Check generated plots
   ls -la analysis_results/
   
   # Read the research report
   cat research_report.txt
   ```

## Manual Experiment Execution

### Option 1: Single Experiment
```bash
# Run single experiment manually
python3 src/ros2_amcl_lite/scripts/run_research_experiments.py \
  --single \
  --scenario static_environment \
  --algorithm amcl_lite_baseline \
  --trials 1
```

### Option 2: Custom Launch
```bash
# Start experiment with custom parameters
ros2 launch ros2_amcl_lite research_experiment.launch.py \
  scenario_name:=static_environment \
  algorithm_name:=amcl_lite_baseline \
  trial_number:=1 \
  enable_dynamic_detection:=false
```

## Data Quality Checks

### During Experiments
1. **Monitor log files**: Check that CSV files are being created and populated
2. **Verify topics**: Ensure all required topics are active:
   ```bash
   ros2 topic list | grep -E "(scan|odom|model_states|amcl_lite_pose)"
   ```
3. **Check ground truth**: Verify Gazebo model_states are being published:
   ```bash
   ros2 topic echo /model_states --once
   ```

### Post-Experiment
1. **Data completeness**: Verify all expected CSV files exist
2. **Data integrity**: Check for missing timestamps or corrupted data
3. **Metadata validation**: Ensure proper scenario/algorithm labeling

## Expected Metrics

### Primary Metrics
- **Position RMSE**: Root Mean Square Error of position estimates
- **Angular RMSE**: Root Mean Square Error of orientation estimates
- **Convergence Time**: Time until error stabilizes below threshold
- **Recovery Time**: Time to relocalize after kidnapping

### Secondary Metrics
- **Effective Sample Size (ESS)**: Particle diversity measure
- **Processing Time**: Computational efficiency
- **Particle Concentration**: Localization confidence indicator

## Troubleshooting

### Common Issues

1. **No ground truth data**:
   - Ensure Gazebo is running and `/model_states` topic is active
   - Check robot model name in ground truth callback

2. **Missing CSV files**:
   - Verify `enable_research_logging` parameter is set to `true`
   - Check write permissions in `logs/research_data/` directory

3. **Incomplete experiments**:
   - Monitor system resources (CPU, memory)
   - Check for ROS2 node crashes in terminal output

4. **Analysis errors**:
   - Verify Python dependencies are installed
   - Check CSV file format and column names

### Performance Optimization
- **Reduce particle count** if experiments are too slow
- **Increase logging frequency** for higher resolution data
- **Monitor disk space** for large datasets

## Expected Timeline
- **Setup and validation**: 1-2 hours
- **Baseline experiments**: 4-6 hours (30 experiments × 5-10 minutes each)
- **Dynamic-aware experiments**: 4-6 hours
- **Data analysis**: 1-2 hours
- **Report generation**: 1-2 hours

**Total estimated time**: 10-16 hours over 2-3 days

## Quality Assurance
- Run each scenario-algorithm combination at least 10 times
- Verify statistical significance with p-value < 0.05
- Calculate effect sizes (Cohen's d) for practical significance
- Cross-validate results with different random seeds

## Deliverables
1. **Raw data**: CSV files with timestamped measurements
2. **Analysis plots**: Comparison visualizations
3. **Statistical report**: Quantitative comparison results
4. **Research summary**: Key findings and conclusions
