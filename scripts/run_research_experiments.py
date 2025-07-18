#!/usr/bin/env python3
"""
Automated Research Data Collection Script for AMCL Comparison
This script automates the process of collecting data for both baseline and dynamic-aware AMCL.
"""

import os
import sys
import time
import subprocess
import argparse
from datetime import datetime


class AMCLResearchRunner:
    def __init__(self):
        self.scenarios = {
            'static_environment': {
                'description': 'Robot navigates in static environment',
                'duration': 300,  # 5 minutes
                'waypoints': [
                    [2.0, 0.0], [2.0, 2.0], [0.0, 2.0], [0.0, 0.0]  # Square path
                ]
            },
            'dynamic_environment': {
                'description': 'Robot navigates with dynamic obstacles',
                'duration': 300,  # 5 minutes
                'waypoints': [
                    [2.0, 0.0], [2.0, 2.0], [0.0, 2.0], [0.0, 0.0]  # Same square path
                ]
            },
            'kidnapped_robot': {
                'description': 'Robot recovery after kidnapping',
                'duration': 180,  # 3 minutes
                'kidnap_positions': [
                    [1.5, 1.5, 0.0], [-1.5, 1.5, 1.57], [1.5, -1.5, 3.14]
                ]
            }
        }
        
        self.algorithms = {
            'amcl_lite_baseline': {
                'enable_dynamic_detection': False,
                'description': 'Standard AMCL without dynamic detection'
            },
            'amcl_lite_dynamic': {
                'enable_dynamic_detection': True,
                'description': 'AMCL with dynamic-object-aware sensor model'
            }
        }
    
    def run_experiment(self, scenario_name, algorithm_name, trial_number):
        """Run a single experiment trial"""
        print(f"\n=== Running Experiment ===")
        print(f"Scenario: {scenario_name}")
        print(f"Algorithm: {algorithm_name}")
        print(f"Trial: {trial_number}")
        print(f"Time: {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}")
        
        # Build launch command
        launch_cmd = [
            'ros2', 'launch', 'ros2_amcl_lite', 'research_experiment.launch.py',
            f'scenario_name:={scenario_name}',
            f'algorithm_name:={algorithm_name}',
            f'trial_number:={trial_number}',
            f'enable_dynamic_detection:={str(self.algorithms[algorithm_name]["enable_dynamic_detection"]).lower()}'
        ]
        
        print(f"Launch command: {' '.join(launch_cmd)}")
        
        # Start the experiment
        process = subprocess.Popen(launch_cmd, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
        
        # Wait for initialization
        print("Waiting for system initialization...")
        time.sleep(10)
        
        # Run the scenario
        scenario_config = self.scenarios[scenario_name]
        
        if scenario_name == 'kidnapped_robot':
            self._run_kidnapped_robot_scenario(scenario_config, trial_number)
        else:
            self._run_navigation_scenario(scenario_config, trial_number)
        
        # Stop the experiment
        print("Stopping experiment...")
        process.terminate()
        process.wait()
        
        print(f"Experiment completed for {scenario_name}, {algorithm_name}, trial {trial_number}")
    
    def _run_navigation_scenario(self, scenario_config, trial_number):
        """Run navigation scenario with waypoints"""
        print(f"Running navigation scenario for {scenario_config['duration']} seconds...")
        
        # Send navigation goals
        waypoints = scenario_config['waypoints']
        waypoint_duration = scenario_config['duration'] / len(waypoints)
        
        for i, waypoint in enumerate(waypoints):
            print(f"Sending goal {i+1}/{len(waypoints)}: x={waypoint[0]}, y={waypoint[1]}")
            
            # Send goal using ROS2 action client
            goal_cmd = [
                'ros2', 'topic', 'pub', '--once', '/goal_pose',
                'geometry_msgs/msg/PoseStamped',
                f'{{"header": {{"frame_id": "map"}}, "pose": {{"position": {{"x": {waypoint[0]}, "y": {waypoint[1]}, "z": 0.0}}, "orientation": {{"w": 1.0}}}}}}'
            ]
            
            subprocess.run(goal_cmd, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
            
            # Wait for waypoint duration
            time.sleep(waypoint_duration)
    
    def _run_kidnapped_robot_scenario(self, scenario_config, trial_number):
        """Run kidnapped robot scenario"""
        print("Running kidnapped robot scenario...")
        
        # Normal navigation for 2 minutes
        print("Phase 1: Normal navigation (2 minutes)")
        goal_cmd = [
            'ros2', 'topic', 'pub', '--once', '/goal_pose',
            'geometry_msgs/msg/PoseStamped',
            '{"header": {"frame_id": "map"}, "pose": {"position": {"x": 2.0, "y": 0.0, "z": 0.0}, "orientation": {"w": 1.0}}}'
        ]
        subprocess.run(goal_cmd, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        time.sleep(120)
        
        # Kidnap robot to random position
        kidnap_positions = scenario_config['kidnap_positions']
        kidnap_pos = kidnap_positions[trial_number % len(kidnap_positions)]
        
        print(f"Phase 2: Kidnapping robot to x={kidnap_pos[0]}, y={kidnap_pos[1]}, theta={kidnap_pos[2]}")
        
        # Use Gazebo service to teleport robot
        teleport_cmd = [
            'ros2', 'service', 'call', '/gazebo/set_entity_state',
            'gazebo_msgs/srv/SetEntityState',
            f'{{"state": {{"name": "bcr_bot", "pose": {{"position": {{"x": {kidnap_pos[0]}, "y": {kidnap_pos[1]}, "z": 0.0}}, "orientation": {{"z": {kidnap_pos[2]}, "w": 1.0}}}}}}}}'
        ]
        
        subprocess.run(teleport_cmd, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        
        # Wait for recovery
        print("Phase 3: Recovery phase (1 minute)")
        time.sleep(60)
    
    def run_full_experiment_suite(self, num_trials=3):
        """Run complete experiment suite"""
        print("=== AMCL Research Experiment Suite ===")
        print(f"Running {num_trials} trials per scenario-algorithm combination")
        
        total_experiments = len(self.scenarios) * len(self.algorithms) * num_trials
        current_experiment = 0
        
        for scenario_name in self.scenarios:
            for algorithm_name in self.algorithms:
                for trial in range(1, num_trials + 1):
                    current_experiment += 1
                    print(f"\n--- Experiment {current_experiment}/{total_experiments} ---")
                    
                    try:
                        self.run_experiment(scenario_name, algorithm_name, trial)
                        print(f"✓ Completed: {scenario_name}, {algorithm_name}, trial {trial}")
                    except Exception as e:
                        print(f"✗ Failed: {scenario_name}, {algorithm_name}, trial {trial}")
                        print(f"Error: {e}")
                    
                    # Wait between experiments
                    if current_experiment < total_experiments:
                        print("Waiting 30 seconds before next experiment...")
                        time.sleep(30)
        
        print("\n=== Experiment Suite Complete ===")
        print("Data files are located in: logs/research_data/")


def main():
    parser = argparse.ArgumentParser(description='AMCL Research Data Collection')
    parser.add_argument('--scenario', type=str, default='all',
                      help='Scenario to run (static_environment, dynamic_environment, kidnapped_robot, all)')
    parser.add_argument('--algorithm', type=str, default='all',
                      help='Algorithm to test (amcl_lite_baseline, amcl_lite_dynamic, all)')
    parser.add_argument('--trials', type=int, default=3,
                      help='Number of trials per scenario-algorithm combination')
    parser.add_argument('--single', action='store_true',
                      help='Run single experiment with specified scenario and algorithm')
    
    args = parser.parse_args()
    
    runner = AMCLResearchRunner()
    
    if args.single:
        if args.scenario == 'all' or args.algorithm == 'all':
            print("Error: --single requires specific scenario and algorithm")
            sys.exit(1)
        runner.run_experiment(args.scenario, args.algorithm, 1)
    else:
        runner.run_full_experiment_suite(args.trials)


if __name__ == '__main__':
    main()
