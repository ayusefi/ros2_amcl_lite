#!/usr/bin/env python3
"""
AMCL Research Data Analysis Script
This script processes the collected CSV data and generates statistical comparisons
between baseline and dynamic-aware AMCL algorithms.
"""

import os
import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
import seaborn as sns
from scipy import stats
import argparse
from pathlib import Path


class AMCLDataAnalyzer:
    def __init__(self, data_dir="logs/research_data"):
        self.data_dir = Path(data_dir)
        self.results = {}
        
    def load_data(self):
        """Load all CSV files from the data directory"""
        csv_files = list(self.data_dir.glob("*.csv"))
        
        if not csv_files:
            print(f"No CSV files found in {self.data_dir}")
            return
            
        print(f"Found {len(csv_files)} data files")
        
        for csv_file in csv_files:
            # Parse filename to extract metadata
            filename = csv_file.stem
            parts = filename.split('_')
            
            if len(parts) >= 6:
                experiment_name = parts[0]
                scenario = parts[1]
                algorithm = parts[2]
                trial_info = parts[3]  # trialXX
                date = parts[4]
                time = parts[5]
                
                # Load data
                try:
                    df = pd.read_csv(csv_file)
                    
                    # Add metadata columns
                    df['experiment'] = experiment_name
                    df['scenario'] = scenario
                    df['algorithm'] = algorithm
                    df['trial'] = trial_info
                    df['date'] = date
                    df['time'] = time
                    df['filename'] = filename
                    
                    # Store in results dictionary
                    key = f"{scenario}_{algorithm}_{trial_info}"
                    self.results[key] = df
                    
                    print(f"Loaded {len(df)} data points from {csv_file.name}")
                    
                except Exception as e:
                    print(f"Error loading {csv_file.name}: {e}")
    
    def calculate_metrics(self):
        """Calculate key performance metrics for each experiment"""
        if not self.results:
            print("No data loaded. Run load_data() first.")
            return
            
        metrics_summary = []
        
        for key, df in self.results.items():
            if df.empty:
                continue
                
            # Basic info
            scenario = df['scenario'].iloc[0]
            algorithm = df['algorithm'].iloc[0]
            trial = df['trial'].iloc[0]
            
            # Calculate metrics
            metrics = {
                'scenario': scenario,
                'algorithm': algorithm,
                'trial': trial,
                'duration': df['timestamp'].max() - df['timestamp'].min(),
                'num_samples': len(df),
                
                # Position accuracy metrics
                'mean_position_error': df['position_error'].mean(),
                'std_position_error': df['position_error'].std(),
                'rmse_position': np.sqrt(df['position_error'].pow(2).mean()),
                'max_position_error': df['position_error'].max(),
                
                # Angular accuracy metrics
                'mean_angular_error': df['angular_error'].mean(),
                'std_angular_error': df['angular_error'].std(),
                'rmse_angular': np.sqrt(df['angular_error'].pow(2).mean()),
                'max_angular_error': df['angular_error'].max(),
                
                # Algorithm performance metrics
                'mean_ess': df['effective_sample_size'].mean(),
                'std_ess': df['effective_sample_size'].std(),
                'mean_particles': df['num_particles'].mean(),
                'mean_processing_time': df['processing_time_ms'].mean(),
                'std_processing_time': df['processing_time_ms'].std(),
                
                # Particle spread metrics
                'mean_particle_std': df['particle_std_dev'].mean(),
                'std_particle_std': df['particle_std_dev'].std(),
                
                # Dynamic detection metrics (if available)
                'mean_dynamic_obstacles': df['dynamic_obstacles_detected'].mean(),
                'dynamic_detection_rate': (df['dynamic_obstacles_detected'] > 0).mean(),
                
                # Convergence metrics
                'convergence_time': self._calculate_convergence_time(df),
                'final_error': df['position_error'].tail(10).mean(),  # Average of last 10 samples
            }
            
            metrics_summary.append(metrics)
        
        self.metrics_df = pd.DataFrame(metrics_summary)
        return self.metrics_df
    
    def _calculate_convergence_time(self, df, error_threshold=0.5):
        """Calculate time to convergence (when error stays below threshold)"""
        if df.empty:
            return np.nan
            
        # Find first time when error goes below threshold and stays there
        below_threshold = df['position_error'] < error_threshold
        
        # Find the first index where error is below threshold
        first_below = below_threshold.idxmax() if below_threshold.any() else None
        
        if first_below is None:
            return np.nan  # Never converged
        
        # Check if it stays below threshold for at least 10 samples
        window_size = min(10, len(df) - first_below)
        if window_size < 10:
            return np.nan
            
        stable_window = below_threshold.iloc[first_below:first_below + window_size]
        
        if stable_window.all():
            return df.iloc[first_below]['timestamp']
        else:
            return np.nan
    
    def statistical_comparison(self):
        """Perform statistical comparison between algorithms"""
        if not hasattr(self, 'metrics_df'):
            print("No metrics calculated. Run calculate_metrics() first.")
            return
            
        comparison_results = {}
        
        # Group by scenario
        scenarios = self.metrics_df['scenario'].unique()
        algorithms = self.metrics_df['algorithm'].unique()
        
        for scenario in scenarios:
            scenario_data = self.metrics_df[self.metrics_df['scenario'] == scenario]
            
            if len(algorithms) < 2:
                continue
                
            comparison_results[scenario] = {}
            
            # Key metrics to compare
            metrics_to_compare = [
                'rmse_position', 'rmse_angular', 'mean_ess', 
                'mean_processing_time', 'convergence_time', 'final_error'
            ]
            
            for metric in metrics_to_compare:
                algorithm_groups = []
                algorithm_names = []
                
                for algorithm in algorithms:
                    data = scenario_data[scenario_data['algorithm'] == algorithm][metric]
                    if not data.empty and not data.isna().all():
                        algorithm_groups.append(data.values)
                        algorithm_names.append(algorithm)
                
                if len(algorithm_groups) >= 2:
                    # Perform t-test
                    try:
                        t_stat, p_value = stats.ttest_ind(algorithm_groups[0], algorithm_groups[1])
                        
                        # Calculate effect size (Cohen's d)
                        pooled_std = np.sqrt(((len(algorithm_groups[0]) - 1) * np.var(algorithm_groups[0], ddof=1) + 
                                            (len(algorithm_groups[1]) - 1) * np.var(algorithm_groups[1], ddof=1)) / 
                                           (len(algorithm_groups[0]) + len(algorithm_groups[1]) - 2))
                        cohens_d = (np.mean(algorithm_groups[0]) - np.mean(algorithm_groups[1])) / pooled_std
                        
                        comparison_results[scenario][metric] = {
                            'algorithm_1': algorithm_names[0],
                            'algorithm_2': algorithm_names[1],
                            'mean_1': np.mean(algorithm_groups[0]),
                            'mean_2': np.mean(algorithm_groups[1]),
                            'std_1': np.std(algorithm_groups[0]),
                            'std_2': np.std(algorithm_groups[1]),
                            't_statistic': t_stat,
                            'p_value': p_value,
                            'cohens_d': cohens_d,
                            'significant': p_value < 0.05
                        }
                    except Exception as e:
                        print(f"Error in statistical test for {scenario}, {metric}: {e}")
        
        self.comparison_results = comparison_results
        return comparison_results
    
    def generate_plots(self, output_dir="plots"):
        """Generate visualization plots"""
        if not hasattr(self, 'metrics_df'):
            print("No metrics calculated. Run calculate_metrics() first.")
            return
            
        output_path = Path(output_dir)
        output_path.mkdir(exist_ok=True)
        
        # Set up plotting style
        plt.style.use('seaborn-v0_8')
        sns.set_palette("husl")
        
        # 1. Position Error Comparison
        plt.figure(figsize=(12, 8))
        
        # Box plot of position errors by scenario and algorithm
        plt.subplot(2, 2, 1)
        sns.boxplot(data=self.metrics_df, x='scenario', y='rmse_position', hue='algorithm')
        plt.title('Position RMSE by Scenario and Algorithm')
        plt.ylabel('Position RMSE (m)')
        plt.xticks(rotation=45)
        
        # Angular error comparison
        plt.subplot(2, 2, 2)
        sns.boxplot(data=self.metrics_df, x='scenario', y='rmse_angular', hue='algorithm')
        plt.title('Angular RMSE by Scenario and Algorithm')
        plt.ylabel('Angular RMSE (rad)')
        plt.xticks(rotation=45)
        
        # Effective Sample Size
        plt.subplot(2, 2, 3)
        sns.boxplot(data=self.metrics_df, x='scenario', y='mean_ess', hue='algorithm')
        plt.title('Effective Sample Size by Scenario and Algorithm')
        plt.ylabel('Mean ESS')
        plt.xticks(rotation=45)
        
        # Processing Time
        plt.subplot(2, 2, 4)
        sns.boxplot(data=self.metrics_df, x='scenario', y='mean_processing_time', hue='algorithm')
        plt.title('Processing Time by Scenario and Algorithm')
        plt.ylabel('Processing Time (ms)')
        plt.xticks(rotation=45)
        
        plt.tight_layout()
        plt.savefig(output_path / 'algorithm_comparison.png', dpi=300, bbox_inches='tight')
        plt.close()
        
        # 2. Time series plots for each trial
        self._plot_time_series(output_path)
        
        # 3. Statistical significance heatmap
        self._plot_statistical_heatmap(output_path)
        
        print(f"Plots saved to {output_path}")
    
    def _plot_time_series(self, output_path):
        """Plot time series data for each trial"""
        for key, df in self.results.items():
            if df.empty:
                continue
                
            scenario = df['scenario'].iloc[0]
            algorithm = df['algorithm'].iloc[0]
            trial = df['trial'].iloc[0]
            
            plt.figure(figsize=(15, 10))
            
            # Position error over time
            plt.subplot(2, 3, 1)
            plt.plot(df['timestamp'], df['position_error'])
            plt.title(f'Position Error - {scenario} - {algorithm} - {trial}')
            plt.xlabel('Time (s)')
            plt.ylabel('Position Error (m)')
            plt.grid(True)
            
            # Angular error over time
            plt.subplot(2, 3, 2)
            plt.plot(df['timestamp'], df['angular_error'])
            plt.title(f'Angular Error - {scenario} - {algorithm} - {trial}')
            plt.xlabel('Time (s)')
            plt.ylabel('Angular Error (rad)')
            plt.grid(True)
            
            # Effective Sample Size over time
            plt.subplot(2, 3, 3)
            plt.plot(df['timestamp'], df['effective_sample_size'])
            plt.title(f'ESS - {scenario} - {algorithm} - {trial}')
            plt.xlabel('Time (s)')
            plt.ylabel('ESS')
            plt.grid(True)
            
            # Processing time over time
            plt.subplot(2, 3, 4)
            plt.plot(df['timestamp'], df['processing_time_ms'])
            plt.title(f'Processing Time - {scenario} - {algorithm} - {trial}')
            plt.xlabel('Time (s)')
            plt.ylabel('Processing Time (ms)')
            plt.grid(True)
            
            # Particle spread over time
            plt.subplot(2, 3, 5)
            plt.plot(df['timestamp'], df['particle_std_dev'])
            plt.title(f'Particle Spread - {scenario} - {algorithm} - {trial}')
            plt.xlabel('Time (s)')
            plt.ylabel('Particle Std Dev (m)')
            plt.grid(True)
            
            # Dynamic obstacles detected over time
            plt.subplot(2, 3, 6)
            plt.plot(df['timestamp'], df['dynamic_obstacles_detected'])
            plt.title(f'Dynamic Obstacles - {scenario} - {algorithm} - {trial}')
            plt.xlabel('Time (s)')
            plt.ylabel('Obstacles Detected')
            plt.grid(True)
            
            plt.tight_layout()
            plt.savefig(output_path / f'timeseries_{key}.png', dpi=300, bbox_inches='tight')
            plt.close()
    
    def _plot_statistical_heatmap(self, output_path):
        """Plot statistical significance heatmap"""
        if not hasattr(self, 'comparison_results'):
            self.statistical_comparison()
            
        # Create significance matrix
        scenarios = list(self.comparison_results.keys())
        metrics = ['rmse_position', 'rmse_angular', 'mean_ess', 'mean_processing_time']
        
        significance_matrix = np.zeros((len(scenarios), len(metrics)))
        
        for i, scenario in enumerate(scenarios):
            for j, metric in enumerate(metrics):
                if metric in self.comparison_results[scenario]:
                    significance_matrix[i, j] = 1 if self.comparison_results[scenario][metric]['significant'] else 0
        
        plt.figure(figsize=(10, 6))
        sns.heatmap(significance_matrix, 
                   xticklabels=metrics, 
                   yticklabels=scenarios,
                   annot=True, 
                   cmap='RdYlBu_r',
                   cbar_kws={'label': 'Statistically Significant (p < 0.05)'})
        plt.title('Statistical Significance of Algorithm Differences')
        plt.tight_layout()
        plt.savefig(output_path / 'statistical_significance.png', dpi=300, bbox_inches='tight')
        plt.close()
    
    def generate_report(self, output_file="research_report.txt"):
        """Generate comprehensive research report"""
        if not hasattr(self, 'metrics_df'):
            print("No metrics calculated. Run calculate_metrics() first.")
            return
            
        with open(output_file, 'w') as f:
            f.write("AMCL Research Experiment Report\n")
            f.write("=" * 50 + "\n\n")
            
            # Summary statistics
            f.write("Summary Statistics\n")
            f.write("-" * 20 + "\n")
            f.write(f"Total experiments: {len(self.metrics_df)}\n")
            f.write(f"Scenarios: {', '.join(self.metrics_df['scenario'].unique())}\n")
            f.write(f"Algorithms: {', '.join(self.metrics_df['algorithm'].unique())}\n")
            f.write(f"Trials per scenario-algorithm: {self.metrics_df.groupby(['scenario', 'algorithm']).size().min()}\n\n")
            
            # Mean performance by algorithm
            f.write("Mean Performance by Algorithm\n")
            f.write("-" * 30 + "\n")
            algorithm_summary = self.metrics_df.groupby('algorithm').agg({
                'rmse_position': ['mean', 'std'],
                'rmse_angular': ['mean', 'std'],
                'mean_ess': ['mean', 'std'],
                'mean_processing_time': ['mean', 'std'],
                'convergence_time': ['mean', 'std']
            }).round(4)
            f.write(algorithm_summary.to_string() + "\n\n")
            
            # Statistical comparison results
            if hasattr(self, 'comparison_results'):
                f.write("Statistical Comparison Results\n")
                f.write("-" * 35 + "\n")
                
                for scenario, results in self.comparison_results.items():
                    f.write(f"\n{scenario.upper()}:\n")
                    for metric, stats in results.items():
                        f.write(f"  {metric}:\n")
                        f.write(f"    {stats['algorithm_1']}: {stats['mean_1']:.4f} ± {stats['std_1']:.4f}\n")
                        f.write(f"    {stats['algorithm_2']}: {stats['mean_2']:.4f} ± {stats['std_2']:.4f}\n")
                        f.write(f"    p-value: {stats['p_value']:.4f}\n")
                        f.write(f"    Cohen's d: {stats['cohens_d']:.4f}\n")
                        f.write(f"    Significant: {'Yes' if stats['significant'] else 'No'}\n\n")
            
            # Conclusions
            f.write("Key Findings\n")
            f.write("-" * 15 + "\n")
            
            # Best performing algorithm overall
            best_pos_algo = self.metrics_df.groupby('algorithm')['rmse_position'].mean().idxmin()
            best_ang_algo = self.metrics_df.groupby('algorithm')['rmse_angular'].mean().idxmin()
            best_ess_algo = self.metrics_df.groupby('algorithm')['mean_ess'].mean().idxmax()
            
            f.write(f"Best position accuracy: {best_pos_algo}\n")
            f.write(f"Best angular accuracy: {best_ang_algo}\n")
            f.write(f"Best particle diversity (ESS): {best_ess_algo}\n")
        
        print(f"Report saved to {output_file}")


def main():
    parser = argparse.ArgumentParser(description='AMCL Research Data Analysis')
    parser.add_argument('--data-dir', type=str, default='logs/research_data',
                      help='Directory containing CSV data files')
    parser.add_argument('--output-dir', type=str, default='plots',
                      help='Directory to save plots')
    parser.add_argument('--report', type=str, default='research_report.txt',
                      help='Output file for research report')
    
    args = parser.parse_args()
    
    # Create analyzer
    analyzer = AMCLDataAnalyzer(args.data_dir)
    
    # Load and analyze data
    analyzer.load_data()
    
    if not analyzer.results:
        print("No data found. Please run experiments first.")
        return
    
    # Calculate metrics
    metrics_df = analyzer.calculate_metrics()
    print(f"\nCalculated metrics for {len(metrics_df)} experiments")
    
    # Perform statistical comparison
    comparison_results = analyzer.statistical_comparison()
    
    # Generate plots
    analyzer.generate_plots(args.output_dir)
    
    # Generate report
    analyzer.generate_report(args.report)
    
    print("\nAnalysis complete!")
    print(f"- Plots saved to: {args.output_dir}")
    print(f"- Report saved to: {args.report}")


if __name__ == '__main__':
    main()
