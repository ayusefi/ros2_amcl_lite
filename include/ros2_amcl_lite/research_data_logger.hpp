#pragma once
#include <fstream>
#include <string>
#include <chrono>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <rclcpp/rclcpp.hpp>

struct ResearchMetrics {
    double timestamp;
    // Ground truth pose (from Gazebo)
    double gt_x, gt_y, gt_theta;
    // Estimated pose (from AMCL)
    double est_x, est_y, est_theta;
    // Algorithm metrics
    int num_particles;
    double effective_sample_size;
    double processing_time_ms;
    int dynamic_obstacles_detected;
    // Error metrics (calculated)
    double position_error;
    double angular_error;
    double particle_std_dev;
};

class ResearchDataLogger {
public:
    ResearchDataLogger(const std::string& experiment_name = "amcl_experiment");
    ~ResearchDataLogger();
    
    /**
     * @brief Initialize logging session with experiment metadata
     * @param scenario_name Name of the test scenario
     * @param algorithm_name Name of the algorithm being tested
     * @param trial_number Current trial number
     */
    void initializeSession(const std::string& scenario_name, 
                          const std::string& algorithm_name, 
                          int trial_number);
    
    /**
     * @brief Log a single data point
     * @param metrics Research metrics to log
     */
    void logDataPoint(const ResearchMetrics& metrics);
    
    /**
     * @brief Set ground truth pose (from Gazebo /model_states)
     * @param pose Ground truth pose
     */
    void setGroundTruthPose(const geometry_msgs::msg::PoseStamped& pose);
    
    /**
     * @brief Set estimated pose (from AMCL)
     * @param pose Estimated pose
     */
    void setEstimatedPose(const geometry_msgs::msg::PoseStamped& pose);
    
    /**
     * @brief Update algorithm metrics
     * @param num_particles Current number of particles
     * @param ess Effective sample size
     * @param processing_time Processing time in milliseconds
     * @param dynamic_obstacles Number of dynamic obstacles detected
     * @param particle_std_dev Standard deviation of particle spread
     */
    void updateAlgorithmMetrics(int num_particles, double ess, 
                               double processing_time, int dynamic_obstacles,
                               double particle_std_dev);
    
    /**
     * @brief Calculate and log current metrics
     */
    void logCurrentState();
    
    /**
     * @brief Enable/disable logging
     * @param enable True to enable logging
     */
    void setLoggingEnabled(bool enable) { logging_enabled_ = enable; }
    
    /**
     * @brief Check if logging is enabled
     * @return True if logging is enabled
     */
    bool isLoggingEnabled() const { return logging_enabled_; }
    
    /**
     * @brief Get current log file path
     * @return Path to current log file
     */
    std::string getCurrentLogFile() const { return current_log_file_; }
    
    /**
     * @brief Finalize current logging session
     */
    void finalizeSession();
    
private:
    /**
     * @brief Calculate position error between ground truth and estimate
     * @return Position error in meters
     */
    double calculatePositionError() const;
    
    /**
     * @brief Calculate angular error between ground truth and estimate
     * @return Angular error in radians
     */
    double calculateAngularError() const;
    
    /**
     * @brief Normalize angle to [-pi, pi]
     * @param angle Angle in radians
     * @return Normalized angle
     */
    double normalizeAngle(double angle) const;
    
    /**
     * @brief Extract yaw from quaternion
     * @param orientation Quaternion orientation
     * @return Yaw angle in radians
     */
    double extractYaw(const geometry_msgs::msg::Quaternion& orientation) const;
    
    /**
     * @brief Write CSV header
     */
    void writeCSVHeader();
    
    // File management
    std::ofstream log_file_;
    std::string experiment_name_;
    std::string current_log_file_;
    bool logging_enabled_;
    
    // Current state
    geometry_msgs::msg::PoseStamped ground_truth_pose_;
    geometry_msgs::msg::PoseStamped estimated_pose_;
    bool gt_pose_valid_;
    bool est_pose_valid_;
    
    // Algorithm metrics
    int num_particles_;
    double effective_sample_size_;
    double processing_time_ms_;
    int dynamic_obstacles_detected_;
    double particle_std_dev_;
    
    // Timing
    std::chrono::high_resolution_clock::time_point session_start_time_;
    std::chrono::high_resolution_clock::time_point last_log_time_;
    
    // Session metadata
    std::string scenario_name_;
    std::string algorithm_name_;
    int trial_number_;
};
