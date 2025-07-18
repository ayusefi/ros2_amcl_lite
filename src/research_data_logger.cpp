#include "ros2_amcl_lite/research_data_logger.hpp"
#include <filesystem>
#include <iomanip>
#include <sstream>
#include <cmath>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>

ResearchDataLogger::ResearchDataLogger(const std::string& experiment_name)
    : experiment_name_(experiment_name), logging_enabled_(false), 
      gt_pose_valid_(false), est_pose_valid_(false),
      num_particles_(0), effective_sample_size_(0.0), processing_time_ms_(0.0),
      dynamic_obstacles_detected_(0), particle_std_dev_(0.0), trial_number_(0) {
    
    // Create logs directory if it doesn't exist
    std::filesystem::create_directories("logs/research_data");
}

ResearchDataLogger::~ResearchDataLogger() {
    if (log_file_.is_open()) {
        log_file_.close();
    }
}

void ResearchDataLogger::initializeSession(const std::string& scenario_name, 
                                         const std::string& algorithm_name, 
                                         int trial_number) {
    scenario_name_ = scenario_name;
    algorithm_name_ = algorithm_name;
    trial_number_ = trial_number;
    
    // Close previous log file if open
    if (log_file_.is_open()) {
        log_file_.close();
    }
    
    // Create timestamp for filename
    auto now = std::chrono::system_clock::now();
    auto time_t = std::chrono::system_clock::to_time_t(now);
    auto tm = *std::localtime(&time_t);
    
    std::stringstream ss;
    ss << "logs/research_data/" << experiment_name_ << "_"
       << scenario_name_ << "_" << algorithm_name_ << "_"
       << "trial" << std::setfill('0') << std::setw(2) << trial_number_ << "_"
       << std::put_time(&tm, "%Y%m%d_%H%M%S") << ".csv";
    
    current_log_file_ = ss.str();
    
    // Open new log file
    log_file_.open(current_log_file_, std::ios::out | std::ios::trunc);
    if (!log_file_.is_open()) {
        throw std::runtime_error("Failed to open log file: " + current_log_file_);
    }
    
    writeCSVHeader();
    
    // Reset state
    session_start_time_ = std::chrono::high_resolution_clock::now();
    last_log_time_ = session_start_time_;
    gt_pose_valid_ = false;
    est_pose_valid_ = false;
    
    logging_enabled_ = true;
}

void ResearchDataLogger::logDataPoint(const ResearchMetrics& metrics) {
    if (!logging_enabled_ || !log_file_.is_open()) {
        return;
    }
    
    log_file_ << std::fixed << std::setprecision(6)
              << metrics.timestamp << ","
              << metrics.gt_x << "," << metrics.gt_y << "," << metrics.gt_theta << ","
              << metrics.est_x << "," << metrics.est_y << "," << metrics.est_theta << ","
              << metrics.num_particles << "," << metrics.effective_sample_size << ","
              << metrics.processing_time_ms << "," << metrics.dynamic_obstacles_detected << ","
              << metrics.position_error << "," << metrics.angular_error << ","
              << metrics.particle_std_dev << std::endl;
    
    log_file_.flush(); // Ensure data is written immediately
}

void ResearchDataLogger::setGroundTruthPose(const geometry_msgs::msg::PoseStamped& pose) {
    ground_truth_pose_ = pose;
    gt_pose_valid_ = true;
}

void ResearchDataLogger::setEstimatedPose(const geometry_msgs::msg::PoseStamped& pose) {
    estimated_pose_ = pose;
    est_pose_valid_ = true;
}

void ResearchDataLogger::updateAlgorithmMetrics(int num_particles, double ess, 
                                               double processing_time, int dynamic_obstacles,
                                               double particle_std_dev) {
    num_particles_ = num_particles;
    effective_sample_size_ = ess;
    processing_time_ms_ = processing_time;
    dynamic_obstacles_detected_ = dynamic_obstacles;
    particle_std_dev_ = particle_std_dev;
}

void ResearchDataLogger::logCurrentState() {
    if (!logging_enabled_ || !gt_pose_valid_ || !est_pose_valid_) {
        return;
    }
    
    auto now = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(now - session_start_time_);
    double timestamp = duration.count() / 1000.0; // Convert to seconds
    
    ResearchMetrics metrics;
    metrics.timestamp = timestamp;
    
    // Ground truth pose
    metrics.gt_x = ground_truth_pose_.pose.position.x;
    metrics.gt_y = ground_truth_pose_.pose.position.y;
    metrics.gt_theta = extractYaw(ground_truth_pose_.pose.orientation);
    
    // Estimated pose
    metrics.est_x = estimated_pose_.pose.position.x;
    metrics.est_y = estimated_pose_.pose.position.y;
    metrics.est_theta = extractYaw(estimated_pose_.pose.orientation);
    
    // Algorithm metrics
    metrics.num_particles = num_particles_;
    metrics.effective_sample_size = effective_sample_size_;
    metrics.processing_time_ms = processing_time_ms_;
    metrics.dynamic_obstacles_detected = dynamic_obstacles_detected_;
    metrics.particle_std_dev = particle_std_dev_;
    
    // Calculate errors
    metrics.position_error = calculatePositionError();
    metrics.angular_error = calculateAngularError();
    
    logDataPoint(metrics);
    last_log_time_ = now;
}

void ResearchDataLogger::finalizeSession() {
    if (log_file_.is_open()) {
        log_file_.close();
    }
    logging_enabled_ = false;
}

double ResearchDataLogger::calculatePositionError() const {
    if (!gt_pose_valid_ || !est_pose_valid_) {
        return 0.0;
    }
    
    double dx = ground_truth_pose_.pose.position.x - estimated_pose_.pose.position.x;
    double dy = ground_truth_pose_.pose.position.y - estimated_pose_.pose.position.y;
    
    return std::sqrt(dx * dx + dy * dy);
}

double ResearchDataLogger::calculateAngularError() const {
    if (!gt_pose_valid_ || !est_pose_valid_) {
        return 0.0;
    }
    
    double gt_yaw = extractYaw(ground_truth_pose_.pose.orientation);
    double est_yaw = extractYaw(estimated_pose_.pose.orientation);
    
    return std::abs(normalizeAngle(gt_yaw - est_yaw));
}

double ResearchDataLogger::normalizeAngle(double angle) const {
    while (angle > M_PI) angle -= 2.0 * M_PI;
    while (angle < -M_PI) angle += 2.0 * M_PI;
    return angle;
}

double ResearchDataLogger::extractYaw(const geometry_msgs::msg::Quaternion& orientation) const {
    tf2::Quaternion q(orientation.x, orientation.y, orientation.z, orientation.w);
    tf2::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);
    return yaw;
}

void ResearchDataLogger::writeCSVHeader() {
    log_file_ << "timestamp,gt_x,gt_y,gt_theta,est_x,est_y,est_theta,"
              << "num_particles,effective_sample_size,processing_time_ms,"
              << "dynamic_obstacles_detected,position_error,angular_error,"
              << "particle_std_dev" << std::endl;
}
