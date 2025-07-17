#include "ros2_amcl_lite/dynamic_object_detector.hpp"
#include <cmath>
#include <algorithm>
#include <chrono>

DynamicObjectDetector::DynamicObjectDetector(double threshold, double decay_rate, 
                                           double min_distance_diff, size_t history_size)
    : detection_threshold_(threshold), decay_rate_(decay_rate), 
      min_distance_diff_(min_distance_diff), history_size_(history_size),
      last_update_time_(0.0) {
}

std::vector<DynamicObstacle> DynamicObjectDetector::detectDynamicObstacles(
    const sensor_msgs::msg::LaserScan& scan,
    const geometry_msgs::msg::PoseStamped& pose,
    const nav_msgs::msg::OccupancyGrid& map) {
    
    updateDetection(scan, pose, map);
    return current_obstacles_;
}

void DynamicObjectDetector::updateDetection(const sensor_msgs::msg::LaserScan& scan,
                                          const geometry_msgs::msg::PoseStamped& pose,
                                          const nav_msgs::msg::OccupancyGrid& map) {
    
    double current_time = scan.header.stamp.sec + scan.header.stamp.nanosec * 1e-9;
    
    // Add current scan to history
    ScanHistory current_scan;
    current_scan.ranges = scan.ranges;
    current_scan.timestamp = current_time;
    current_scan.pose = pose;
    
    scan_history_.push_back(current_scan);
    
    // Maintain history size
    if (scan_history_.size() > history_size_) {
        scan_history_.pop_front();
    }
    
    // Clear previous obstacles and detect new ones
    current_obstacles_.clear();
    
    // Need at least 2 scans for comparison
    if (scan_history_.size() < 2) {
        last_update_time_ = current_time;
        return;
    }
    
    // Get previous scan for comparison
    const auto& previous = scan_history_[scan_history_.size() - 2];
    
    // Compare current scan with previous scans
    for (size_t i = 0; i < scan.ranges.size(); ++i) {
        double range = scan.ranges[i];
        
        // Skip invalid ranges
        if (range < scan.range_min || range > scan.range_max) {
            continue;
        }
        
        // Calculate beam angle
        double angle = scan.angle_min + i * scan.angle_increment;
        
        // Transform to map frame
        double map_x, map_y;
        transformToMapFrame(range, angle, pose, map_x, map_y);
        
        // Check if this point should be free according to static map
        if (!isExpectedFree(map_x, map_y, map)) {
            continue; // This is expected static obstacle
        }
        
        // Compare with previous scan at same angle
        if (i < previous.ranges.size()) {
            double prev_range = previous.ranges[i];
            
            // Skip if previous range was invalid
            if (prev_range < scan.range_min || prev_range > scan.range_max) {
                continue;
            }
            
            // Calculate distance difference
            double range_diff = std::abs(range - prev_range);
            
            // If there's a significant difference and current reading is shorter
            // (indicating something moved into the beam), mark as dynamic
            if (range_diff > min_distance_diff_ && range < prev_range) {
                
                // Calculate confidence based on distance difference
                double confidence = std::min(1.0, range_diff / (2.0 * min_distance_diff_));
                
                // Create dynamic obstacle
                DynamicObstacle obstacle;
                obstacle.x = map_x;
                obstacle.y = map_y;
                obstacle.confidence = confidence;
                obstacle.timestamp = current_time;
                
                current_obstacles_.push_back(obstacle);
            }
        }
    }
    
    // Apply temporal filtering - merge nearby obstacles
    std::vector<DynamicObstacle> filtered_obstacles;
    const double merge_distance = 0.5; // meters
    
    for (const auto& obstacle : current_obstacles_) {
        bool merged = false;
        for (auto& existing : filtered_obstacles) {
            double dist = std::sqrt(std::pow(obstacle.x - existing.x, 2) + 
                                  std::pow(obstacle.y - existing.y, 2));
            if (dist < merge_distance) {
                // Merge - take higher confidence
                if (obstacle.confidence > existing.confidence) {
                    existing.x = obstacle.x;
                    existing.y = obstacle.y;
                    existing.confidence = obstacle.confidence;
                    existing.timestamp = obstacle.timestamp;
                }
                merged = true;
                break;
            }
        }
        
        if (!merged) {
            filtered_obstacles.push_back(obstacle);
        }
    }
    
    current_obstacles_ = filtered_obstacles;
    
    // Apply decay to obstacle confidence over time
    double dt = current_time - last_update_time_;
    for (auto& obstacle : current_obstacles_) {
        obstacle.confidence *= std::exp(-decay_rate_ * dt);
    }
    
    // Remove obstacles with low confidence
    current_obstacles_.erase(
        std::remove_if(current_obstacles_.begin(), current_obstacles_.end(),
                      [this](const DynamicObstacle& obs) {
                          return obs.confidence < detection_threshold_;
                      }),
        current_obstacles_.end());
    
    last_update_time_ = current_time;
}

double DynamicObjectDetector::getBeamWeightModifier(double beam_x, double beam_y) const {
    const double influence_radius = 0.3; // meters
    double min_weight = 1.0;
    
    for (const auto& obstacle : current_obstacles_) {
        double dist = std::sqrt(std::pow(beam_x - obstacle.x, 2) + 
                              std::pow(beam_y - obstacle.y, 2));
        
        if (dist < influence_radius) {
            // Weight modifier based on distance and confidence
            double weight_reduction = obstacle.confidence * 
                                    (1.0 - dist / influence_radius);
            double weight = 1.0 - weight_reduction;
            min_weight = std::min(min_weight, weight);
        }
    }
    
    // Ensure minimum weight to avoid complete beam rejection
    return std::max(0.1, min_weight);
}

bool DynamicObjectDetector::isExpectedFree(double x, double y, 
                                         const nav_msgs::msg::OccupancyGrid& map) const {
    // Convert world coordinates to map coordinates
    int mx = static_cast<int>((x - map.info.origin.position.x) / map.info.resolution);
    int my = static_cast<int>((y - map.info.origin.position.y) / map.info.resolution);
    
    // Check bounds
    if (mx < 0 || my < 0 || mx >= static_cast<int>(map.info.width) || 
        my >= static_cast<int>(map.info.height)) {
        return false;
    }
    
    // Check if cell is free (value < 50 typically means free)
    int idx = my * map.info.width + mx;
    return map.data[idx] < 50;
}

void DynamicObjectDetector::transformToMapFrame(double range, double angle,
                                              const geometry_msgs::msg::PoseStamped& pose,
                                              double& map_x, double& map_y) const {
    // Get robot yaw
    double robot_yaw = getYaw(pose.pose.orientation);
    
    // Transform laser point to map frame
    double laser_x = range * std::cos(angle);
    double laser_y = range * std::sin(angle);
    
    // Rotate and translate to map frame
    map_x = pose.pose.position.x + laser_x * std::cos(robot_yaw) - laser_y * std::sin(robot_yaw);
    map_y = pose.pose.position.y + laser_x * std::sin(robot_yaw) + laser_y * std::cos(robot_yaw);
}

double DynamicObjectDetector::getYaw(const geometry_msgs::msg::Quaternion& q) const {
    double siny_cosp = 2.0 * (q.w * q.z + q.x * q.y);
    double cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z);
    return std::atan2(siny_cosp, cosy_cosp);
}
