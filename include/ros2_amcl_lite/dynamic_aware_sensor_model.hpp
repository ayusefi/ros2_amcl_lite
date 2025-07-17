#pragma once
#include "ros2_amcl_lite/likelihood_field.hpp"
#include "ros2_amcl_lite/dynamic_object_detector.hpp"
#include <sensor_msgs/msg/laser_scan.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>

class DynamicAwareSensorModel {
public:
    DynamicAwareSensorModel(double sigma = 0.2, double dynamic_weight = 0.5);
    
    /**
     * @brief Calculate particle weight based on laser scan with dynamic object awareness
     * @param scan Laser scan message
     * @param particle_pose Particle pose (x, y, theta)
     * @param likelihood_field Likelihood field for static obstacles
     * @param dynamic_detector Dynamic object detector
     * @param map Static occupancy grid map
     * @return Particle weight (likelihood)
     */
    double calculateParticleWeight(const sensor_msgs::msg::LaserScan& scan,
                                 const std::array<double, 3>& particle_pose,
                                 const LikelihoodField& likelihood_field,
                                 const DynamicObjectDetector& dynamic_detector,
                                 const nav_msgs::msg::OccupancyGrid& map) const;
    
    /**
     * @brief Set sensor model parameters
     * @param sigma Standard deviation of sensor noise
     * @param dynamic_weight Weight for dynamic object detection influence
     */
    void setParameters(double sigma, double dynamic_weight);
    
    /**
     * @brief Get current parameters
     * @return Pair of (sigma, dynamic_weight)
     */
    std::pair<double, double> getParameters() const { return {sigma_, dynamic_weight_}; }
    
private:
    /**
     * @brief Calculate expected measurement for a beam
     * @param beam_angle Beam angle relative to robot
     * @param particle_pose Particle pose (x, y, theta)
     * @param map Static occupancy grid map
     * @param max_range Maximum sensor range
     * @return Expected range measurement
     */
    double calculateExpectedRange(double beam_angle, 
                                const std::array<double, 3>& particle_pose,
                                const nav_msgs::msg::OccupancyGrid& map,
                                double max_range) const;
    
    /**
     * @brief Ray casting to find expected range
     * @param start_x Starting x coordinate
     * @param start_y Starting y coordinate
     * @param angle Ray angle
     * @param map Static occupancy grid map
     * @param max_range Maximum range to cast
     * @return Range to first obstacle or max_range
     */
    double castRay(double start_x, double start_y, double angle,
                   const nav_msgs::msg::OccupancyGrid& map,
                   double max_range) const;
    
    /**
     * @brief Check if map cell is occupied
     * @param x X coordinate in map frame
     * @param y Y coordinate in map frame
     * @param map Static occupancy grid map
     * @return True if cell is occupied
     */
    bool isOccupied(double x, double y, const nav_msgs::msg::OccupancyGrid& map) const;
    
    double sigma_;          // Sensor noise standard deviation
    double dynamic_weight_; // Weight for dynamic object influence
};
