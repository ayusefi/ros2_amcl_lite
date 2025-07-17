#include "ros2_amcl_lite/dynamic_aware_sensor_model.hpp"
#include <cmath>
#include <algorithm>

DynamicAwareSensorModel::DynamicAwareSensorModel(double sigma, double dynamic_weight)
    : sigma_(sigma), dynamic_weight_(dynamic_weight) {
}

double DynamicAwareSensorModel::calculateParticleWeight(
    const sensor_msgs::msg::LaserScan& scan,
    const std::array<double, 3>& particle_pose,
    const LikelihoodField& likelihood_field,
    const DynamicObjectDetector& dynamic_detector,
    const nav_msgs::msg::OccupancyGrid& /* map */) const {
    
    double log_weight = 0.0;
    double norm_const = 1.0 / (std::sqrt(2 * M_PI) * sigma_);
    
    // Sample every nth beam to reduce computation
    const size_t beam_skip = 8;
    
    for (size_t i = 0; i < scan.ranges.size(); i += beam_skip) {
        double range = scan.ranges[i];
        
        // Skip invalid ranges
        if (range < scan.range_min || range > scan.range_max) {
            continue;
        }
        
        double angle = scan.angle_min + i * scan.angle_increment;
        
        // Transform beam endpoint to map frame
        double px = particle_pose[0];
        double py = particle_pose[1];
        double ptheta = particle_pose[2];
        
        double beam_x = px + range * std::cos(ptheta + angle);
        double beam_y = py + range * std::sin(ptheta + angle);
        
        // Get distance to nearest obstacle from likelihood field
        double dist_to_obstacle = likelihood_field.getDistance(beam_x, beam_y);
        
        // Calculate base probability using likelihood field
        double base_prob = norm_const * std::exp(-0.5 * (dist_to_obstacle * dist_to_obstacle) / (sigma_ * sigma_));
        
        // Get dynamic object weight modifier
        double dynamic_modifier = dynamic_detector.getBeamWeightModifier(beam_x, beam_y);
        
        // Combine base probability with dynamic object awareness
        // Lower dynamic_modifier means likely dynamic object, so reduce weight
        double combined_prob = base_prob * (dynamic_weight_ * dynamic_modifier + (1.0 - dynamic_weight_));
        
        // Add to log weight
        log_weight += std::log(combined_prob + 1e-9);
    }
    
    return std::exp(log_weight);
}

void DynamicAwareSensorModel::setParameters(double sigma, double dynamic_weight) {
    sigma_ = sigma;
    dynamic_weight_ = dynamic_weight;
}

double DynamicAwareSensorModel::calculateExpectedRange(
    double beam_angle, 
    const std::array<double, 3>& particle_pose,
    const nav_msgs::msg::OccupancyGrid& map,
    double max_range) const {
    
    double px = particle_pose[0];
    double py = particle_pose[1];
    double ptheta = particle_pose[2];
    
    double ray_angle = ptheta + beam_angle;
    
    return castRay(px, py, ray_angle, map, max_range);
}

double DynamicAwareSensorModel::castRay(double start_x, double start_y, double angle,
                                       const nav_msgs::msg::OccupancyGrid& map,
                                       double max_range) const {
    
    double dx = std::cos(angle);
    double dy = std::sin(angle);
    
    double step_size = map.info.resolution * 0.5; // Sub-pixel stepping
    double range = 0.0;
    
    while (range < max_range) {
        double x = start_x + range * dx;
        double y = start_y + range * dy;
        
        if (isOccupied(x, y, map)) {
            return range;
        }
        
        range += step_size;
    }
    
    return max_range;
}

bool DynamicAwareSensorModel::isOccupied(double x, double y, 
                                        const nav_msgs::msg::OccupancyGrid& map) const {
    // Convert world coordinates to map coordinates
    int mx = static_cast<int>((x - map.info.origin.position.x) / map.info.resolution);
    int my = static_cast<int>((y - map.info.origin.position.y) / map.info.resolution);
    
    // Check bounds
    if (mx < 0 || my < 0 || mx >= static_cast<int>(map.info.width) || 
        my >= static_cast<int>(map.info.height)) {
        return true; // Treat out-of-bounds as occupied
    }
    
    // Check if cell is occupied (value > 50 typically means occupied)
    int idx = my * map.info.width + mx;
    return map.data[idx] > 50;
}
