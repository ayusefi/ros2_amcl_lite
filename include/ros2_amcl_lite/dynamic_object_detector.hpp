#pragma once
#include <vector>
#include <deque>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>

struct DynamicObstacle {
    double x, y;  // Position in map frame
    double confidence;  // Confidence that this is a dynamic obstacle
    double timestamp;  // When this obstacle was detected
};

class DynamicObjectDetector {
public:
    DynamicObjectDetector(double threshold = 0.5, double decay_rate = 0.1, 
                         double min_distance_diff = 0.3, size_t history_size = 10);
    
    /**
     * @brief Detect dynamic obstacles from laser scan
     * @param scan Current laser scan
     * @param pose Current robot pose estimate
     * @param map Static occupancy grid map
     * @return Vector of detected dynamic obstacles
     */
    std::vector<DynamicObstacle> detectDynamicObstacles(
        const sensor_msgs::msg::LaserScan& scan,
        const geometry_msgs::msg::PoseStamped& pose,
        const nav_msgs::msg::OccupancyGrid& map);
    
    /**
     * @brief Get weight modifier for a laser beam based on dynamic obstacle detection
     * @param beam_x X coordinate of laser beam endpoint
     * @param beam_y Y coordinate of laser beam endpoint
     * @return Weight modifier (0.0 to 1.0) where lower values indicate dynamic obstacles
     */
    double getBeamWeightModifier(double beam_x, double beam_y) const;
    
    /**
     * @brief Update the detector with new scan data
     * @param scan Current laser scan
     * @param pose Current robot pose estimate
     * @param map Static occupancy grid map
     */
    void updateDetection(const sensor_msgs::msg::LaserScan& scan,
                        const geometry_msgs::msg::PoseStamped& pose,
                        const nav_msgs::msg::OccupancyGrid& map);
    
    /**
     * @brief Get current dynamic obstacles
     * @return Vector of currently tracked dynamic obstacles
     */
    const std::vector<DynamicObstacle>& getCurrentObstacles() const { return current_obstacles_; }
    
private:
    struct ScanHistory {
        std::vector<float> ranges;
        double timestamp;
        geometry_msgs::msg::PoseStamped pose;
    };
    
    /**
     * @brief Check if a point is expected to be free based on the static map
     * @param x X coordinate in map frame
     * @param y Y coordinate in map frame
     * @param map Static occupancy grid map
     * @return True if point should be free according to static map
     */
    bool isExpectedFree(double x, double y, const nav_msgs::msg::OccupancyGrid& map) const;
    
    /**
     * @brief Transform laser scan point to map frame
     * @param range Range measurement
     * @param angle Beam angle
     * @param pose Robot pose
     * @param map_x Output X coordinate in map frame
     * @param map_y Output Y coordinate in map frame
     */
    void transformToMapFrame(double range, double angle, 
                           const geometry_msgs::msg::PoseStamped& pose,
                           double& map_x, double& map_y) const;
    
    /**
     * @brief Extract yaw angle from quaternion
     * @param q Quaternion
     * @return Yaw angle in radians
     */
    double getYaw(const geometry_msgs::msg::Quaternion& q) const;
    
    // Configuration parameters
    double detection_threshold_;     // Minimum confidence to consider as dynamic
    double decay_rate_;             // Rate at which dynamic obstacle confidence decays
    double min_distance_diff_;      // Minimum distance difference to trigger detection
    size_t history_size_;           // Number of scans to keep in history
    
    // Internal state
    std::deque<ScanHistory> scan_history_;
    std::vector<DynamicObstacle> current_obstacles_;
    double last_update_time_;
};
