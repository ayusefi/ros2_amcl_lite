#pragma once
#include <vector>
#include "nav_msgs/msg/occupancy_grid.hpp"

class LikelihoodField {
public:
    LikelihoodField() = default;
    void fromOccupancyGrid(const nav_msgs::msg::OccupancyGrid& map);
    double getDistance(double x, double y) const;
private:
    std::vector<float> field_;
    unsigned int width_ = 0, height_ = 0;
    double resolution_ = 0.0, origin_x_ = 0.0, origin_y_ = 0.0;
};
