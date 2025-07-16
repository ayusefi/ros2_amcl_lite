#include "ros2_amcl_lite/likelihood_field.hpp"
#include <queue>
#include <cmath>
#include <limits>

void LikelihoodField::fromOccupancyGrid(const nav_msgs::msg::OccupancyGrid& map) {
    width_ = map.info.width;
    height_ = map.info.height;
    resolution_ = map.info.resolution;
    origin_x_ = map.info.origin.position.x;
    origin_y_ = map.info.origin.position.y;
    field_.assign(width_ * height_, std::numeric_limits<float>::max());

    // Mark occupied cells
    std::queue<std::pair<unsigned int, unsigned int>> q;
    for (unsigned int y = 0; y < height_; ++y) {
        for (unsigned int x = 0; x < width_; ++x) {
            unsigned int idx = y * width_ + x;
            if (map.data[idx] > 50) { // Occupied
                field_[idx] = 0.0f;
                q.push({x, y});
            }
        }
    }
    // BFS to fill distances
    const int dx[4] = {1, -1, 0, 0};
    const int dy[4] = {0, 0, 1, -1};
    while (!q.empty()) {
        auto [x, y] = q.front(); q.pop();
        unsigned int idx = y * width_ + x;
        for (int d = 0; d < 4; ++d) {
            int nx = x + dx[d], ny = y + dy[d];
            if (nx >= 0 && ny >= 0 && (unsigned)nx < width_ && (unsigned)ny < height_) {
                unsigned int nidx = ny * width_ + nx;
                float new_dist = field_[idx] + resolution_;
                if (field_[nidx] > new_dist) {
                    field_[nidx] = new_dist;
                    q.push({(unsigned)nx, (unsigned)ny});
                }
            }
        }
    }
}

double LikelihoodField::getDistance(double x, double y) const {
    int mx = static_cast<int>((x - origin_x_) / resolution_);
    int my = static_cast<int>((y - origin_y_) / resolution_);
    if (mx < 0 || my < 0 || (unsigned)mx >= width_ || (unsigned)my >= height_) {
        return std::numeric_limits<double>::max();
    }
    return field_[my * width_ + mx];
}
