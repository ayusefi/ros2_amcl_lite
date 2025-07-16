#include "ros2_amcl_lite/particle_filter.hpp"
#include <random>

ParticleFilter::ParticleFilter(size_t num_particles) {
    particles_.resize(num_particles);
    std::uniform_real_distribution<double> dist_x(-1.0, 1.0);
    std::uniform_real_distribution<double> dist_y(-1.0, 1.0);
    std::uniform_real_distribution<double> dist_theta(-M_PI, M_PI);
    for (auto& p : particles_) {
        p.x = dist_x(gen_);
        p.y = dist_y(gen_);
        p.theta = dist_theta(gen_);
        p.weight = 1.0 / num_particles;
    }
}

void ParticleFilter::predict(double delta_x, double delta_y, double delta_theta, double noise_std[]) {
    std::normal_distribution<double> noise_x(0.0, noise_std[0]);
    std::normal_distribution<double> noise_y(0.0, noise_std[1]);
    std::normal_distribution<double> noise_theta(0.0, noise_std[2]);
    for (auto& p : particles_) {
        p.x += delta_x + noise_x(gen_);
        p.y += delta_y + noise_y(gen_);
        p.theta += delta_theta + noise_theta(gen_);
    }
}

std::vector<Particle>& ParticleFilter::particles() {
    return particles_;
}
