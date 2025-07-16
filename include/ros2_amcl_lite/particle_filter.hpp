#pragma once
#include <vector>
#include <random>

struct Particle {
    double x;
    double y;
    double theta;
    double weight;
};

class ParticleFilter {
public:
    ParticleFilter(size_t num_particles = 100);
    void predict(double delta_x, double delta_y, double delta_theta, double noise_std[]);
    std::vector<Particle>& particles();
private:
    std::vector<Particle> particles_;
    std::default_random_engine gen_;
};
