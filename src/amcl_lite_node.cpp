#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <gazebo_msgs/msg/model_states.hpp>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <ros2_amcl_lite/particle_filter.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <optional>
#include <ros2_amcl_lite/likelihood_field.hpp>
#include <ros2_amcl_lite/dynamic_object_detector.hpp>
#include <ros2_amcl_lite/dynamic_aware_sensor_model.hpp>
#include <ros2_amcl_lite/research_data_logger.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <chrono>

class AmclLiteNode : public rclcpp::Node {
public:
    AmclLiteNode() : Node("amcl_lite_node"), 
                     dynamic_detector_(0.5, 0.1, 0.3, 10),
                     dynamic_sensor_model_(0.2, 0.7),
                     data_logger_("amcl_research") {
        
        // Declare parameters first
        this->declare_parameter("max_particles", 2000);
        this->declare_parameter("min_particles", 500);
        this->declare_parameter("enable_dynamic_detection", true);
        this->declare_parameter("dynamic_detection_threshold", 0.5);
        this->declare_parameter("dynamic_weight", 0.7);
        this->declare_parameter("sensor_sigma", 0.2);
        this->declare_parameter("set_initial_pose", true);
        this->declare_parameter("initial_pose_x", 0.0);
        this->declare_parameter("initial_pose_y", 0.0);
        this->declare_parameter("initial_pose_a", 0.0);
        
        // Research parameters
        this->declare_parameter("enable_research_logging", false);
        this->declare_parameter("scenario_name", "default");
        this->declare_parameter("algorithm_name", "amcl_lite");
        this->declare_parameter("trial_number", 1);
        
        // Initialize particle filter with configurable particle count
        int max_particles = this->get_parameter("max_particles").as_int();
        pf_ = ParticleFilter(max_particles);
        
        // Initialize transform broadcaster
        tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);
        scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "/bcr_bot/scan", 10,
            std::bind(&AmclLiteNode::scan_callback, this, std::placeholders::_1));
        odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/bcr_bot/odom", 10,
            std::bind(&AmclLiteNode::odom_callback, this, std::placeholders::_1));
        map_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
            "/map", 10,
            std::bind(&AmclLiteNode::map_callback, this, std::placeholders::_1));
        initial_pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
            "/initialpose", 1,
            std::bind(&AmclLiteNode::initial_pose_callback, this, std::placeholders::_1));
        
        // Ground truth subscription for research logging
        ground_truth_sub_ = this->create_subscription<gazebo_msgs::msg::ModelStates>(
            "/model_states", 10,
            std::bind(&AmclLiteNode::ground_truth_callback, this, std::placeholders::_1));
        
        pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(
            "amcl_lite_pose", 10);
        particles_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
            "/amcl_lite_particles", 10);
        
        // Initialize research logging if enabled
        if (this->get_parameter("enable_research_logging").as_bool()) {
            std::string scenario = this->get_parameter("scenario_name").as_string();
            std::string algorithm = this->get_parameter("algorithm_name").as_string();
            int trial = this->get_parameter("trial_number").as_int();
            
            data_logger_.initializeSession(scenario, algorithm, trial);
            RCLCPP_INFO(this->get_logger(), "Research logging enabled. Log file: %s", 
                       data_logger_.getCurrentLogFile().c_str());
        }
        
        // Set initial pose if configured
        if (this->get_parameter("set_initial_pose").as_bool()) {
            initializeParticlesAtPose(
                this->get_parameter("initial_pose_x").as_double(),
                this->get_parameter("initial_pose_y").as_double(),
                this->get_parameter("initial_pose_a").as_double()
            );
        }
    }
    void initial_pose_callback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg) {
        // Set all particles around the initial pose with some noise
        auto& particles = pf_.particles();
        double x = msg->pose.pose.position.x;
        double y = msg->pose.pose.position.y;
        double theta = getYaw(msg->pose.pose.orientation);
        double std_x = std::sqrt(msg->pose.covariance[0]);
        double std_y = std::sqrt(msg->pose.covariance[7]);
        double std_theta = std::sqrt(msg->pose.covariance[35]);
        std::default_random_engine gen(std::random_device{}());
        std::normal_distribution<double> dist_x(x, std_x > 0.01 ? std_x : 0.1);
        std::normal_distribution<double> dist_y(y, std_y > 0.01 ? std_y : 0.1);
        std::normal_distribution<double> dist_theta(theta, std_theta > 0.01 ? std_theta : 0.1);
        for (auto& p : particles) {
            p.x = dist_x(gen);
            p.y = dist_y(gen);
            p.theta = dist_theta(gen);
            p.weight = 1.0 / particles.size();
        }
        RCLCPP_INFO(this->get_logger(), "Set initial pose: (%.2f, %.2f, %.2f)", x, y, theta);
    }

private:

    void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
        if (!likelihood_field_initialized_ || !current_map_) return;
        
        // Start timing for research logging
        auto scan_start_time_ = std::chrono::high_resolution_clock::now();
        
        // Get current parameters
        bool enable_dynamic = this->get_parameter("enable_dynamic_detection").as_bool();
        double dynamic_weight = this->get_parameter("dynamic_weight").as_double();
        double sensor_sigma = this->get_parameter("sensor_sigma").as_double();
        
        // Update sensor model parameters
        dynamic_sensor_model_.setParameters(sensor_sigma, dynamic_weight);
        
        auto& particles = pf_.particles();
        
        // Get current pose estimate for dynamic object detection
        geometry_msgs::msg::PoseStamped current_pose;
        if (!particles.empty()) {
            // Use mean pose as current estimate
            double mean_x = 0, mean_y = 0, mean_theta = 0;
            for (const auto& p : particles) {
                mean_x += p.x;
                mean_y += p.y;
                mean_theta += p.theta;
            }
            mean_x /= particles.size();
            mean_y /= particles.size();
            mean_theta /= particles.size();
            
            current_pose.header.stamp = msg->header.stamp;
            current_pose.header.frame_id = "map";
            current_pose.pose.position.x = mean_x;
            current_pose.pose.position.y = mean_y;
            current_pose.pose.orientation.w = std::cos(mean_theta / 2);
            current_pose.pose.orientation.z = std::sin(mean_theta / 2);
        }
        
        // Update dynamic object detection if enabled
        if (enable_dynamic && !particles.empty()) {
            dynamic_detector_.updateDetection(*msg, current_pose, *current_map_);
        }
        
        // Update particle weights
        if (enable_dynamic) {
            // Use dynamic-aware sensor model
            for (auto& p : particles) {
                std::array<double, 3> pose = {p.x, p.y, p.theta};
                p.weight = dynamic_sensor_model_.calculateParticleWeight(
                    *msg, pose, likelihood_field_, dynamic_detector_, *current_map_);
            }
        } else {
            // Use standard sensor model
            double sigma = sensor_sigma;
            double norm_const = 1.0 / (std::sqrt(2 * M_PI) * sigma);
            for (auto& p : particles) {
                double log_weight = 0.0;
                double px = p.x, py = p.y, ptheta = p.theta;
                for (size_t i = 0; i < msg->ranges.size(); i += 8) {
                    double range = msg->ranges[i];
                    if (range < msg->range_min || range > msg->range_max) continue;
                    double angle = msg->angle_min + i * msg->angle_increment;
                    double mx = px + range * std::cos(ptheta + angle);
                    double my = py + range * std::sin(ptheta + angle);
                    double dist = likelihood_field_.getDistance(mx, my);
                    double prob = norm_const * std::exp(-0.5 * (dist * dist) / (sigma * sigma));
                    log_weight += std::log(prob + 1e-9);
                }
                p.weight = std::exp(log_weight);
            }
        }
        
        // Normalize weights
        double sum = 0.0;
        for (const auto& p : particles) sum += p.weight;
        for (auto& p : particles) p.weight /= (sum + 1e-9);

        // Disabled kidnapping detection to prevent pose jumping
        // Check for kidnapping - if all weights are very low, robot might be kidnapped
        double max_weight = 0.0;
        double avg_weight = 0.0;
        for (const auto& p : particles) {
            if (p.weight > max_weight) max_weight = p.weight;
            avg_weight += p.weight;
        }
        avg_weight /= particles.size();
        
        // Kidnapping detection is disabled - comment out the detection logic
        // This prevents random pose jumping during normal operation
        /*
        // Use a much more conservative threshold - only trigger if weights are extremely low
        // AND the ratio between max and average is very low (indicating uniform low confidence)
        double kidnapping_threshold = 1.0 / particles.size() * 5.0; // Much more conservative
        double weight_ratio = max_weight / (avg_weight + 1e-9);
        
        if (max_weight < kidnapping_threshold && weight_ratio < 2.0) {
            // Add a counter to prevent continuous triggering
            static int kidnapping_count = 0;
            static auto last_kidnapping_time = std::chrono::steady_clock::now();
            auto current_time = std::chrono::steady_clock::now();
            auto time_diff = std::chrono::duration_cast<std::chrono::seconds>(current_time - last_kidnapping_time);
            
            // Only trigger kidnapping recovery if it's been more than 30 seconds since last trigger
            if (time_diff.count() > 30) {
                RCLCPP_WARN(this->get_logger(), "Possible kidnapping detected! Max weight: %.6f, Avg weight: %.6f, Ratio: %.2f", 
                           max_weight, avg_weight, weight_ratio);
                
                // Reinitialize particles globally across the map
                if (current_map_) {
                    initializeParticlesGlobally();
                }
                
                last_kidnapping_time = current_time;
                kidnapping_count++;
            }
        }
        */

        // Resample step (systematic resampling)
        std::vector<Particle> new_particles;
        new_particles.reserve(particles.size());
        double step = 1.0 / particles.size();
        double r = ((double)rand() / RAND_MAX) * step;
        double c = particles[0].weight;
        size_t i = 0;
        for (size_t m = 0; m < particles.size(); ++m) {
            double U = r + m * step;
            while (U > c && i < particles.size() - 1) {
                i++;
                c += particles[i].weight;
            }
            new_particles.push_back(particles[i]);
            new_particles.back().weight = 1.0 / particles.size();
        }
        particles = new_particles;

        // Publish estimated pose (mean)
        double mean_x = 0, mean_y = 0, mean_theta = 0;
        for (const auto& p : particles) {
            mean_x += p.x;
            mean_y += p.y;
            mean_theta += p.theta;
        }
        mean_x /= particles.size();
        mean_y /= particles.size();
        mean_theta /= particles.size();
        geometry_msgs::msg::PoseStamped pose_msg;
        pose_msg.header.stamp = msg->header.stamp;
        pose_msg.header.frame_id = "map";
        pose_msg.pose.position.x = mean_x;
        pose_msg.pose.position.y = mean_y;
        pose_msg.pose.position.z = 0.0;
        pose_msg.pose.orientation.w = std::cos(mean_theta / 2);
        pose_msg.pose.orientation.x = 0.0;
        pose_msg.pose.orientation.y = 0.0;
        pose_msg.pose.orientation.z = std::sin(mean_theta / 2);
        pose_pub_->publish(pose_msg);

        // Publish map->odom transform only if localization is confident
        // Wait for a few successful updates before taking over from static transform
        if (last_odom_) {
            static int stable_updates = 0;
            static bool transform_active = false;
            
            // Check if localization is stable (good particle weights)
            double confidence = max_weight / (avg_weight + 1e-9);
            bool is_stable = (confidence > 3.0) && (max_weight > 0.001);
            
            if (is_stable) {
                stable_updates++;
            } else {
                stable_updates = std::max(0, stable_updates - 1);
            }
            
            // Only start publishing transform after 10 stable updates
            if (stable_updates > 10) {
                transform_active = true;
            }
            
            if (transform_active) {
                double odom_x = last_odom_->pose.pose.position.x;
                double odom_y = last_odom_->pose.pose.position.y;
                double odom_theta = getYaw(last_odom_->pose.pose.orientation);
                
                // Calculate the transform as the difference between map pose and odom pose
                double transform_x = mean_x - odom_x;
                double transform_y = mean_y - odom_y;
                double transform_theta = mean_theta - odom_theta;
                
                publishTransform(transform_x, transform_y, transform_theta, msg->header.stamp);
            }
        }

        // Publish particles as PointCloud2
        sensor_msgs::msg::PointCloud2 cloud;
        cloud.header.stamp = msg->header.stamp;
        cloud.header.frame_id = "map";
        cloud.height = 1;
        cloud.width = particles.size();
        cloud.is_dense = false;
        cloud.is_bigendian = false;
        sensor_msgs::PointCloud2Modifier modifier(cloud);
        modifier.setPointCloud2FieldsByString(1, "xyz");
        modifier.resize(particles.size());
        sensor_msgs::PointCloud2Iterator<float> iter_x(cloud, "x");
        sensor_msgs::PointCloud2Iterator<float> iter_y(cloud, "y");
        sensor_msgs::PointCloud2Iterator<float> iter_z(cloud, "z");
        for (const auto& p : particles) {
            *iter_x = p.x; ++iter_x;
            *iter_y = p.y; ++iter_y;
            *iter_z = 0.0f; ++iter_z;
        }
        particles_pub_->publish(cloud);
        
        // Log dynamic obstacles if enabled
        if (enable_dynamic) {
            const auto& obstacles = dynamic_detector_.getCurrentObstacles();
            if (!obstacles.empty()) {
                RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000, 
                                    "Detected %zu dynamic obstacles", obstacles.size());
            }
        }
        
        // Research data logging
        if (data_logger_.isLoggingEnabled()) {
            // Calculate processing time
            auto end_time = std::chrono::high_resolution_clock::now();
            auto processing_time = std::chrono::duration_cast<std::chrono::microseconds>(end_time - scan_start_time_);
            
            // Calculate effective sample size
            double ess = calculateEffectiveSampleSize(particles);
            
            // Calculate particle standard deviation
            double particle_std = calculateParticleStandardDeviation(particles);
            
            // Update algorithm metrics
            int num_dynamic_obstacles = enable_dynamic ? dynamic_detector_.getCurrentObstacles().size() : 0;
            data_logger_.updateAlgorithmMetrics(particles.size(), ess, 
                                              processing_time.count() / 1000.0, // Convert to milliseconds
                                              num_dynamic_obstacles, particle_std);
            
            // Set estimated pose
            data_logger_.setEstimatedPose(pose_msg);
            
            // Log current state (only if ground truth is available)
            data_logger_.logCurrentState();
        }
    }
    void map_callback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
        RCLCPP_INFO(this->get_logger(), "Received OccupancyGrid with size: %zu", msg->data.size());
        likelihood_field_.fromOccupancyGrid(*msg);
        current_map_ = msg;  // Store map for dynamic object detection
        RCLCPP_INFO(this->get_logger(), "Likelihood field initialized.");
        likelihood_field_initialized_ = true;
    }

    void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg) {
        // Compute odometry delta
        if (last_odom_) {
            double dx = msg->pose.pose.position.x - last_odom_->pose.pose.position.x;
            double dy = msg->pose.pose.position.y - last_odom_->pose.pose.position.y;
            double dtheta = getYaw(msg->pose.pose.orientation) - getYaw(last_odom_->pose.pose.orientation);
            double noise[3] = {0.01, 0.01, 0.01}; // Example noise stddevs
            pf_.predict(dx, dy, dtheta, noise);
        }
        last_odom_ = *msg;
    }

    double getYaw(const geometry_msgs::msg::Quaternion& q) {
        // Convert quaternion to yaw
        double siny_cosp = 2.0 * (q.w * q.z + q.x * q.y);
        double cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z);
        return std::atan2(siny_cosp, cosy_cosp);
    }

    void initializeParticlesAtPose(double x, double y, double theta) {
        auto& particles = pf_.particles();
        std::default_random_engine gen(std::random_device{}());
        std::normal_distribution<double> dist_x(x, 0.1);
        std::normal_distribution<double> dist_y(y, 0.1);
        std::normal_distribution<double> dist_theta(theta, 0.1);
        
        for (auto& p : particles) {
            p.x = dist_x(gen);
            p.y = dist_y(gen);
            p.theta = dist_theta(gen);
            p.weight = 1.0 / particles.size();
        }
    }

    void initializeParticlesGlobally() {
        if (!current_map_) return;
        
        auto& particles = pf_.particles();
        std::default_random_engine gen(std::random_device{}());
        std::uniform_real_distribution<double> dist_theta(-M_PI, M_PI);
        
        // Find free space in the map
        std::vector<std::pair<double, double>> free_positions;
        for (int y = 0; y < current_map_->info.height; y += 5) {
            for (int x = 0; x < current_map_->info.width; x += 5) {
                int index = y * current_map_->info.width + x;
                if (index < current_map_->data.size() && current_map_->data[index] == 0) {
                    // Convert grid coordinates to world coordinates
                    double world_x = current_map_->info.origin.position.x + x * current_map_->info.resolution;
                    double world_y = current_map_->info.origin.position.y + y * current_map_->info.resolution;
                    free_positions.push_back({world_x, world_y});
                }
            }
        }
        
        if (free_positions.empty()) {
            RCLCPP_WARN(this->get_logger(), "No free space found in map for global initialization");
            return;
        }
        
        std::uniform_int_distribution<size_t> pos_dist(0, free_positions.size() - 1);
        
        for (auto& p : particles) {
            auto pos = free_positions[pos_dist(gen)];
            p.x = pos.first;
            p.y = pos.second;
            p.theta = dist_theta(gen);
            p.weight = 1.0 / particles.size();
        }
    }

    void publishTransform(double x, double y, double theta, const rclcpp::Time& stamp) {
        geometry_msgs::msg::TransformStamped transform;
        transform.header.stamp = stamp;
        transform.header.frame_id = "map";
        transform.child_frame_id = "odom";
        transform.transform.translation.x = x;
        transform.transform.translation.y = y;
        transform.transform.translation.z = 0.0;
        tf2::Quaternion q;
        q.setRPY(0, 0, theta);
        transform.transform.rotation.x = q.x();
        transform.transform.rotation.y = q.y();
        transform.transform.rotation.z = q.z();
        transform.transform.rotation.w = q.w();
        tf_broadcaster_->sendTransform(transform);
    }
    
    void ground_truth_callback(const gazebo_msgs::msg::ModelStates::SharedPtr msg) {
        if (!data_logger_.isLoggingEnabled()) return;
        
        // Find the robot model (usually named "bcr_bot" or similar)
        std::string robot_name = "bcr_bot";
        for (size_t i = 0; i < msg->name.size(); ++i) {
            if (msg->name[i] == robot_name) {
                geometry_msgs::msg::PoseStamped ground_truth_pose;
                ground_truth_pose.header.stamp = this->get_clock()->now();
                ground_truth_pose.header.frame_id = "map";
                ground_truth_pose.pose = msg->pose[i];
                
                data_logger_.setGroundTruthPose(ground_truth_pose);
                return;
            }
        }
        
        // If exact name not found, try to find first model that contains "bot"
        for (size_t i = 0; i < msg->name.size(); ++i) {
            if (msg->name[i].find("bot") != std::string::npos) {
                geometry_msgs::msg::PoseStamped ground_truth_pose;
                ground_truth_pose.header.stamp = this->get_clock()->now();
                ground_truth_pose.header.frame_id = "map";
                ground_truth_pose.pose = msg->pose[i];
                
                data_logger_.setGroundTruthPose(ground_truth_pose);
                return;
            }
        }
    }
    
    double calculateEffectiveSampleSize(const std::vector<Particle>& particles) {
        double sum_weights_squared = 0.0;
        for (const auto& p : particles) {
            sum_weights_squared += p.weight * p.weight;
        }
        return 1.0 / (sum_weights_squared + 1e-9);
    }
    
    double calculateParticleStandardDeviation(const std::vector<Particle>& particles) {
        if (particles.empty()) return 0.0;
        
        // Calculate mean position
        double mean_x = 0.0, mean_y = 0.0;
        for (const auto& p : particles) {
            mean_x += p.x;
            mean_y += p.y;
        }
        mean_x /= particles.size();
        mean_y /= particles.size();
        
        // Calculate standard deviation
        double sum_squared_diff = 0.0;
        for (const auto& p : particles) {
            double dx = p.x - mean_x;
            double dy = p.y - mean_y;
            sum_squared_diff += dx * dx + dy * dy;
        }
        
        return std::sqrt(sum_squared_diff / particles.size());
    }

    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Subscription<gazebo_msgs::msg::ModelStates>::SharedPtr ground_truth_sub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_pub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr particles_pub_;
    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;
    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    ParticleFilter pf_;
    LikelihoodField likelihood_field_;
    DynamicObjectDetector dynamic_detector_;
    DynamicAwareSensorModel dynamic_sensor_model_;
    ResearchDataLogger data_logger_;
    bool likelihood_field_initialized_ = false;
    std::optional<nav_msgs::msg::Odometry> last_odom_;
    rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr initial_pose_sub_;
    nav_msgs::msg::OccupancyGrid::SharedPtr current_map_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<AmclLiteNode>());
    rclcpp::shutdown();
    return 0;
}
