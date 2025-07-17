#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <nav_msgs/msg/odometry.hpp>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <ros2_amcl_lite/particle_filter.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <optional>
#include <ros2_amcl_lite/likelihood_field.hpp>
#include <ros2_amcl_lite/dynamic_object_detector.hpp>
#include <ros2_amcl_lite/dynamic_aware_sensor_model.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>

class AmclLiteNode : public rclcpp::Node {
public:
    AmclLiteNode() : Node("amcl_lite_node"), pf_(100), 
                     dynamic_detector_(0.5, 0.1, 0.3, 10),
                     dynamic_sensor_model_(0.2, 0.7) {
        scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "/bcr_bot/scan", 10,
            std::bind(&AmclLiteNode::scan_callback, this, std::placeholders::_1));
        odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/bcr_bot/odom", 10,
            std::bind(&AmclLiteNode::odom_callback, this, std::placeholders::_1));
        map_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
            "/map", rclcpp::QoS(1).transient_local(),
            std::bind(&AmclLiteNode::map_callback, this, std::placeholders::_1));
        initial_pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
            "/initialpose", 1,
            std::bind(&AmclLiteNode::initial_pose_callback, this, std::placeholders::_1));
        pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(
            "amcl_lite_pose", 10);
        particles_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
            "/amcl_lite_particles", 10);
        
        // Add parameter for enabling/disabling dynamic object detection
        this->declare_parameter("enable_dynamic_detection", true);
        this->declare_parameter("dynamic_detection_threshold", 0.5);
        this->declare_parameter("dynamic_weight", 0.7);
        this->declare_parameter("sensor_sigma", 0.2);
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
    }
    void map_callback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
        std::cout << "Received OccupancyGrid with size: " << msg->data.size() << std::endl;
        likelihood_field_.fromOccupancyGrid(*msg);
        current_map_ = msg;  // Store map for dynamic object detection
        std::cout << "Likelihood field initialized." << std::endl;
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

    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_pub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr particles_pub_;
    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;
    ParticleFilter pf_;
    LikelihoodField likelihood_field_;
    DynamicObjectDetector dynamic_detector_;
    DynamicAwareSensorModel dynamic_sensor_model_;
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
