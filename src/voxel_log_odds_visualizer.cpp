#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <tf2_ros/transform_listener.hpp>
#include <tf2_ros/buffer.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <unordered_map>
#include <atomic>
#include <mutex>
#include <cmath>
#include <fstream>
#include <iomanip>
#include <Eigen/Dense>

using namespace std::chrono_literals;

struct VoxelKey {
    int x, y, z;
    bool operator==(const VoxelKey &other) const {
        return x == other.x && y == other.y && z == other.z;
    }
};

struct KeyHash {
    std::size_t operator()(const VoxelKey &k) const {
        return ((std::hash<int>()(k.x) ^
                (std::hash<int>()(k.y) << 1)) >> 1) ^
               (std::hash<int>()(k.z) << 1);
    }
};
    

class VoxelLogOddsVisualizer : public rclcpp::Node {
public:
    //Constructor
    VoxelLogOddsVisualizer() : Node("voxel_logodds_visualizer") {
        loadParams();
        setupROS();
    }

    void loadParams() {
        // --- Frames ---
        this->declare_parameter<std::string>("frame_id", "map");
        this->get_parameter("frame_id", frame_id_);

        this->declare_parameter<std::string>("robot_frame_id", "base_link");
        this->get_parameter("robot_frame_id", robot_frame_id_);

        // --- Topics ---
        this->declare_parameter<std::vector<std::string>>("pointcloud_sub_topics", {"/pointcloud"});
        this->get_parameter("pointcloud_sub_topics", sub_pointcloud_topics_);

        this->declare_parameter<std::string>("odometry_sub_topic", "/odometry");
        this->get_parameter("odometry_sub_topic", sub_odometry_topic_);

        this->declare_parameter<std::string>("voxel_pub_topic", "/occupancy_grid");
        this->get_parameter("voxel_pub_topic", pub_voxel_topic_);

        this->declare_parameter<std::string>("global_costmap_pub_topic", "/global_costmap_2d");
        this->get_parameter("global_costmap_pub_topic", pub_global_costmap_topic_);

        this->declare_parameter<std::string>("local_costmap_pub_topic", "/local_costmap_2d");
        this->get_parameter("local_costmap_pub_topic", pub_local_costmap_topic_);

        // --- Voxel map ---
        this->declare_parameter<double>("voxel_resolution", 1.0);
        this->get_parameter("voxel_resolution", voxel_res_);

        this->declare_parameter<double>("logodds_min", -5.0);
        this->get_parameter("logodds_min", logodds_min_);

        this->declare_parameter<double>("logodds_max", 5.0);
        this->get_parameter("logodds_max", logodds_max_);

        this->declare_parameter<double>("probability_threshold", 0.1);
        this->get_parameter("probability_threshold", prob_threshold_);

        // --- Costmaps ---
        this->declare_parameter<double>("global_costmap_dimension", 400.0);
        this->get_parameter("global_costmap_dimension", global_costmap_dim_);

        this->declare_parameter<double>("local_costmap_dimension", 100.0);
        this->get_parameter("local_costmap_dimension", local_costmap_dim_);

        this->declare_parameter<double>("depth_deviation", 1.0);
        this->get_parameter("depth_deviation", depth_deviation_);

        this->declare_parameter<double>("z_cutoff", -1.0);
        this->get_parameter("z_cutoff", z_cutoff_);

        // --- Decay ---
        this->declare_parameter<double>("decay_time", -1.0);
        this->get_parameter("decay_time", decay_time_);

        // --- PCD export ---
        this->declare_parameter<bool>("save_pcd", false);
        this->get_parameter("save_pcd", save_pcd_);

        this->declare_parameter<std::string>("output_pcd_file", "occupancy_grid.pcd");
        this->get_parameter("output_pcd_file", output_pcd_file_);

        this->declare_parameter<double>("map_publish_rate", 5.0);
        this->get_parameter("map_publish_rate", map_publish_rate_);

        n_voxels_ = static_cast<int>(std::ceil(global_costmap_dim_ / voxel_res_));
        half_grid_ = global_costmap_dim_ / 2.0;
    }

    void setupROS() {
        sensor_cb_group_  = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
        decay_cb_group_   = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
        publish_cb_group_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

        rclcpp::SubscriptionOptions sensor_opts;
        sensor_opts.callback_group = sensor_cb_group_;

        for (const auto& topic : sub_pointcloud_topics_) {
            pc_subs_.push_back(this->create_subscription<sensor_msgs::msg::PointCloud2>(
                topic, 10,
                std::bind(&VoxelLogOddsVisualizer::pcCallback, this, std::placeholders::_1),
                sensor_opts
            ));
        }

        odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            sub_odometry_topic_, 10,
            std::bind(&VoxelLogOddsVisualizer::odometryCallback, this, std::placeholders::_1),
            sensor_opts
        );

        prob_cloud_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
           pub_voxel_topic_, 10
        );

        global_ogm_pub_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>(pub_global_costmap_topic_, 10);
        local_ogm_pub_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>(pub_local_costmap_topic_, 10);

        tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

        if (decay_time_ > 0.0) {
            decay_timer_ = this->create_wall_timer(1s,
                std::bind(&VoxelLogOddsVisualizer::decayCallback, this),
                decay_cb_group_);
            RCLCPP_INFO(this->get_logger(), "Voxel decay enabled: %.1f s", decay_time_);
        }

        auto publish_ms = std::chrono::milliseconds(static_cast<int>(1000.0 / map_publish_rate_));
        publish_timer_ = this->create_wall_timer(
            publish_ms,
            std::bind(&VoxelLogOddsVisualizer::publishAll, this),
            publish_cb_group_);

        RCLCPP_INFO(this->get_logger(), "VoxelLogOddsVisualizer initialized. Grid size: %.2fm, resolution: %.2fm",
                    global_costmap_dim_, voxel_res_);
    }

    //Destructor
    ~VoxelLogOddsVisualizer() {
        if (save_pcd_) {
            RCLCPP_INFO(this->get_logger(), "Shutting down, saving PCD file...");
            savePCD(output_pcd_file_);
        }
    }

private:
    // --- Frames ---
    std::string frame_id_;
    std::string robot_frame_id_;

    // --- Topics ---
    std::vector<std::string> sub_pointcloud_topics_;
    std::string sub_odometry_topic_;
    std::string pub_voxel_topic_;
    std::string pub_global_costmap_topic_;
    std::string pub_local_costmap_topic_;

    // --- Voxel map ---
    double voxel_res_, logodds_min_, logodds_max_;
    double prob_threshold_;
    double half_grid_;
    int n_voxels_;
    std::unordered_map<VoxelKey, double, KeyHash> logodds_grid_;
    std::unordered_map<VoxelKey, rclcpp::Time, KeyHash> last_seen_;

    // --- Costmaps ---
    double global_costmap_dim_;   // meters
    double local_costmap_dim_;    // meters
    double depth_deviation_;
    double z_cutoff_;
    std::atomic<double> vehicle_z_{0.0};
    double decay_time_{-1.0};     // seconds a voxel survives without a new hit; -1 = disabled
    double map_publish_rate_{5.0};

    // --- PCD export ---
    bool save_pcd_;
    std::string output_pcd_file_;

    std::vector<rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr> pc_subs_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr prob_cloud_pub_;
    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr global_ogm_pub_;
    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr local_ogm_pub_;
    rclcpp::TimerBase::SharedPtr decay_timer_;
    rclcpp::TimerBase::SharedPtr publish_timer_;

    rclcpp::CallbackGroup::SharedPtr sensor_cb_group_;
    rclcpp::CallbackGroup::SharedPtr decay_cb_group_;
    rclcpp::CallbackGroup::SharedPtr publish_cb_group_;

    std::mutex map_mutex_;

    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

    inline double to_logodds(double p) const {
        return std::log(p / (1.0 - p));
    }

    inline double to_prob(double logodds) const {
        return 1.0 / (1.0 + std::exp(-logodds));
    }
 
    void savePCD(const std::string& filename) {
        std::ofstream pcd_file(filename);
        if (!pcd_file.is_open()) {
            RCLCPP_ERROR(this->get_logger(), "Failed to open PCD file: %s", filename.c_str());
            return;
        }

        // Count valid voxels (above threshold)
        size_t num_voxels = 0;
        for (const auto& kv : logodds_grid_) {
            double prob = to_prob(kv.second);
            if (prob >= prob_threshold_) {
                num_voxels++;
            }
        }

        // Write PCD header
        pcd_file << "# .PCD v.7 - Point Cloud Data file format\n";
        pcd_file << "VERSION .7\n";
        pcd_file << "FIELDS x y z rgb occupancy\n";
        pcd_file << "SIZE 4 4 4 4 4\n";
        pcd_file << "TYPE F F F U F\n";
        pcd_file << "COUNT 1 1 1 1 1\n";
        pcd_file << "WIDTH " << num_voxels << "\n";
        pcd_file << "HEIGHT 1\n";
        pcd_file << "VIEWPOINT 0 0 0 1 0 0 0\n";
        pcd_file << "POINTS " << num_voxels << "\n";
        pcd_file << "DATA ascii\n";

        // Write voxel data
        for (const auto& kv : logodds_grid_) {
            const VoxelKey& key = kv.first;
            double logodds = kv.second;
            double prob = to_prob(logodds);

            if (prob < prob_threshold_) continue;

            // Calculate voxel center in world coordinates
            float x = key.x * voxel_res_ - half_grid_ + voxel_res_ / 2.0f;
            float y = key.y * voxel_res_ - half_grid_ + voxel_res_ / 2.0f;
            float z = key.z * voxel_res_ - half_grid_ + voxel_res_ / 2.0f;

            // Calculate color (red = occupied, blue = free)
            uint8_t red = static_cast<uint8_t>(prob * 255);
            uint8_t green = 0;
            uint8_t blue = static_cast<uint8_t>((1.0 - prob) * 255);

            // Pack RGB into single 32-bit integer
            uint32_t rgb = (static_cast<uint32_t>(red) << 16) |
                          (static_cast<uint32_t>(green) << 8) |
                          static_cast<uint32_t>(blue);

            pcd_file << std::fixed << std::setprecision(6)
                    << x << " " << y << " " << z << " "
                    << rgb << " "
                    << prob << "\n";
        }

        pcd_file.close();
        RCLCPP_INFO(this->get_logger(), "Saved %zu voxels to %s", num_voxels, filename.c_str());
    }

    void odometryCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
        vehicle_z_.store(msg->pose.pose.position.z);
    }

    void pcCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
        // Convert to frame_id_
        geometry_msgs::msg::TransformStamped trans;
        try {
            trans = tf_buffer_->lookupTransform(
                frame_id_, msg->header.frame_id, msg->header.stamp, 100ms
            );
        } catch (tf2::TransformException &ex) {
            RCLCPP_WARN(this->get_logger(), "TF lookup failed: %s", ex.what());
            return;
        }

        // Convert PointCloud2 → xyz + intensity
        sensor_msgs::PointCloud2ConstIterator<float> iter_x(*msg, "x");
        sensor_msgs::PointCloud2ConstIterator<float> iter_y(*msg, "y");
        sensor_msgs::PointCloud2ConstIterator<float> iter_z(*msg, "z");
        sensor_msgs::PointCloud2ConstIterator<float> iter_i(*msg, "intensity");
        
        std::vector<Eigen::Vector4f> points;
        std::vector<float> sensor_model_prob;
        for (; iter_x != iter_x.end(); ++iter_x, ++iter_y, ++iter_z, ++iter_i) {
            if (std::isfinite(*iter_x) && std::isfinite(*iter_y) && std::isfinite(*iter_z)) {
                points.emplace_back(*iter_x, *iter_y, *iter_z, 1.0f);
                sensor_model_prob.push_back(*iter_i);
            }
        }

        if (points.empty()) return;

        // Transform points to map frame using TF
        Eigen::Matrix4f T = transformToMatrix(trans);

        std::lock_guard<std::mutex> lock(map_mutex_);
        for (size_t i = 0; i < points.size(); i++) {
            Eigen::Vector4f p_map = T * points[i];
            
            // Voxel indices
            int ix = static_cast<int>(std::floor((p_map.x() + half_grid_) / voxel_res_));
            int iy = static_cast<int>(std::floor((p_map.y() + half_grid_) / voxel_res_));
            int iz = static_cast<int>(std::floor((p_map.z() + half_grid_) / voxel_res_));

            if (ix < 0 || iy < 0 || iz < 0 || ix >= n_voxels_ || iy >= n_voxels_ || iz >= n_voxels_)
                continue;

            VoxelKey key{ix, iy, iz};
            
            // Bayesian log-odds update
            double logodds_measurement = to_logodds(sensor_model_prob[i]);
            double logodds_prior = to_logodds(0.5);
            double evidence = logodds_measurement - logodds_prior;
            
            if (logodds_grid_.find(key) != logodds_grid_.end()) {
                logodds_grid_[key] += evidence;
            } else {
                logodds_grid_[key] = evidence;
            }

            // Clip
            logodds_grid_[key] = std::min(std::max(logodds_grid_[key], logodds_min_), logodds_max_);

            if (decay_time_ > 0.0) {
                last_seen_[key] = this->get_clock()->now();
            }

        }

    }

    void publishAll() {
        publishVoxelMap();
        publishGlobalCostmap();
        publishLocalCostmap();
    }

    void publishVoxelMap() {
        std::lock_guard<std::mutex> lock(map_mutex_);
        // Count valid voxels
        size_t num_voxels = 0;
        for (const auto& kv : logodds_grid_) {
            double prob = to_prob(kv.second);
            if (prob >= prob_threshold_) {
                num_voxels++;
            }
        }

        if (num_voxels == 0) return;

        // Create PointCloud2 message
        sensor_msgs::msg::PointCloud2 cloud_msg;
        cloud_msg.header.stamp = this->get_clock()->now();
        cloud_msg.header.frame_id = frame_id_;
        cloud_msg.height = 1;
        cloud_msg.width = num_voxels;
        cloud_msg.is_dense = true;
        cloud_msg.is_bigendian = false;

        // // Define fields: x, y, z, intensity
        sensor_msgs::PointCloud2Modifier modifier(cloud_msg);
        modifier.setPointCloud2Fields(4,
            "x", 1, sensor_msgs::msg::PointField::FLOAT32,
            "y", 1, sensor_msgs::msg::PointField::FLOAT32,
            "z", 1, sensor_msgs::msg::PointField::FLOAT32,
            "intensity", 1, sensor_msgs::msg::PointField::FLOAT32
        );
        
        modifier.resize(num_voxels);
        
        cloud_msg.point_step = 16;  // 12 (xyz) + 4 (intensity)
        cloud_msg.row_step = cloud_msg.point_step * num_voxels;
        cloud_msg.data.resize(cloud_msg.row_step);

        // Create iterators
        sensor_msgs::PointCloud2Iterator<float> iter_x(cloud_msg, "x");
        sensor_msgs::PointCloud2Iterator<float> iter_y(cloud_msg, "y");
        sensor_msgs::PointCloud2Iterator<float> iter_z(cloud_msg, "z");
        sensor_msgs::PointCloud2Iterator<float> iter_intensity(cloud_msg, "intensity");

        // Fill point cloud data
        for (const auto& kv : logodds_grid_) {
            const VoxelKey& key = kv.first;
            double logodds = kv.second;
            double prob = to_prob(logodds);

            if (prob < prob_threshold_) continue;

            // Calculate voxel center in world coordinates
            *iter_x = key.x * voxel_res_ - half_grid_ + voxel_res_ / 2.0f;
            *iter_y = key.y * voxel_res_ - half_grid_ + voxel_res_ / 2.0f;
            *iter_z = key.z * voxel_res_ - half_grid_ + voxel_res_ / 2.0f;

            // Set intensity as probability (0-1 range)
            *iter_intensity = static_cast<float>(prob);

            ++iter_x;
            ++iter_y;
            ++iter_z;
            ++iter_intensity;
        }

        prob_cloud_pub_->publish(cloud_msg);
    }

    // Helper: compute depth band bounds
    std::pair<double,double> depthBand() const {
        const double vz = vehicle_z_.load();
        return {vz - depth_deviation_,
                std::min(vz + depth_deviation_, z_cutoff_)};
    }

    // Helper: check z filtering for a voxel
    bool voxelPassesDepthFilter(const VoxelKey& key, double depth_lo, double depth_hi) const {
        float z_world = key.z * voxel_res_ - half_grid_ + voxel_res_ / 2.0f;
        return z_world <= z_cutoff_ && z_world >= depth_lo && z_world <= depth_hi;
    }

    void publishGlobalCostmap() {
        std::lock_guard<std::mutex> lock(map_mutex_);
        const int cells = static_cast<int>(std::ceil(global_costmap_dim_ / voxel_res_));
        const double half = global_costmap_dim_ / 2.0;

        nav_msgs::msg::OccupancyGrid og;
        og.header.stamp = this->get_clock()->now();
        og.header.frame_id = frame_id_;
        og.info.resolution = static_cast<float>(voxel_res_);
        og.info.width  = cells;
        og.info.height = cells;
        og.info.origin.position.x = -half;
        og.info.origin.position.y = -half;
        og.info.origin.orientation.w = 1.0;
        og.data.assign(cells * cells, -1);

        auto [depth_lo, depth_hi] = depthBand();

        for (const auto& kv : logodds_grid_) {
            const VoxelKey& key = kv.first;

            double prob = to_prob(kv.second);
            if (prob < prob_threshold_) continue;
            if (!voxelPassesDepthFilter(key, depth_lo, depth_hi)) continue;

            float x_world = key.x * voxel_res_ - half_grid_ + voxel_res_ / 2.0f;
            float y_world = key.y * voxel_res_ - half_grid_ + voxel_res_ / 2.0f;

            int gx = static_cast<int>(std::floor((x_world + half) / voxel_res_));
            int gy = static_cast<int>(std::floor((y_world + half) / voxel_res_));
            if (gx < 0 || gy < 0 || gx >= cells || gy >= cells) continue;

            og.data[gy * cells + gx] = static_cast<int8_t>(std::round(prob * 100.0));
        }

        global_ogm_pub_->publish(og);
    }

    void publishLocalCostmap() {
        std::lock_guard<std::mutex> lock(map_mutex_);
        const int cells = static_cast<int>(std::ceil(local_costmap_dim_ / voxel_res_));
        const double half = local_costmap_dim_ / 2.0;

        // Look up robot position in the voxel-map frame
        geometry_msgs::msg::TransformStamped robot_trans;
        try {
            robot_trans = tf_buffer_->lookupTransform(
                frame_id_, robot_frame_id_, tf2::TimePointZero, 100ms
            );
        } catch (tf2::TransformException &ex) {
            RCLCPP_WARN(this->get_logger(), "Local costmap TF lookup failed: %s", ex.what());
            return;
        }

        const double robot_x = robot_trans.transform.translation.x;
        const double robot_y = robot_trans.transform.translation.y;
        // Round to 1 precision point.
        const double origin_x = std::round((robot_x - half) * 10.0) / 10.0;
        const double origin_y = std::round((robot_y - half) * 10.0) / 10.0;

        nav_msgs::msg::OccupancyGrid og;
        og.header.stamp = this->get_clock()->now();
        og.header.frame_id = frame_id_;
        og.info.resolution = static_cast<float>(voxel_res_);
        og.info.width  = cells;
        og.info.height = cells;
        og.info.origin.position.x = origin_x;
        og.info.origin.position.y = origin_y;
        og.info.origin.orientation.w = 1.0;
        og.data.assign(cells * cells, -1);

        auto [depth_lo, depth_hi] = depthBand();

        for (const auto& kv : logodds_grid_) {
            const VoxelKey& key = kv.first;

            double prob = to_prob(kv.second);
            if (prob < prob_threshold_) continue;
            if (!voxelPassesDepthFilter(key, depth_lo, depth_hi)) continue;

            float x_world = key.x * voxel_res_ - half_grid_ + voxel_res_ / 2.0f;
            float y_world = key.y * voxel_res_ - half_grid_ + voxel_res_ / 2.0f;

            int lx = static_cast<int>(std::floor((x_world - origin_x) / voxel_res_));
            int ly = static_cast<int>(std::floor((y_world - origin_y) / voxel_res_));
            if (lx < 0 || ly < 0 || lx >= cells || ly >= cells) continue;

            og.data[ly * cells + lx] = static_cast<int8_t>(std::round(prob * 100.0));
        }

        local_ogm_pub_->publish(og);
    }

    void decayCallback() {
        std::lock_guard<std::mutex> lock(map_mutex_);
        const rclcpp::Time now = this->get_clock()->now();
        const rclcpp::Duration threshold = rclcpp::Duration::from_seconds(decay_time_);

        std::vector<VoxelKey> to_erase;
        for (const auto& kv : last_seen_) {
            if ((now - kv.second) >= threshold) {
                to_erase.push_back(kv.first);
            }
        }
        for (const auto& key : to_erase) {
            logodds_grid_.erase(key);
            last_seen_.erase(key);
        }
    }

    Eigen::Matrix4f transformToMatrix(const geometry_msgs::msg::TransformStamped &trans) {
        Eigen::Quaternionf q(trans.transform.rotation.w,
                             trans.transform.rotation.x,
                             trans.transform.rotation.y,
                             trans.transform.rotation.z);
        Eigen::Vector3f t(trans.transform.translation.x,
                          trans.transform.translation.y,
                          trans.transform.translation.z);
        Eigen::Matrix4f T = Eigen::Matrix4f::Identity();
        T.block<3,3>(0,0) = q.toRotationMatrix();
        T.block<3,1>(0,3) = t;
        return T;
    }
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<VoxelLogOddsVisualizer>();
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node);
    executor.spin();
    rclcpp::shutdown();
    return 0;
}