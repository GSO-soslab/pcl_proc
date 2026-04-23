#include <cmath>
#include <vector>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <ping360_msgs/msg/sonar_echo.hpp>
#include <std_msgs/msg/float32.hpp>

using std::placeholders::_1;
// Distance across elevation : tan(12.5)*2*50  = 22.1m
// Distance across azimuth: tan(1)*2*50 = 1.7m
class MSISVoxels : public rclcpp::Node
{
public:
    MSISVoxels()
    : Node("msis_voxel_node")
    {
        // Parameters
        this->declare_parameter("range_max", 50.0);
        this->declare_parameter("horizontal_fov_deg", 2.0);
        this->declare_parameter("vertical_fov_deg", 25.0);
        this->declare_parameter("resolution", 1.0);
        this->declare_parameter<std::string>("marker_topic");
        this->declare_parameter("stonefish", false);
        this->declare_parameter<std::string>("cloud_sub_topic", "");
        this->declare_parameter<std::string>("sensor_frame_id", "alpha_rise/ping360_link");
        this->declare_parameter<std::string>("echo_sub_topic", "/alpha_rise/msis/echo");

        this->get_parameter("range_max", max_range_);
        double h_fov_deg, v_fov_deg;
        this->get_parameter("horizontal_fov_deg", h_fov_deg);
        this->get_parameter("vertical_fov_deg", v_fov_deg);
        this->get_parameter("resolution", resolution_);
        this->get_parameter("sensor_frame_id", frame_id_);
        std::string marker_topic;
        this->get_parameter("marker_topic", marker_topic);
        bool stonefish;
        this->get_parameter("stonefish", stonefish);

        h_fov_ = h_fov_deg * M_PI / 180.0;
        v_fov_ = v_fov_deg * M_PI / 180.0;

        marker_pub_ = this->create_publisher<visualization_msgs::msg::Marker>(marker_topic, 10);

        if (stonefish) {
            std::string cloud_topic;
            this->get_parameter("cloud_sub_topic", cloud_topic);
            angle_sub_ = this->create_subscription<std_msgs::msg::Float32>(
                cloud_topic + "/current_angle", 10, std::bind(&MSISVoxels::angle_callback, this, _1));
        } else {
            std::string echo_topic;
            this->get_parameter("echo_sub_topic", echo_topic);
            echo_sub_ = this->create_subscription<ping360_msgs::msg::SonarEcho>(
                echo_topic, 10, std::bind(&MSISVoxels::echo_callback, this, _1));
        }
    }

private:
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_pub_;
    rclcpp::Subscription<ping360_msgs::msg::SonarEcho>::SharedPtr echo_sub_;
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr angle_sub_;
    
    double max_range_;
    double h_fov_;
    double v_fov_;
    double resolution_;
    std::string frame_id_;

    // ---------------- Yaw rotation ----------------
    void rotate_yaw(double x, double y, double z, double yaw,
                    double &x_r, double &y_r, double &z_r)
    {
        double c = cos(yaw);
        double s = sin(yaw);
        x_r = c * x - s * y;
        y_r = s * x + c * y;
        z_r = z;
    }

    // ---------------- Sonar callback ----------------
    void echo_callback(const ping360_msgs::msg::SonarEcho::SharedPtr msg)
    {
        double yaw = msg->angle;
        voxels(yaw);
        // marker_outline(yaw); // optional
    }

    // ---------------- Sim callbacks ----------------
    void angle_callback(const std_msgs::msg::Float32::SharedPtr msg)
    {
        voxels(static_cast<double>(msg->data));
    }

    // ---------------- Voxel grid ----------------
    void voxels(double yaw)
    {
        visualization_msgs::msg::Marker marker;
        marker.header.frame_id = frame_id_;
        // marker.header.stamp = this->now();
        marker.ns = "fov_voxels";
        marker.id = 0;
        marker.type = visualization_msgs::msg::Marker::CUBE_LIST;
        marker.action = visualization_msgs::msg::Marker::ADD;

        marker.scale.x = resolution_;
        marker.scale.y = resolution_;
        marker.scale.z = resolution_;

        marker.color.r = 0.0;
        marker.color.g = 1.0;
        marker.color.b = 0.0;
        marker.color.a = 0.3;

        int x_steps = static_cast<int>(std::ceil(max_range_ / resolution_));

        for (int ix = 0; ix < x_steps; ix++)
        {
            double x = (ix + 0.5) * resolution_;
            if (x > max_range_)
                continue;

            double z_limit = x * tan(v_fov_ / 2.0);
            int z_steps = static_cast<int>(std::ceil((2.0 * z_limit) / resolution_));

            for (int iz = -z_steps / 2; iz <= z_steps / 2; iz++)
            {
                double z = iz * resolution_;
                if (std::abs(z) > z_limit + 0.5 * resolution_)
                    continue;

                double x_r, y_r, z_r;
                rotate_yaw(x, 0.0, z, yaw, x_r, y_r, z_r);

                geometry_msgs::msg::Point p;
                p.x = x_r;
                p.y = y_r;
                p.z = z_r;
                marker.points.push_back(p);
            }
        }

        marker_pub_->publish(marker);
    }

    // ---------------- FOV outline ----------------
    void marker_outline(double yaw)
    {
        visualization_msgs::msg::Marker marker;
        marker.header.frame_id = frame_id_;
        // marker.header.stamp = this->now();
        marker.ns = "line";
        marker.id = 0;
        marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
        marker.action = visualization_msgs::msg::Marker::ADD;
        marker.scale.x = 0.05;
        marker.color.r = 0.0;
        marker.color.g = 0.0;
        marker.color.b = 1.0;
        marker.color.a = 1.0;

        double length = max_range_;
        double v_half_extent = length * tan(v_fov_ / 2.0);
        double h_half_extent = length * tan(h_fov_ / 2.0);

        std::vector<std::array<double,3>> corners = {
            { length,  h_half_extent,  v_half_extent},  // top-right
            { length, -h_half_extent,  v_half_extent},  // top-left
            { length,  h_half_extent, -v_half_extent},  // bottom-right
            { length, -h_half_extent, -v_half_extent},  // bottom-left
        };

        geometry_msgs::msg::Point origin;
        origin.x = 0.0;
        origin.y = 0.0;
        origin.z = 0.0;

        for (auto &corner : corners)
        {
            double x_r, y_r, z_r;
            rotate_yaw(corner[0], corner[1], corner[2], yaw, x_r, y_r, z_r);

            marker.points.push_back(origin);
            geometry_msgs::msg::Point p;
            p.x = x_r;
            p.y = y_r;
            p.z = z_r;
            marker.points.push_back(p);
        }

        marker_pub_->publish(marker);
    }
};

// ---------------- Main ----------------
int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<MSISVoxels>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
