/**
 * @file lidar_node.cpp
 * @brief ROS 2 LiDAR driver node (with OpenCV visualization + outlier filtering)
 */

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>

#include <iostream>
#include <string>
#include <vector>
#include <cmath>
#include <algorithm>
#include <cstring>
#include <cerrno>
#include <limits>

// Linux serial low-level headers
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <asm/termbits.h>

// OpenCV headers
#include <opencv2/opencv.hpp>

// --- Configuration Macros ---
#define LIDAR_BAUDRATE 150000
#define RETRY_COUNT 3

class LidarNode : public rclcpp::Node
{
public:
    LidarNode() 
        : Node("lidar_node"), fd_(-1), is_shutdown_(false), last_point_angle_(0.0), scan_count_(0)
    {
        // 1. Declare and get parameters
        this->declare_parameter<std::string>("port_name", "/dev/ttyACM0");
        this->declare_parameter<std::string>("frame_id", "laser_link");
        
        // Filter-related parameters
        this->declare_parameter<bool>("filter.enabled", true);           // Enable filtering
        this->declare_parameter<double>("filter.radius", 0.10);         // Search radius (meters), default 10 cm
        this->declare_parameter<int>("filter.min_neighbors", 2);        // Minimum neighbors within radius (including itself)

        this->get_parameter("port_name", port_name_);
        this->get_parameter("frame_id", frame_id_);
        
        // Get filter parameters
        this->get_parameter("filter.enabled", filter_enabled_);
        this->get_parameter("filter.radius", filter_radius_);
        this->get_parameter("filter.min_neighbors", filter_min_neighbors_);

        // 2. Initialize publisher (ROS 2 style)
        scan_pub_ = this->create_publisher<sensor_msgs::msg::LaserScan>("scan", 10);
        full_scan_buffer_.reserve(1000);

        // 3. Initialize OpenCV visualization
        initVisualization();

        // 4. Open and configure serial port
        if (!openSerial(port_name_, LIDAR_BAUDRATE)) {
            RCLCPP_FATAL(this->get_logger(), "Failed to open serial port: %s", port_name_.c_str());
            exit(1);
        }

        // 5. Send start command (A5 60)
        sendCmd({0xA5, 0x60});
        RCLCPP_INFO(this->get_logger(), "LiDAR started. Port: %s", port_name_.c_str());
        RCLCPP_INFO(this->get_logger(), "Filter Config -> Enabled: %s, Radius: %.2fm, Min Neighbors: %d", 
            filter_enabled_ ? "true" : "false", filter_radius_, filter_min_neighbors_);
    }

    ~LidarNode()
    {
        shutdown();
        cv::destroyAllWindows();
    }

    /**
     * @brief Main loop of the node, called from main()
     */
    void run_loop()
    {
        uint8_t buffer[1024];
        // Control loop frequency using Rate
        rclcpp::Rate r(500); 

        while (rclcpp::ok() && !is_shutdown_)
        {
            int n = read(fd_, buffer, sizeof(buffer));
            if (n > 0)
            {
                for (int i = 0; i < n; i++)
                {
                    processByte(buffer[i]);
                }
            }
            else if (n < 0)
            {
                if (errno != EAGAIN) {
                    RCLCPP_WARN(this->get_logger(), "Serial read error: %s", strerror(errno));
                }
            }

            // OpenCV GUI event processing
            // cv::waitKey(1); 
            
            // ROS 2 callback processing
            rclcpp::spin_some(this->get_node_base_interface());
            
            r.sleep();
        }
    }

private:
    rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr scan_pub_;
    std::string port_name_;
    std::string frame_id_;
    
    // Filter control variables
    bool filter_enabled_;
    double filter_radius_;
    int filter_min_neighbors_;

    int fd_; 
    bool is_shutdown_;

    // Scan counter
    int scan_count_ = 0;

    // --- Parsing state machine ---
    enum State {
        WAIT_HEADER1,
        WAIT_HEADER2,
        READ_META,
        READ_PAYLOAD
    };
    
    State state_ = WAIT_HEADER1;
    std::vector<uint8_t> packet_buffer_; 
    uint8_t current_lsn_ = 0;
    size_t target_payload_size_ = 0;

    // --- Point cloud processing ---
    struct LidarPoint
    {
        double angle_rad;   // Angle in radians
        double distance_m;  // Distance in meters
        // Precomputed Cartesian coordinates for faster calculations
        double x; 
        double y;
    };
    std::vector<LidarPoint> full_scan_buffer_;
    double last_point_angle_;
    rclcpp::Time scan_start_time_;
    bool first_packet_of_scan_ = true;

    // --- OpenCV visualization ---
    cv::Mat map_image_;
    const int img_size_ = 800;    
    const double max_range_ = 3.0; 
    double px_scale_;             
    const std::string win_name_ = "Lidar Scan Monitor";

    // ================= Initialization & Utilities =================

    void initVisualization()
    {
        map_image_ = cv::Mat::zeros(img_size_, img_size_, CV_8UC3);
        px_scale_ = (img_size_ / 2.0) / max_range_;
    }

    void resetImage()
    {
        map_image_ = cv::Scalar(0, 0, 0);
        cv::line(map_image_, cv::Point(img_size_/2, 0), cv::Point(img_size_/2, img_size_), cv::Scalar(50, 50, 50), 1);
        cv::line(map_image_, cv::Point(0, img_size_/2), cv::Point(img_size_, img_size_/2), cv::Scalar(50, 50, 50), 1);
        
        // Display filter status on screen
        std::string status = filter_enabled_ ? "Filter: ON" : "Filter: OFF";
        cv::putText(map_image_, status, cv::Point(10, 30), cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(0, 255, 0), 2);
    }

    // ================= Serial Low-Level Operations =================

    bool openSerial(const std::string& port, int baudrate)
    {
        fd_ = open(port.c_str(), O_RDWR | O_NOCTTY | O_NDELAY);
        if (fd_ == -1) {
            RCLCPP_ERROR(this->get_logger(), "Open Error: %s", strerror(errno));
            return false;
        }

        struct termios2 tio;
        if (ioctl(fd_, TCGETS2, &tio) != 0) {
            close(fd_);
            return false;
        }

        tio.c_cflag &= ~CBAUD;
        tio.c_cflag |= BOTHER;
        tio.c_ispeed = baudrate;
        tio.c_ospeed = baudrate;

        tio.c_cflag &= ~PARENB;
        tio.c_cflag &= ~CSTOPB;
        tio.c_cflag &= ~CSIZE;
        tio.c_cflag |= CS8;

        tio.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
        tio.c_iflag &= ~(IXON | IXOFF | IXANY);
        tio.c_oflag &= ~OPOST;

        if (ioctl(fd_, TCSETS2, &tio) != 0) {
            close(fd_);
            return false;
        }

        ioctl(fd_, TCFLSH, TCIOFLUSH);
        return true;
    }

    void shutdown()
    {
        if (is_shutdown_) return;
        is_shutdown_ = true;

        if (fd_ != -1)
        {
            std::vector<uint8_t> stop_cmd = {0xA5, 0x00, 0xA5, 0x65, 0xA5, 0x65};
            write(fd_, stop_cmd.data(), stop_cmd.size());
            ioctl(fd_, TCSBRK, 1);
            close(fd_);
            fd_ = -1;
        }
    }

    void sendCmd(const std::vector<uint8_t>& cmd)
    {
        if (fd_ != -1) {
            write(fd_, cmd.data(), cmd.size());
        }
    }

    // ================= Protocol Parsing State Machine =================

    void processByte(uint8_t byte)
    {
        switch (state_)
        {
        case WAIT_HEADER1:
            if (byte == 0xAA) {
                state_ = WAIT_HEADER2;
                packet_buffer_.clear();
                packet_buffer_.push_back(byte);
            }
            break;

        case WAIT_HEADER2:
            if (byte == 0x55) {
                state_ = READ_META;
                packet_buffer_.push_back(byte);
            } else if (byte == 0xAA) {
                state_ = WAIT_HEADER2; 
                packet_buffer_.clear();
                packet_buffer_.push_back(byte);
            } else {
                state_ = WAIT_HEADER1;
                packet_buffer_.clear();
            }
            break;

        case READ_META:
            packet_buffer_.push_back(byte);
            if (packet_buffer_.size() == 4) {
                current_lsn_ = packet_buffer_[3];
                target_payload_size_ = 4 + (current_lsn_ * 3);
                state_ = READ_PAYLOAD;
            }
            break;

        case READ_PAYLOAD:
            packet_buffer_.push_back(byte);
            if (packet_buffer_.size() == 4 + target_payload_size_) {
                parsePacket(packet_buffer_);
                state_ = WAIT_HEADER1; 
            }
            break;
        }
    }

    // ================= Point Cloud Conversion Algorithms =================
    
    inline uint16_t bytesToUint16(const std::vector<uint8_t> &data, size_t index)
    {
        return (static_cast<uint16_t>(data[index + 1]) << 8) | data[index];
    }

    void parsePacket(const std::vector<uint8_t> &packet_data)
    {
        if (first_packet_of_scan_) {
            scan_start_time_ = this->now(); 
            first_packet_of_scan_ = false;
        }

        uint8_t lsn = packet_data[3];
        if (lsn == 0) return;

        uint16_t fsangle_raw = bytesToUint16(packet_data, 4);
        uint16_t lsangle_raw = bytesToUint16(packet_data, 6);

        double angle_start_deg = static_cast<double>(fsangle_raw >> 1) / 64.0;
        double angle_end_deg = static_cast<double>(lsangle_raw >> 1) / 64.0;

        double diff_angle_deg = 0.0;
        if (lsn > 1) {
            diff_angle_deg = angle_end_deg - angle_start_deg;
            if (diff_angle_deg < 0) {
                diff_angle_deg += 360.0;
            }
        }

        for (int i = 0; i < lsn; ++i)
        {
           size_t offset = 8 + i * 3;
           if (offset + 1 >= packet_data.size()) break; 

           uint16_t dist_raw = bytesToUint16(packet_data, offset);
           
           double distance_m = static_cast<double>(dist_raw) / 4.0 / 1000.0;
           double distance_mm = static_cast<double>(dist_raw) / 4.0;

           double angle_deg = angle_start_deg;
           if (lsn > 1) {
               angle_deg = (diff_angle_deg / (lsn - 1)) * i + angle_start_deg;
           }

           double angle_correct_deg = 0.0;
           if (distance_mm != 0) {
               double numerator = 21.8 * (155.3 - distance_mm);
               double denominator = 155.3 * distance_mm;
               double angle_correct_rad = std::atan(numerator / denominator);
               angle_correct_deg = angle_correct_rad * 180.0 / M_PI;
           }

           double final_angle_deg = angle_deg + angle_correct_deg;
           final_angle_deg = std::fmod(final_angle_deg, 360.0);
           if (final_angle_deg < 0) final_angle_deg += 360.0;

           double angle_rad = M_PI * final_angle_deg / 180.0;

            if (distance_m > 0.01) 
            {
                // Note: OpenCV draws raw data (including noise),
                // because filtering is applied after a full scan is completed.
                if (distance_m <= max_range_) 
                {
                    int center_xy = img_size_ / 2;
                    int px = center_xy + static_cast<int>(distance_m * px_scale_ * std::sin(angle_rad));
                    int py = center_xy - static_cast<int>(distance_m * px_scale_ * std::cos(angle_rad));

                    if (px >= 0 && px < img_size_ && py >= 0 && py < img_size_) {
                        cv::circle(map_image_, cv::Point(px, py), 1, cv::Scalar(0, 0, 255), -1);
                    }
                }

                // Detect wraparound (end of one full revolution)
                if (angle_rad < last_point_angle_ - M_PI) 
                {
                    if (scan_count_ > 0) {
                        // Call publish function; filtering is performed inside it
                        publishScan();
                        // cv::imshow(win_name_, map_image_);
                    } else {
                        RCLCPP_INFO(this->get_logger(), "Skipping first partial scan...");
                    }
                    
                    scan_count_++;
                    resetImage();
                    full_scan_buffer_.clear();
                    first_packet_of_scan_ = true;
                    scan_start_time_ = this->now();
                }

                LidarPoint p;
                p.angle_rad = angle_rad;
                p.distance_m = distance_m;
                // Precompute coordinates to speed up later filtering
                p.x = distance_m * std::cos(angle_rad);
                p.y = distance_m * std::sin(angle_rad);
                
                full_scan_buffer_.push_back(p);
                last_point_angle_ = angle_rad;
            }
        }
    }

    /**
     * @brief Radius-based outlier removal
     * Iterates through all points. If a point has fewer than
     * min_neighbors within the given radius, it is treated as noise and removed.
     */
    void removeOutliers(std::vector<LidarPoint>& points)
    {
        if (points.empty()) return;

        std::vector<LidarPoint> clean_points;
        clean_points.reserve(points.size());

        double r2 = filter_radius_ * filter_radius_; // Compare squared distance to avoid sqrt

        // Brute-force double loop is acceptable for a few hundred points per scan
        for (size_t i = 0; i < points.size(); ++i) {
            int neighbors = 0;
            const auto& p1 = points[i];

            for (size_t j = 0; j < points.size(); ++j) {
                double dx = p1.x - points[j].x;
                double dy = p1.y - points[j].y;
                if ((dx*dx + dy*dy) < r2) {
                    neighbors++;
                }
                if (neighbors >= filter_min_neighbors_) break;
            }

            // Keep the point if enough neighbors exist (neighbors includes the point itself)
            if (neighbors >= filter_min_neighbors_) {
                clean_points.push_back(p1);
            }
        }

        points = std::move(clean_points);
    }

    void publishScan()
    {
        if (full_scan_buffer_.empty()) return;

        // Apply filtering before converting to ROS message
        if (filter_enabled_) {
            size_t raw_size = full_scan_buffer_.size();
            removeOutliers(full_scan_buffer_);
            // RCLCPP_INFO(this->get_logger(), "Filtered: %lu -> %lu", raw_size, full_scan_buffer_.size());
        }

        sensor_msgs::msg::LaserScan scan;
        scan.header.stamp = (scan_start_time_.nanoseconds() == 0) ? this->now() : scan_start_time_;
        scan.header.frame_id = frame_id_;

        scan.angle_min = 0.0;
        scan.angle_max = 2.0 * M_PI;
        
        const int SCAN_SIZE = 720; 
        scan.angle_increment = (2.0 * M_PI) / SCAN_SIZE;
        
        scan.range_min = 0.05;
        scan.range_max = 8.0;

        scan.ranges.assign(SCAN_SIZE, std::numeric_limits<float>::infinity());

        for (const auto& point : full_scan_buffer_)
        {
            double raw_angle = point.angle_rad;
            // ROS LaserScan defines CCW as positive; some LiDARs output CW, keeping original logic
            double index_angle = 2.0 * M_PI - raw_angle;
            
            if (index_angle >= 2.0 * M_PI) index_angle -= 2.0 * M_PI;
            if (index_angle < 0) index_angle += 2.0 * M_PI;

            int index = static_cast<int>(index_angle / scan.angle_increment);

            if (index >= 0 && index < SCAN_SIZE)
            {
                if (scan.ranges[index] == std::numeric_limits<float>::infinity() || 
                    point.distance_m < scan.ranges[index])
                {
                    scan.ranges[index] = static_cast<float>(point.distance_m);
                }
            }
        }

        scan.scan_time = 1.0 / 7.0; 
        scan.time_increment = scan.scan_time / SCAN_SIZE;

        scan_pub_->publish(scan);
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<LidarNode>();
    try {
        node->run_loop();
    }
    catch (const std::exception &e) {
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Exception: %s", e.what());
    }
    rclcpp::shutdown();
    return 0;
}
