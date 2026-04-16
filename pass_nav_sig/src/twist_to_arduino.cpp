#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/imu.hpp> // <-- MODIFIED: Use standard IMU message
#include <string>
#include <vector>    // For parsing
#include <sstream>   // For parsing
#include <algorithm> // For std::max, std::min
#include <chrono>    // For 2s literal
#include <thread>    // For std::this_thread::sleep_for
#include <cerrno>    // For errno
#include <cstring>   // For strerror

// Headers for serial port communication (Linux/macOS)
#include <fcntl.h>   // For open()
#include <termios.h> // For serial port config (struct termios)
#include <unistd.h>  // For write(), close(), read()

using namespace std::chrono_literals;

class TwistToArduino : public rclcpp::Node
{
public:
    TwistToArduino() : Node("twist_to_arduino_cpp")
    {
        // Declare parameters
        this->declare_parameter<std::string>("serial_port", "/dev/ttyACM0");
        this->declare_parameter<int>("baud_rate", 115200);
        this->declare_parameter<double>("wheel_base", 0.258); 
        this->declare_parameter<double>("max_speed", 1.0);
        this->declare_parameter<std::string>("odom_frame_id", "odom");
        this->declare_parameter<std::string>("base_frame_id", "base_link");
        this->declare_parameter<std::string>("imu_frame_id", "imu"); // <-- ADDED: Frame ID for IMU

        // Get parameters
        serial_port_ = this->get_parameter("serial_port").as_string();
        baud_rate_ = this->get_parameter("baud_rate").as_int();
        wheel_base_ = this->get_parameter("wheel_base").as_double();
        max_speed_ = this->get_parameter("max_speed").as_double();
        odom_frame_id_ = this->get_parameter("odom_frame_id").as_string();
        base_frame_id_ = this->get_parameter("base_frame_id").as_string();
        imu_frame_id_ = this->get_parameter("imu_frame_id").as_string(); 

        // Initialize serial connection
        serial_ok_ = init_serial();

        // Create subscription to the /cmd_vel topic
        subscription_ = this->create_subscription<geometry_msgs::msg::Twist>(
            "cmd_vel", 10, std::bind(&TwistToArduino::twist_callback, this, std::placeholders::_1));
        
        // Create publisher for odometry
        odom_publisher_ = this->create_publisher<nav_msgs::msg::Odometry>("odom", 10);
        
        imu_publisher_ = this->create_publisher<sensor_msgs::msg::Imu>("imu", 10);

        // Create a timer to read from serial
        read_timer_ = this->create_wall_timer(
            30ms, std::bind(&TwistToArduino::read_serial_callback, this));
            
        RCLCPP_INFO(this->get_logger(), "Node initialized. Subscribing to /cmd_vel, publishing to /odom and /imu");
    }

    ~TwistToArduino()
    {
        if (serial_fd_ != -1) {
            // Send stop command and close port
            std::string stop_command = "L0,R0\n";
            write(serial_fd_, stop_command.c_str(), stop_command.length());
            close(serial_fd_);
        }
    }

    /**
     * @brief Public method to check if serial port was opened successfully.
     */
    bool is_serial_ok() const {
        return serial_ok_;
    }

private:
    /**
     * @brief Callback for /cmd_vel. Calculates diff drive and sends to Arduino.
     */
    void twist_callback(const geometry_msgs::msg::Twist::SharedPtr msg)
    {
        if (!serial_ok_) return; // Don't try to write if serial isn't open

        double linear_x = msg->linear.x;
        double angular_z = msg->angular.z;

        // Differential Drive Kinematics
        double right_vel = linear_x + (angular_z * wheel_base_ / 2.0);
        double left_vel = linear_x - (angular_z * wheel_base_ / 2.0);

        // Scaling and Clamping
        double left_cmd = (left_vel / max_speed_) * 127.0;
        double right_cmd = (right_vel / max_speed_) * 127.0;

        left_cmd = std::max(-127.0, std::min(left_cmd, 127.0));
        right_cmd = std::max(-127.0, std::min(right_cmd, 127.0));
       
        // Define the minimum command that will make the motor move
        const double MIN_CMD = 50.0;
        // Define the maximum possible command
        const double MAX_CMD = 127.0;

        double abs_l = std::abs(left_cmd);
        double abs_r = std::abs(right_cmd);
        
        // Find the largest scaling factor needed to get a motor moving
        double scale = 1.0;
        
        // If left motor is requested to move but is below threshold, calculate scale
        if (abs_l > 0.0 && abs_l < MIN_CMD) {
            scale = std::max(scale, MIN_CMD / abs_l);
        }
        
        // If right motor is requested to move but is below threshold, calculate scale
        if (abs_r > 0.0 && abs_r < MIN_CMD) {
            scale = std::max(scale, MIN_CMD / abs_r);
        }

        // Apply the scaling factor to both commands to preserve ratio
        if (scale > 1.0) {
            left_cmd *= scale;
            right_cmd *= scale;
        }

        // Re-clamp the values to the absolute max/min after scaling
        // This ensures we don't send a command like "L300,R600"
        left_cmd = std::max(-MAX_CMD, std::min(left_cmd, MAX_CMD));
        right_cmd = std::max(-MAX_CMD, std::min(right_cmd, MAX_CMD));

        // Format the command string
        std::string command = "L" + std::to_string(static_cast<int>(left_cmd)) + ",R" + std::to_string(static_cast<int>(right_cmd)) + "\n";
        
        //RCLCPP_INFO(this->get_logger(), "COMMAND: %s", command.c_str());
        write(serial_fd_, command.c_str(), command.length());
    }

    /**
     * @brief Timer callback to read all available data from the serial port.
     */
    void read_serial_callback()
    {
        if (!serial_ok_) return;
        
        char buffer[256];
        int n = read(serial_fd_, buffer, sizeof(buffer));

        if (n < 0) {
            // If EAGAIN, it's not a real error, just no data available
            if (errno == EAGAIN) {
                return;
            }
            RCLCPP_WARN(this->get_logger(), "Error reading from serial: %s", strerror(errno));
            return;
        }

        if (n > 0) {
            read_buffer_.append(buffer, n);
 
            // Process all complete lines in the buffer
            size_t newline_pos;
            while ((newline_pos = read_buffer_.find('\n')) != std::string::npos) {
                std::string line = read_buffer_.substr(0, newline_pos);
                read_buffer_.erase(0, newline_pos + 1);

                // Clean up carriage return if present
                if (!line.empty() && line.back() == '\r') {
                    line.pop_back();
                }
                if (line.empty()) continue;

                // --- MODIFIED: Route line based on prefix ---
                if (line.rfind('V', 0) == 0) {
                    parse_and_publish_odom(line);
                } else if (line.rfind('I', 0) == 0) {
                    parse_and_publish_imu(line);
                } else if (line.find("Arduino") != std::string::npos || line.find("BNO08x") != std::string::npos || line.find("reports") != std::string::npos) {
                    // Log setup messages from Arduino
                    RCLCPP_INFO(this->get_logger(), "Arduino setup: %s", line.c_str());
                } else {
                    RCLCPP_WARN(this->get_logger(), "Got unknown line: '%s'", line.c_str());
                }
            }
        }
    }
    
    /**
     * @brief Parses an Odometry line (e.g., "V0.123,-0.432") and publishes.
     */
    void parse_and_publish_odom(std::string line)
    {
        try {
            size_t comma_pos = line.find(',');
            if (comma_pos == std::string::npos) return; // Malformed
            
            // Start substring from 1 to skip the 'V'
            std::string left_str = line.substr(1, comma_pos - 1);
            std::string right_str = line.substr(comma_pos + 1);

            double left_vel = std::stod(left_str);
            double right_vel = std::stod(right_str);

            rclcpp::Time current_time = this->get_clock()->now();

            // Calculate robot's linear and angular velocity (inverse kinematics)
            double linear_vel = (left_vel + right_vel) / 2.0;
            double angular_vel = (right_vel - left_vel) / wheel_base_;

            // Create and publish odometry message
            nav_msgs::msg::Odometry odom_msg;
            odom_msg.header.stamp = current_time;
            odom_msg.header.frame_id = odom_frame_id_;
            odom_msg.child_frame_id = base_frame_id_;

            // We only publish twist (velocity), not pose
            odom_msg.pose.pose.position.x = 0.0; // We don't integrate pose
            odom_msg.pose.pose.position.y = 0.0;
            odom_msg.pose.pose.orientation.w = 1.0;

            odom_msg.twist.twist.linear.x = linear_vel;
            odom_msg.twist.twist.angular.z = angular_vel;
            
                        
            odom_publisher_->publish(odom_msg);
        } catch (const std::invalid_argument& e) {
            RCLCPP_WARN(this->get_logger(), "Could not parse velocity: '%s'", line.c_str());
        } catch (const std::out_of_range& e) {
            RCLCPP_WARN(this->get_logger(), "Could not parse velocity (out of range): '%s'", line.c_str());
        }
    }

    /**
     * @brief --- MODIFIED: Parses an IMU line and publishes sensor_msgs::msg::Imu.
     */
    void parse_and_publish_imu(std::string line)
    {
        // Line format: "I,acc_x,acc_y,acc_z,gyro_x,gyro_y,gyro_z,mag_x,mag_y,mag_z,lin_acc_x,lin_acc_y,lin_acc_z,quat_x,quat_y,quat_z,quat_w" (16 values)
        
        // Start after the "I,"
        std::stringstream ss(line.substr(2));
        std::string part;
        std::vector<double> values;

        while (std::getline(ss, part, ',')) {
            try {
                values.push_back(std::stod(part));
            } catch (const std::exception& e) {
                RCLCPP_WARN(this->get_logger(), "Could not parse IMU value '%s' from line '%s'", part.c_str(), line.c_str());
                return; // Stop parsing this line
            }
        }

        if (values.size() != 16) {
            RCLCPP_WARN(this->get_logger(), "Malformed IMU line (expected 16 values, got %zu): %s", values.size(), line.c_str());
            return;
        }

        // Create and populate the standard IMU message
        auto msg = sensor_msgs::msg::Imu();
        msg.header.stamp = this->get_clock()->now();
        msg.header.frame_id = imu_frame_id_;
        
        // --- Map values to standard IMU fields ---

        // Orientation (Quaternions: values 12-15)
        msg.orientation.x = values[12];
        msg.orientation.y = values[13];
        msg.orientation.z = values[14];
        msg.orientation.w = values[15];
        
        // Angular Velocity (Gyro: values 3-5)
        msg.angular_velocity.x = values[3];
        msg.angular_velocity.y = values[4];
        msg.angular_velocity.z = values[5];
        
        // Linear Acceleration (No gravity: values 9-11)
        // Note: values 0-2 (total accel) and 6-8 (mag) are not used in this message
        msg.linear_acceleration.x = values[9];
        msg.linear_acceleration.y = values[10];
        msg.linear_acceleration.z = values[11];
        
                
        // Publish the message
        imu_publisher_->publish(msg);
    }

    /**
     * @brief Initializes the serial port connection.
     * @return true if successful, false otherwise.
     */
    bool init_serial()
    {
        // Open serial port in non-blocking read/write mode
        serial_fd_ = open(serial_port_.c_str(), O_RDWR | O_NOCTTY | O_NDELAY);

        if (serial_fd_ == -1) {
            RCLCPP_ERROR(this->get_logger(), "Failed to open serial port: %s. Error: %s", serial_port_.c_str(), strerror(errno));
            return false;
        }

        struct termios options;
        tcgetattr(serial_fd_, &options);

        // Set baud rate
        speed_t baud;
        switch (baud_rate_) {
            case 9600:   baud = B9600;   break;
            case 19200:  baud = B19200;  break;
            case 38400:  baud = B38400;  break;
            case 57600:  baud = B57600;  break;
            case 115200: baud = B115200; break;
            default:     baud = B115200; break;
        }
        cfsetispeed(&options, baud);
        cfsetospeed(&options, baud);

        // Set minimal options for raw serial mode
        options.c_cflag |= (CLOCAL | CREAD); // Enable receiver, ignore modem control
        options.c_cflag &= ~PARENB;          // No parity
        options.c_cflag &= ~CSTOPB;          // 1 stop bit
        options.c_cflag &= ~CSIZE;
        options.c_cflag |= CS8;              // 8 data bits
        options.c_iflag &= ~(IXON | IXOFF | IXANY); // Disable software flow control
        options.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG); // Non-canonical mode
        options.c_oflag &= ~OPOST;           // Raw output

        // Set VMIN and VTIME for non-blocking reads
        options.c_cc[VTIME] = 0;
        options.c_cc[VMIN] = 0;

        // Apply the attributes
        tcflush(serial_fd_, TCIFLUSH); // Flush input buffer
        if (tcsetattr(serial_fd_, TCSANOW, &options) != 0) {
            RCLCPP_ERROR(this->get_logger(), "Failed to set serial attributes: %s", strerror(errno));
            return false;
        }
        
        RCLCPP_INFO(this->get_logger(), "Serial port opened successfully: %s at %d baud", serial_port_.c_str(), baud_rate_);
        std::this_thread::sleep_for(2s); // Give Arduino time to reset
        return true;
    }

    // ROS Members
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr subscription_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_publisher_;
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_publisher_; // <-- MODIFIED
    rclcpp::TimerBase::SharedPtr read_timer_;
    
    // State Variables
    std::string read_buffer_; // Buffer for incoming serial data
    std::string serial_port_;
    int baud_rate_;
    double wheel_base_;
    double max_speed_;
    int serial_fd_ = -1; // File descriptor for the serial port
    bool serial_ok_ = false; // Flag for successful serial init
    
    // Frame IDs
    std::string odom_frame_id_;
    std::string base_frame_id_;
    std::string imu_frame_id_; // <-- ADDED
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<TwistToArduino>();
    
    // --- MODIFIED: Check if serial init failed ---
    if (!node->is_serial_ok()) {
        RCLCPP_FATAL(node->get_logger(), "Serial initialization failed. Shutting down.");
        rclcpp::shutdown();
        return 1;
    }

    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}

