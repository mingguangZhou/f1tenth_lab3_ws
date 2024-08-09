#include "rclcpp/rclcpp.hpp"
#include <string>
#include "sensor_msgs/msg/laser_scan.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "ackermann_msgs/msg/ackermann_drive_stamped.hpp"

#include <cmath>
#include <chrono>

class WallFollow : public rclcpp::Node {

public:
    WallFollow() : Node("wall_follow_node"), time_prev_(this->now())
    {
        // create ROS subscribers and publishers
        scan_subscriber_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
                "scan", 10, std::bind(&WallFollow::scan_callback, this, std::placeholders::_1));
        odom_subscriber_ = this->create_subscription<nav_msgs::msg::Odometry>(
                "ego_racecar/odom", 10, std::bind(&WallFollow::drive_callback, this, std::placeholders::_1));
        drive_publisher_ = this->create_publisher<ackermann_msgs::msg::AckermannDriveStamped>("drive", 10);
        RCLCPP_INFO(this->get_logger(), "Wall Follow Node has been started");

        // Declare parameters with default values
        this->declare_parameter<double>("kp", 0.0);
        this->declare_parameter<double>("ki", 0.0);
        this->declare_parameter<double>("kd", 0.0);

        // Get parameters from the parameter server (YAML file)
        kp_ = this->get_parameter("kp").as_double();
        ki_ = this->get_parameter("ki").as_double();
        kd_ = this->get_parameter("kd").as_double();

        RCLCPP_INFO(this->get_logger(), "PID parameters loaded: kp=%f, ki=%f, kd=%f", kp_, ki_, kd_);
      
    }

private:
    // PID CONTROL PARAMS
    double kp_;
    double ki_;
    double kd_;
    double servo_offset = 0.0;
    double prev_error = 0.0;
    // double error = 0.0;
    double integral = 0.0;
    double dist_desired = 0.5;

    rclcpp::Time time_prev_;

    double speed_curr = 0.0;

    // Topics
    // std::string lidarscan_topic = "/scan";
    // std::string drive_topic = "/drive";
    // std::string odom_topic = '/ego_racecar/odom';
    /// create ROS subscribers and publishers
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_subscriber_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_subscriber_;
    rclcpp::Publisher<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr drive_publisher_;
    
    

    double get_range(const std::vector<float>& range_data, float angle_deg, float angle_min_rad, float angle_increment_rad)
    {
        /*
        Simple helper to return the corresponding range measurement at a given angle. Make sure you take care of NaNs and infs.

        Args:
            range_data: single range array from the LiDAR
            angle: between angle_min and angle_max of the LiDAR

        Returns:
            range: range measurement in meters at the given angle
        */

        // convert angle to the left of the car
        float angle_rad = (90.0 - angle_deg) * (M_PI / 180.0);
        // Find the corresponding index in the range_data array
        int index = static_cast<int>((angle_rad - angle_min_rad) / angle_increment_rad);

        // Get the range value at the calculated index
        float range = range_data[index];

        // Check if the range value is valid (not NaN or infinity)
        if (std::isnan(range) || std::isinf(range)) {
            // Return a default invalid range if the value is NaN or infinity
            return std::numeric_limits<double>::quiet_NaN();
        }

        return static_cast<double>(range);
    }

    void drive_callback(const nav_msgs::msg::Odometry::ConstSharedPtr msg)
    {
        /// update current speed
        speed_curr = msg->twist.twist.linear.x;
    }

    double get_error(const std::vector<float>& range_data, double dist, double speed, float angle_min_rad, float angle_increment_rad)
    {
        /*
        Calculates the error to the wall. Follow the wall to the left (going counter clockwise in the Levine loop). You potentially will need to use get_range()

        Args:
            range_data: single range array from the LiDAR
            dist: desired distance to the left wall

        Returns:
            error: calculated error
        */

        // implement
        // Parameters
        double ang_1 = 30.0;
        double ang_2 = 50.0;
        const double max_angle_2 = 70.0;
        // Assumed delay time
        double t_delay = 0.05;
        

        double a = get_range(range_data, ang_1, angle_min_rad, angle_increment_rad);
        double b = get_range(range_data, ang_2, angle_min_rad, angle_increment_rad);

        while ((std::isnan(a) || std::isnan(b)) && ang_2 <= max_angle_2) {
            ang_1 += 1.0;
            ang_2 += 1.0;
            a = get_range(range_data, ang_1, angle_min_rad, angle_increment_rad);
            b = get_range(range_data, ang_2, angle_min_rad, angle_increment_rad);
        }

        // If ang_2 exceeds the maximum allowed angle, return NaN as error
        if (ang_2 > max_angle_2) {
            return std::numeric_limits<double>::quiet_NaN();
        }


        double theta = (ang_1 - ang_2) * (M_PI / 180.0);

        double alpha = std::atan((a * std::cos(theta) - b)/(a * std::sin(theta)));
        double dist_curr = b * std::cos(alpha);

        // debug 
        double alpha_deg = alpha * (180 / M_PI);
        RCLCPP_INFO(this->get_logger(), "current a dist: %f [m]", a);
        RCLCPP_INFO(this->get_logger(), "current b dist: %f [m]", b);
        // double c = get_range(range_data, 0.0, angle_min_rad, angle_increment_rad);
        // RCLCPP_INFO(this->get_logger(), "0 deg dist: %f [m]", c);
        RCLCPP_INFO(this->get_logger(), "current alpha: %f [deg]", alpha_deg);
        RCLCPP_INFO(this->get_logger(), "current dist to left wall: %f [m]", dist_curr);

        double dist_pred = dist_curr + speed * t_delay * std::sin(alpha);

        return (dist - dist_pred);
    }

    void pid_control(double error, double dt)
    {
        /*
        Based on the calculated error, publish vehicle control

        Args:
            error: calculated error
            velocity: desired velocity

        Returns:
            None
        */
        
        // Use kp, ki & kd to implement a PID controller
        double angle = kp_ * error + ki_ * integral + kd_ * (error - prev_error) / dt;
        double angle_deg = angle * (180.0 / M_PI);
        // generate desired velocity from steering angle
        double velocity;
        if (fabs(angle_deg)>=0.0 && fabs(angle_deg)<=10.0) {
            velocity = 1.5;
        } else if (fabs(angle_deg)>10.0 && fabs(angle_deg)<=20.0) {
            velocity = 1.0;
        } else {
            velocity = 0.5;
        }
        // velocity = 0.1;

        auto drive_msg = ackermann_msgs::msg::AckermannDriveStamped();
        // fill in drive message and publish
        drive_msg.drive.speed = static_cast<float>(velocity);
        drive_msg.drive.steering_angle = static_cast<float>(-angle);
        // drive_msg.drive.steering_angle = 0.0;
        drive_publisher_ ->publish(drive_msg);
        RCLCPP_INFO(this->get_logger(), "steering command: %f [deg]", angle_deg);
        // RCLCPP_INFO(this->get_logger(), "speed command: %f m/s", velocity);

        integral += error * dt;
        prev_error = error;
    }

    void scan_callback(const sensor_msgs::msg::LaserScan::ConstSharedPtr scan_msg) 
    {
        /*
        Callback function for LaserScan messages. Calculate the error and publish the drive message in this function.

        Args:
            msg: Incoming LaserScan message

        Returns:
            None
        */
        float angle_min = scan_msg->angle_min;
        float angle_increment = scan_msg->angle_increment;

        // handle time
        rclcpp::Time time_now = scan_msg->header.stamp;
        rclcpp::Duration duration = time_now - time_prev_;
        double dt = duration.seconds();
        RCLCPP_INFO(this->get_logger(), "Time difference: %f seconds", dt);

        // error calculated by get_error()
        double error = get_error(scan_msg->ranges, dist_desired, speed_curr, angle_min, angle_increment);
        RCLCPP_INFO(this->get_logger(), "Current Error: %f m", error);

        // actuate the car with PID
        pid_control(error, dt);

        time_prev_ = time_now;
    }

};
int main(int argc, char ** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<WallFollow>());
    rclcpp::shutdown();
    return 0;
}