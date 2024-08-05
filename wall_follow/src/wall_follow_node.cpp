#include "rclcpp/rclcpp.hpp"
#include <string>
#include "sensor_msgs/msg/laser_scan.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "ackermann_msgs/msg/ackermann_drive_stamped.hpp"

#include <cmath>

class WallFollow : public rclcpp::Node {

public:
    WallFollow() : Node("wall_follow_node")
    {
        // create ROS subscribers and publishers
        scan_subscriber_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
                lidarscan_topic, 10, std::bind(&WallFollow::scan_callback, this, std::placeholders::_1));
        drive_publisher_ = this->create_publisher<ackermann_msgs::msg::AckermannDriveStamped>(drive_topic, 10);
        RCLCPP_INFO(this->get_logger(), "Wall Follow Node has been started");
      
    }

private:
    // PID CONTROL PARAMS
    double kp = 1.0;
    double kd = 0.0;
    double ki = 0.0;
    double servo_offset = 0.0;
    double prev_error = 0.0;
    double error = 0.0;
    double integral = 0.0;

    // Topics
    std::string lidarscan_topic = "/scan";
    std::string drive_topic = "/drive";
    /// create ROS subscribers and publishers
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_subscriber_;
    rclcpp::Publisher<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr drive_publisher_;
    
    

    double get_range(float* range_data, float angle_deg, float angle_min_rad, float angle_increment_rad)
    {
        /*
        Simple helper to return the corresponding range measurement at a given angle. Make sure you take care of NaNs and infs.

        Args:
            range_data: single range array from the LiDAR
            angle: between angle_min and angle_max of the LiDAR

        Returns:
            range: range measurement in meters at the given angle
        */

        // implement
        float angle_rad = angle_deg * (M_PI / 180.0);
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

    double get_error(float* range_data, double dist, double velocity, float angle_min_rad, float angle_increment_rad)
    {
        /*
        Calculates the error to the wall. Follow the wall to the left (going counter clockwise in the Levine loop). You potentially will need to use get_range()

        Args:
            range_data: single range array from the LiDAR
            dist: desired distance to the wall

        Returns:
            error: calculated error
        */

        // implement
        // Parameters
        double ang_1 = 30.0;
        double ang_2 = 50.0;
        const double max_angle_2 = 70.0;
        // Assumed delay time
        double t_delay = 0.1;
        

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


        double theta = (ang_2 - ang_1) * (M_PI / 180.0);

        double alpha = std::atan((a * std::cos(theta) - b)/(a * std::sin(theta)));
        double dist_curr = b * std::cos(alpha);

        double dist_pred = dist_curr + velocity * t_delay * std::sin(alpha)

        return (dist - dist_pred);
    }

    void pid_control(double error, double velocity, double dt)
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
        double angle = kp * error + ki * integral + kd * (error - prev_error) / dt;
        integral += error * dt;
        prev_error = error;

        auto drive_msg = ackermann_msgs::msg::AckermannDriveStamped();
        // TODO: fill in drive message and publish
        drive_msg.drive.speed = static_cast<float>(velocity);
        drive_msg.drive.steering_angle = static_cast<float>(angle);
        drive_publisher_ ->publish(drive_msg);
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
        double error = 0.0; // TODO: replace with error calculated by get_error()
        double velocity = 0.0; // TODO: calculate desired car velocity based on error
        // TODO: actuate the car with PID

        float angle_min = scan_msg->angle_min;
        float angle_increment = scan_msg->angle_increment;

    }

};
int main(int argc, char ** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<WallFollow>());
    rclcpp::shutdown();
    return 0;
}