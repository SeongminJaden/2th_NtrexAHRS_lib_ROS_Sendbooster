#pragma once

#include <atomic>
#include <memory>
#include <mutex>
#include <string>
#include <thread>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/magnetic_field.hpp>
#include <std_msgs/msg/float64.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>

#include "mw_serial.hpp"

constexpr uint8_t ACC = 0x33;
constexpr uint8_t GYO = 0x34;
constexpr uint8_t DEG = 0x35;
constexpr uint8_t MAG = 0x36;

constexpr double convertor_g2a = 9.80665;        // linear_acceleration (g to m/s^2)
constexpr double convertor_d2r = M_PI / 180.0;   // angular_velocity (degree to radian)
constexpr double convertor_ut2t = 1000000.0;      // magnetic_field (uT to Tesla)
constexpr double convertor_c = 1.0;               // temperature (celsius)

namespace ntrex
{
    class MwAhrsRosDriver : public rclcpp::Node
    {
    public:
        MwAhrsRosDriver(const std::string &port, int baud_rate);
        ~MwAhrsRosDriver();

        void StartReading();
        void StopReading();
        void StartPubing();
        void StopPubing();

    private:
        void MW_AHRS_Covariance();
        void MwAhrsRead();
        tf2::Quaternion Euler2Quaternion(float roll, float pitch, float yaw);
        void publish_topic();

        // ROS parameters
        double linear_acceleration_stddev_{0.0};
        double angular_velocity_stddev_{0.0};
        double magnetic_field_stddev_{0.0};
        double orientation_stddev_{0.0};

        double linear_acceleration_cov{0.0};
        double angular_velocity_cov{0.0};
        double magnetic_field_cov{0.0};
        double orientation_cov{0.0};

        double roll{0.0};
        double pitch{0.0};
        double yaw{0.0};

        bool publish_tf_{false};
        std::string parent_frame_id_{"base_link"};
        std::string frame_id_{"imu_link"};

        // Thread control
        std::atomic<bool> running_{false};
        std::mutex data_mutex_;
        std::thread reading_thread_;
        std::thread publisher_thread_;

        // Message objects
        sensor_msgs::msg::Imu imu_data_raw_msg_;
        sensor_msgs::msg::Imu imu_data_msg_;
        sensor_msgs::msg::MagneticField imu_magnetic_msg_;
        std_msgs::msg::Float64 imu_yaw_msg_;
        tf2::Quaternion tf_orientation_;

        // Publishers
        rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_data_raw_pub_;
        rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_data_pub_;
        rclcpp::Publisher<sensor_msgs::msg::MagneticField>::SharedPtr imu_mag_pub_;
        rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr imu_yaw_pub_;
        std::unique_ptr<tf2_ros::TransformBroadcaster> broadcaster_;
    };
}
