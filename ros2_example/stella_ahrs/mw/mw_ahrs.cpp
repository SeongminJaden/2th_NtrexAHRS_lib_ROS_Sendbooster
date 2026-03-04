#include "mw_ahrs.hpp"

#include <chrono>

using namespace std::chrono_literals;

namespace ntrex
{
  void MwAhrsRosDriver::StartReading()
  {
    running_ = true;
    std::this_thread::sleep_for(1s);
    reading_thread_ = std::thread(&MwAhrsRosDriver::MwAhrsRead, this);
  }

  void MwAhrsRosDriver::StopReading()
  {
    if (running_)
    {
      running_ = false;
      std::this_thread::sleep_for(1s);
      if (reading_thread_.joinable())
      {
        reading_thread_.join();
      }
    }
  }

  void MwAhrsRosDriver::StartPubing()
  {
    if (running_)
      publisher_thread_ = std::thread(&MwAhrsRosDriver::publish_topic, this);
  }

  void MwAhrsRosDriver::StopPubing()
  {
    if (publisher_thread_.joinable())
      publisher_thread_.join();
  }

  void MwAhrsRosDriver::MW_AHRS_Covariance(void)
  {
    imu_data_raw_msg_ = sensor_msgs::msg::Imu();
    imu_data_msg_ = sensor_msgs::msg::Imu();
    imu_magnetic_msg_ = sensor_msgs::msg::MagneticField();
    imu_yaw_msg_ = std_msgs::msg::Float64();

    linear_acceleration_cov = linear_acceleration_stddev_ * linear_acceleration_stddev_;
    angular_velocity_cov = angular_velocity_stddev_ * angular_velocity_stddev_;
    magnetic_field_cov = magnetic_field_stddev_ * magnetic_field_stddev_;
    orientation_cov = orientation_stddev_ * orientation_stddev_;

    imu_data_raw_msg_.linear_acceleration_covariance[0] =
        imu_data_raw_msg_.linear_acceleration_covariance[4] =
            imu_data_raw_msg_.linear_acceleration_covariance[8] =
                imu_data_msg_.linear_acceleration_covariance[0] =
                    imu_data_msg_.linear_acceleration_covariance[4] =
                        imu_data_msg_.linear_acceleration_covariance[8] =
                            linear_acceleration_cov;

    imu_data_raw_msg_.angular_velocity_covariance[0] =
        imu_data_raw_msg_.angular_velocity_covariance[4] =
            imu_data_raw_msg_.angular_velocity_covariance[8] =
                imu_data_msg_.angular_velocity_covariance[0] =
                    imu_data_msg_.angular_velocity_covariance[4] =
                        imu_data_msg_.angular_velocity_covariance[8] =
                            angular_velocity_cov;

    imu_data_msg_.orientation_covariance[0] =
        imu_data_msg_.orientation_covariance[4] =
            imu_data_msg_.orientation_covariance[8] =
                orientation_cov;

    imu_magnetic_msg_.magnetic_field_covariance[0] =
        imu_magnetic_msg_.magnetic_field_covariance[4] =
            imu_magnetic_msg_.magnetic_field_covariance[8] =
                magnetic_field_cov;
  }

  void MwAhrsRosDriver::MwAhrsRead()
  {
    float acc_value[3] = {0.0f};
    float gyr_value[3] = {0.0f};
    float deg_value[3] = {0.0f};
    float mag_value[3] = {0.0f};

    while (running_)
    {
      unsigned char data[8];

      if (AHRS_Read(data))
      {
        std::lock_guard<std::mutex> lock(data_mutex_);

        switch (static_cast<int>(static_cast<unsigned char>(data[1])))
        {
        case ACC:
          acc_value[0] = static_cast<int16_t>(static_cast<int>(static_cast<unsigned char>(data[2])) | static_cast<int>(static_cast<unsigned char>(data[3])) << 8) / 1000.0f;
          acc_value[1] = static_cast<int16_t>(static_cast<int>(static_cast<unsigned char>(data[4])) | static_cast<int>(static_cast<unsigned char>(data[5])) << 8) / 1000.0f;
          acc_value[2] = static_cast<int16_t>(static_cast<int>(static_cast<unsigned char>(data[6])) | static_cast<int>(static_cast<unsigned char>(data[7])) << 8) / 1000.0f;

          imu_data_raw_msg_.linear_acceleration.x = imu_data_msg_.linear_acceleration.x =
              acc_value[0] * convertor_g2a;
          imu_data_raw_msg_.linear_acceleration.y = imu_data_msg_.linear_acceleration.y =
              acc_value[1] * convertor_g2a;
          imu_data_raw_msg_.linear_acceleration.z = imu_data_msg_.linear_acceleration.z =
              acc_value[2] * convertor_g2a;

          break;

        case GYO:
          gyr_value[0] = static_cast<int16_t>(static_cast<int>(static_cast<unsigned char>(data[2])) | static_cast<int>(static_cast<unsigned char>(data[3])) << 8) / 10.0f;
          gyr_value[1] = static_cast<int16_t>(static_cast<int>(static_cast<unsigned char>(data[4])) | static_cast<int>(static_cast<unsigned char>(data[5])) << 8) / 10.0f;
          gyr_value[2] = static_cast<int16_t>(static_cast<int>(static_cast<unsigned char>(data[6])) | static_cast<int>(static_cast<unsigned char>(data[7])) << 8) / 10.0f;

          imu_data_raw_msg_.angular_velocity.x = imu_data_msg_.angular_velocity.x =
              gyr_value[0] * convertor_d2r;
          imu_data_raw_msg_.angular_velocity.y = imu_data_msg_.angular_velocity.y =
              gyr_value[1] * convertor_d2r;
          imu_data_raw_msg_.angular_velocity.z = imu_data_msg_.angular_velocity.z =
              gyr_value[2] * convertor_d2r;

          break;

        case DEG:
          deg_value[0] = static_cast<int16_t>(static_cast<int>(static_cast<unsigned char>(data[2])) | static_cast<int>(static_cast<unsigned char>(data[3])) << 8) / 100.0f;
          deg_value[1] = static_cast<int16_t>(static_cast<int>(static_cast<unsigned char>(data[4])) | static_cast<int>(static_cast<unsigned char>(data[5])) << 8) / 100.0f;
          deg_value[2] = static_cast<int16_t>(static_cast<int>(static_cast<unsigned char>(data[6])) | static_cast<int>(static_cast<unsigned char>(data[7])) << 8) / 100.0f;

          roll = deg_value[0] * convertor_d2r;
          pitch = deg_value[1] * convertor_d2r;
          yaw = deg_value[2] * convertor_d2r;

          tf_orientation_ = Euler2Quaternion(roll, pitch, yaw);

          imu_yaw_msg_.data = deg_value[2];

          imu_data_msg_.orientation.x = tf_orientation_.x();
          imu_data_msg_.orientation.y = tf_orientation_.y();
          imu_data_msg_.orientation.z = tf_orientation_.z();
          imu_data_msg_.orientation.w = tf_orientation_.w();

          break;

        case MAG:
          mag_value[0] = static_cast<int16_t>(static_cast<int>(static_cast<unsigned char>(data[2])) | static_cast<int>(static_cast<unsigned char>(data[3])) << 8) / 10.0f;
          mag_value[1] = static_cast<int16_t>(static_cast<int>(static_cast<unsigned char>(data[4])) | static_cast<int>(static_cast<unsigned char>(data[5])) << 8) / 10.0f;
          mag_value[2] = static_cast<int16_t>(static_cast<int>(static_cast<unsigned char>(data[6])) | static_cast<int>(static_cast<unsigned char>(data[7])) << 8) / 10.0f;

          imu_magnetic_msg_.magnetic_field.x = mag_value[0] / convertor_ut2t;
          imu_magnetic_msg_.magnetic_field.y = mag_value[1] / convertor_ut2t;
          imu_magnetic_msg_.magnetic_field.z = mag_value[2] / convertor_ut2t;

          break;
        }
      }
    }
  }

  void MwAhrsRosDriver::publish_topic()
  {
    rclcpp::Rate rate(1000);

    while (rclcpp::ok() && running_)
    {
      sensor_msgs::msg::Imu imu_raw_copy;
      sensor_msgs::msg::Imu imu_copy;
      sensor_msgs::msg::MagneticField mag_copy;
      std_msgs::msg::Float64 yaw_copy;
      geometry_msgs::msg::Quaternion orientation_copy;

      {
        std::lock_guard<std::mutex> lock(data_mutex_);
        imu_raw_copy = imu_data_raw_msg_;
        imu_copy = imu_data_msg_;
        mag_copy = imu_magnetic_msg_;
        yaw_copy = imu_yaw_msg_;
        orientation_copy = imu_data_msg_.orientation;
      }

      rclcpp::Time now = this->get_clock()->now();

      imu_raw_copy.header.stamp = now;
      imu_raw_copy.header.frame_id = frame_id_;
      imu_copy.header.stamp = now;
      imu_copy.header.frame_id = frame_id_;
      mag_copy.header.stamp = now;
      mag_copy.header.frame_id = frame_id_;

      imu_data_raw_pub_->publish(imu_raw_copy);
      imu_data_pub_->publish(imu_copy);
      imu_mag_pub_->publish(mag_copy);
      imu_yaw_pub_->publish(yaw_copy);

      if (publish_tf_)
      {
        geometry_msgs::msg::TransformStamped tf;
        tf.header.stamp = now;
        tf.header.frame_id = parent_frame_id_;
        tf.child_frame_id = frame_id_;
        tf.transform.translation.x = 0.0;
        tf.transform.translation.y = 0.0;
        tf.transform.translation.z = 0.0;
        tf.transform.rotation = orientation_copy;

        broadcaster_->sendTransform(tf);
      }
      rate.sleep();
    }
  }

  tf2::Quaternion MwAhrsRosDriver::Euler2Quaternion(float roll, float pitch, float yaw)
  {
    float qx = (sin(roll / 2) * cos(pitch / 2) * cos(yaw / 2)) -
               (cos(roll / 2) * sin(pitch / 2) * sin(yaw / 2));
    float qy = (cos(roll / 2) * sin(pitch / 2) * cos(yaw / 2)) +
               (sin(roll / 2) * cos(pitch / 2) * sin(yaw / 2));
    float qz = (cos(roll / 2) * cos(pitch / 2) * sin(yaw / 2)) -
               (sin(roll / 2) * sin(pitch / 2) * cos(yaw / 2));
    float qw = (cos(roll / 2) * cos(pitch / 2) * cos(yaw / 2)) +
               (sin(roll / 2) * sin(pitch / 2) * sin(yaw / 2));

    tf2::Quaternion q(qx, qy, qz, qw);
    return q;
  }

  MwAhrsRosDriver::MwAhrsRosDriver(const std::string &port, int baud_rate)
    : Node("MW_AHRS_ROS2")
  {
    // Declare all parameters with defaults
    this->declare_parameter("port", port);
    this->declare_parameter("baud_rate", baud_rate);
    this->declare_parameter("publish_tf", false);
    this->declare_parameter("frame_id", std::string("imu_link"));
    this->declare_parameter("parent_frame_id", std::string("base_link"));
    this->declare_parameter("linear_acceleration_stddev", 0.02);
    this->declare_parameter("angular_velocity_stddev", 0.01);
    this->declare_parameter("magnetic_field_stddev", 0.00000327486);
    this->declare_parameter("orientation_stddev", 0.00125);

    // Read parameters
    std::string actual_port = this->get_parameter("port").as_string();
    int actual_baud = this->get_parameter("baud_rate").as_int();
    publish_tf_ = this->get_parameter("publish_tf").as_bool();
    frame_id_ = this->get_parameter("frame_id").as_string();
    parent_frame_id_ = this->get_parameter("parent_frame_id").as_string();

    this->get_parameter("linear_acceleration_stddev", linear_acceleration_stddev_);
    this->get_parameter("angular_velocity_stddev", angular_velocity_stddev_);
    this->get_parameter("magnetic_field_stddev", magnetic_field_stddev_);
    this->get_parameter("orientation_stddev", orientation_stddev_);

    int res = MW_AHRS_Serial_Connect(const_cast<char *>(actual_port.c_str()),
                                     static_cast<uint32_t>(actual_baud), 0);

    if (res)
    {
      MW_AHRS_Covariance();

      if (publish_tf_)
      {
        broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
      }

      auto qos = rclcpp::QoS(rclcpp::KeepLast(10)).reliable().durability_volatile();

      imu_data_raw_pub_ = this->create_publisher<sensor_msgs::msg::Imu>("imu/data_raw", qos);
      imu_data_pub_ = this->create_publisher<sensor_msgs::msg::Imu>("imu/data", qos);
      imu_mag_pub_ = this->create_publisher<sensor_msgs::msg::MagneticField>("imu/mag", qos);
      imu_yaw_pub_ = this->create_publisher<std_msgs::msg::Float64>("imu/yaw", qos);

      calibration_srv_ = this->create_service<std_srvs::srv::Trigger>(
          "imu/calibration",
          [this](const std_srvs::srv::Trigger::Request::SharedPtr,
                 std_srvs::srv::Trigger::Response::SharedPtr res) {
            RCLCPP_INFO(this->get_logger(), "Magnetometer calibration started...");
            int ret = AHRS_Calibration();
            res->success = (ret != 0);
            res->message = res->success ? "Calibration OK" : "Calibration failed";
            RCLCPP_INFO(this->get_logger(), "AHRS_Calibration result: %d", ret);
          });

      euler_reset_srv_ = this->create_service<std_srvs::srv::Trigger>(
          "imu/euler_reset",
          [this](const std_srvs::srv::Trigger::Request::SharedPtr,
                 std_srvs::srv::Trigger::Response::SharedPtr res) {
            int ret = AHRS_Euler_RESET();
            res->success = (ret != 0);
            res->message = res->success ? "Euler reset OK" : "Euler reset failed";
            RCLCPP_INFO(this->get_logger(), "AHRS_Euler_RESET result: %d", ret);
          });

      StartReading();
      StartPubing();
      RCLCPP_INFO(this->get_logger(), "MW-AHRS ROS Init Success");
    }
    else
    {
      RCLCPP_ERROR(this->get_logger(), "MW-AHRS ROS Init Fail");
    }
  }

  MwAhrsRosDriver::~MwAhrsRosDriver()
  {
    StopReading();
    StopPubing();
    MW_Serial_DisConnect();
  }
}
