
#include "sabertooth_motor_driver/sabertooth_driver_node.hpp"

#include "rclcpp/logging.hpp"

namespace sabertooth_motor_driver {

SabertoothDriverNode::SabertoothDriverNode(const std::string & node_name)
: Node(node_name) {

  this->declare_parameter<std::string>("serial_port", "/dev/ttyACM0");
  this->declare_parameter<double>("feedback_rate_hz", 5.0);

  this->get_parameter("serial_port", serial_port_);
  this->get_parameter("feedback_rate_hz", feedback_rate_hz_);

  // Initialize the driver with the serial port and logger
  driver_ = std::make_shared<Driver>( serial_port_, this->get_logger() );

  // Create the subscription to the cmd topic
  // Using SensorDataQoS for better performance with sensor data
  cmd_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
    "platform/motors/cmd", rclcpp::SensorDataQoS(),
    std::bind(&SabertoothDriverNode::cmdCallback, this, std::placeholders::_1));

  // Create the publisher for the feedback topic
  // Using SensorDataQoS for better performance with sensor data
  feedback_pub_ = this->create_publisher<sensor_msgs::msg::JointState>(
    "platform/motors/feedback", rclcpp::SensorDataQoS());

  // set the interval for the feedback timer
  // Convert feedback_rate_hz to a duration for the timer    
  auto interval = std::chrono::duration<double>(1.0 / feedback_rate_hz_);
  
  // Create the feedback timer
  feedback_timer_ = this->create_wall_timer(std::chrono::duration_cast<std::chrono::milliseconds>(interval),
    std::bind(
      &SabertoothDriverNode::feedbackTimerCallback, this));
    
  };

  /**
   * @brief Callback for the cmd topic.
   * 
   * This function processes the incoming JointState message, 
   * extracts the velocities for the left and right wheels.
   * 
   * @param msg The JointState message containing wheel velocities.
   */
  void SabertoothDriverNode::cmdCallback( const sensor_msgs::msg::JointState::SharedPtr msg ) {
  double left_vel = 0.0, right_vel = 0.0;
  bool left_found = false, right_found = false;
  RCLCPP_DEBUG(this->get_logger(), "Received cmd message with %zu joints", msg->name.size());

  // Iterate through the joint names to find left and right wheel velocities
  for (size_t i = 0; i < msg->name.size(); ++i) {
    if (msg->name[i] == "front_left_joint") {
      left_vel = msg->velocity[i];
      left_found = true;
    } else if (msg->name[i] == "front_right_joint") {
      right_vel = msg->velocity[i];
      right_found = true;
    }
  }

  // If either left or right velocity is not found, log a warning
  if (!left_found || !right_found) {
    RCLCPP_WARN(this->get_logger(), "Missing left or right wheel velocities");
    return;
  }

  RCLCPP_DEBUG(this->get_logger(),
    "Received velocities - Left: %.2f, Right: %.2f", left_vel, right_vel);
  
  // Send the velocity command to the driver
  if (!driver_->sendVelocityCommand(left_vel, right_vel)) {
    RCLCPP_ERROR(this->get_logger(), "Failed to send velocity command to microcontroller");
  }
}

/**
 * @brief Timer callback to request feedback from the driver.
 * 
 * This function requests feedback from the driver and publishes it as a JointState message.
 * It logs the feedback values for debugging purposes.
 * 
 * @return void
 */
void SabertoothDriverNode::feedbackTimerCallback() {

  // Request feedback from the driver
  auto feedback = driver_->requestFeedback();

  // If feedback is valid, create a JointState message and publish it
  // Otherwise, log an error message
  if(feedback.has_value()) {
    sensor_msgs::msg::JointState feedback_msg;
    feedback_msg.header.stamp = this->now();
    feedback_msg.name = {"front_left_joint", "front_right_joint"};
    feedback_msg.velocity = {
      feedback->front_left_velocity_rad_s,
      feedback->front_right_velocity_rad_s
    };
    feedback_msg.position = {
      feedback->front_left_position_rad,
      feedback->front_right_position_rad
    };
    
    // Publish the feedback message
    feedback_pub_->publish(feedback_msg);
    
    RCLCPP_DEBUG(this->get_logger(),
    "Feedback - FL Vel: %.2f, FL Pos: %.2f, FR Vel: %.2f, FR Pos: %.2f",
    feedback->front_left_velocity_rad_s,
    feedback->front_left_position_rad,
    feedback->front_right_velocity_rad_s,
    feedback->front_right_position_rad);
  } else {
    
    // Log an error if feedback is not available
    RCLCPP_ERROR(this->get_logger(), "Failed to get feedback from driver");
  }
}

} // namespace sabertooth_motor_driver
