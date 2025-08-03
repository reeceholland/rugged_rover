#include "rugged_rover_hardware_interfaces/sabertooth/sabertooth_system_interface.hpp"

#include <string>
#include <vector>

#include "hardware_interface/types/hardware_interface_type_values.hpp"

namespace rugged_rover_hardware_interfaces::sabertooth
{

  /**
   * @brief Initializes the Sabertooth System Interface.
   *
   * This function initializes the system interface with the provided hardware information.
   *
   * @param info The hardware information containing details about the system.
   *
   * @return CallbackReturn::SUCCESS if initialization is successful, otherwise returns an error.
   */
  hardware_interface::CallbackReturn
  SabertoothSystemInterface::on_init(const hardware_interface::HardwareInfo& info)
  {
    // Initialise common hardware interface properties
    if (hardware_interface::SystemInterface::on_init(info) !=
        hardware_interface::CallbackReturn::SUCCESS)
    {

      return hardware_interface::CallbackReturn::ERROR;
    }

    // Allocate memory for the number of joints specified in URDF
    joint_names_.reserve(info.joints.size());

    // Store each joint name in the joint_names_ vector
    for (const auto& joint : info.joints)
    {
      joint_names_.push_back(joint.name);
    }

    // Resize the hardware state and command vectors to match the number of joints
    hw_positions_.resize(joint_names_.size(), 0.0);
    hw_velocities_.resize(joint_names_.size(), 0.0);
    hw_commands_.resize(joint_names_.size(), 0.0);

    // Return success if initialisation is complete
    return hardware_interface::CallbackReturn::SUCCESS;
  }

  /**
   * @brief Activates the Sabertooth System Interface.
   *
   * This function sets up the node, subscriptions, and publishers required for the system
   * interface.
   *
   * @return CallbackReturn::SUCCESS if activation is successful, otherwise returns an error.
   */
  hardware_interface::CallbackReturn
  SabertoothSystemInterface::on_activate(const rclcpp_lifecycle::State&)
  {
    // Log activation message
    RCLCPP_INFO(rclcpp::get_logger("SabertoothSystemInterface"),
                "Activating Sabertooth System Interface");

    // Create a new node for the system interface
    node_ = rclcpp::Node::make_shared("sabertooth_hw_interface_node");

    // Create a subscription to the feedback topic
    feedback_sub_ = node_->create_subscription<sensor_msgs::msg::JointState>(
        "platform/motors/feedback", rclcpp::SensorDataQoS(),
        std::bind(&SabertoothSystemInterface::feedbackCallback, this, std::placeholders::_1));

    // Create a publisher for the command topic
    cmd_pub_ = node_->create_publisher<sensor_msgs::msg::JointState>("platform/motors/cmd",
                                                                     rclcpp::SensorDataQoS());

    // Return success if activation is complete
    return hardware_interface::CallbackReturn::SUCCESS;
  }

  /**
   * @brief Deactivates the Sabertooth System Interface.
   *
   * This function cleans up the node, subscriptions, and publishers created during activation.
   *
   * @return CallbackReturn::SUCCESS if deactivation is successful, otherwise returns an error.
   */
  hardware_interface::CallbackReturn
  SabertoothSystemInterface::on_deactivate(const rclcpp_lifecycle::State&)
  {
    // Log deactivation message
    RCLCPP_INFO(rclcpp::get_logger("SabertoothSystemInterface"),
                "Deactivating hardware interface...");

    // Reset the node, subscription, and publisher to clean up resources
    feedback_sub_.reset();
    cmd_pub_.reset();
    node_.reset();

    // Return success if deactivation is complete
    return hardware_interface::CallbackReturn::SUCCESS;
  }

  /**
   * @brief Exports the state interfaces for the Sabertooth System Interface.
   *
   * This function creates and returns a vector of state interfaces that represent the current state
   * of the hardware.
   *
   * @return A vector of StateInterface objects representing the state of the hardware.
   */
  std::vector<hardware_interface::StateInterface>
  SabertoothSystemInterface::export_state_interfaces()
  {
    // Create a vector to hold the state interfaces
    std::vector<hardware_interface::StateInterface> state_interfaces;

    // For each joint defined in the hardware interface,
    for (size_t i = 0; i < joint_names_.size(); ++i)
    {

      // Create state interfaces for position
      state_interfaces.emplace_back(hardware_interface::StateInterface(
          joint_names_[i], hardware_interface::HW_IF_POSITION, &hw_positions_[i]));

      // Create state interfaces for velocity
      state_interfaces.emplace_back(hardware_interface::StateInterface(
          joint_names_[i], hardware_interface::HW_IF_VELOCITY, &hw_velocities_[i]));
    }

    // Return the vector of state interfaces
    return state_interfaces;
  }

  /**
   * @brief Exports the command interfaces for the Sabertooth System Interface.
   *
   * This function creates and returns a vector of command interfaces that allow control over the
   * hardware.
   *
   * @return A vector of CommandInterface objects representing the command interfaces for the
   * hardware.
   */
  std::vector<hardware_interface::CommandInterface>
  SabertoothSystemInterface::export_command_interfaces()
  {
    // Create a vector to hold the command interfaces
    std::vector<hardware_interface::CommandInterface> command_interfaces;

    // For each joint defined in the hardware interface,
    for (size_t i = 0; i < joint_names_.size(); ++i)
    {
      // Create command interfaces for velocity
      command_interfaces.emplace_back(hardware_interface::CommandInterface(
          joint_names_[i], hardware_interface::HW_IF_VELOCITY, &hw_commands_[i]));
    }

    // Return the vector of command interfaces
    return command_interfaces;
  }

  hardware_interface::return_type SabertoothSystemInterface::read(const rclcpp::Time&,
                                                                  const rclcpp::Duration&)
  {
    std::lock_guard<std::mutex> lock(feedback_mutex_);

    // Update positions and velocities from the most recent feedback
    for (size_t i = 0; i < joint_names_.size(); ++i)
    {
      // Find the index of the joint in the last feedback message
      auto it = std::find(last_feedback_.name.begin(), last_feedback_.name.end(), joint_names_[i]);

      // If the joint is found.
      if (it != last_feedback_.name.end())
      {
        // Get the index of the joint in the feedback message
        size_t index = std::distance(last_feedback_.name.begin(), it);

        // Update the hardware positions and velocities if the index is valid
        if (index < last_feedback_.position.size())
          hw_positions_[i] = last_feedback_.position[index];
        if (index < last_feedback_.velocity.size())
          hw_velocities_[i] = last_feedback_.velocity[index];
      }
    }
    // Return OK to indicate successful read operation
    return hardware_interface::return_type::OK;
  }

  /**
   * @brief Writes commands to the hardware.
   *
   * This function publishes the current command values to the hardware via a ROS topic.
   *
   * @param time The current time.
   * @param period The time period since the last write.
   *
   * @return hardware_interface::return_type::OK if the write operation is successful, otherwise
   * returns an error.
   */
  hardware_interface::return_type SabertoothSystemInterface::write(const rclcpp::Time&,
                                                                   const rclcpp::Duration&)
  {
    // Check if the command publisher and node are initialized
    if (!cmd_pub_ || !node_)
    {
      RCLCPP_ERROR(this->logger_, "Command publisher or node is not initialized.");
      return hardware_interface::return_type::ERROR;
    }
    // Create a JointState message to publish the commands
    sensor_msgs::msg::JointState cmd_msg;

    // Set the header for the command message
    cmd_msg.header.stamp = node_->now();

    // Set the joint names and commands in the command message
    cmd_msg.name = joint_names_;
    cmd_msg.velocity = hw_commands_;

    // Publish the command message to the topic

    cmd_pub_->publish(cmd_msg);

    // Return OK to indicate successful write operation
    return hardware_interface::return_type::OK;
  }

  /**
   * @brief Callback function for receiving feedback messages.
   *
   * This function is called whenever a new feedback message is received from the ROS topic.
   * It updates the last_feedback_ member variable with the received message.
   *
   * @param msg The received feedback message.
   */
  void
  SabertoothSystemInterface::feedbackCallback(const sensor_msgs::msg::JointState::SharedPtr msg)
  {
    // Lock the feedback mutex to ensure thread safety
    std::lock_guard<std::mutex> lock(feedback_mutex_);

    // Update the last_feedback_ member variable with the received message
    last_feedback_ = *msg;
  }

} // namespace rugged_rover_hardware_interfaces::sabertooth

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(rugged_rover_hardware_interfaces::sabertooth::SabertoothSystemInterface,
                       hardware_interface::SystemInterface)