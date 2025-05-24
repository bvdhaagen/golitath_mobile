#ifndef GOLIATH_CONTROLLER__GOLIATH_HARDWARE_HPP_
#define GOLIATH_CONTROLLER__GOLIATH_HARDWARE_HPP_

#include <boost/asio.hpp>
#include <boost/thread.hpp>
#include <memory>
#include <string>
#include <vector>
#include <iostream>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"  // For return_type
#include "rclcpp/rclcpp.hpp"

#define SERIAL_PORT_READ_BUF_SIZE 256  // Added buffer size definition

namespace goliath_controller
{
using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;
using hardware_interface::return_type;  // Explicitly bring in return_type

class RobotSystem : public hardware_interface::SystemInterface
{
public:
  CallbackReturn on_init(const hardware_interface::HardwareInfo &info) override;
  CallbackReturn on_activate(const rclcpp_lifecycle::State &) override;
  CallbackReturn on_deactivate(const rclcpp_lifecycle::State &) override;

  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  return_type read(const rclcpp::Time &, const rclcpp::Duration &) override;
  return_type write(const rclcpp::Time &, const rclcpp::Duration &) override;

private:
  // Boost.Asio members
  boost::asio::io_service io_service_;
  std::shared_ptr<boost::asio::serial_port> serial_port_;
  boost::thread io_thread_;
  boost::mutex mutex_;
  char read_buf_raw_[SERIAL_PORT_READ_BUF_SIZE]{};  // Added buffer declaration

  // Joint states
  std::vector<double> joint_position_command_;
  std::vector<double> joint_position_state_;
  std::vector<double> joint_velocity_state_;

  // Configuration
  std::string port_name_;
  int baud_rate_;
  int timeout_ms_;

  
};

}  // namespace goliath_controller

#endif  // GOLIATH_CONTROLLER__GOLIATH_HARDWARE_HPP_
