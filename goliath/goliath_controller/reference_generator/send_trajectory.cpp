#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/chainiksolvervel_pinv.hpp>
#include <kdl/jntarray.hpp>
#include <kdl/tree.hpp>
#include <kdl_parser/kdl_parser.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <chrono>

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("send_trajectory");
  auto pub = node->create_publisher<std_msgs::msg::Float64MultiArray>(
    "/position_controller/commands", 10);

  // get robot description
  auto robot_param = rclcpp::Parameter();
  node->declare_parameter("robot_description", rclcpp::ParameterType::PARAMETER_STRING);
  node->get_parameter("robot_description", robot_param);
  auto robot_description = robot_param.as_string();

  // create kinematic chain
  KDL::Tree robot_tree;
  KDL::Chain chain;
  kdl_parser::treeFromString(robot_description, robot_tree);
  robot_tree.getChain("base_link", "tool0", chain);

  auto joint_positions = KDL::JntArray(chain.getNrOfJoints());
  auto joint_velocities = KDL::JntArray(chain.getNrOfJoints());
  auto twist = KDL::Twist();
  
  // create KDL solvers
  auto ik_vel_solver_ = std::make_shared<KDL::ChainIkSolverVel_pinv>(chain, 0.0000001);

  // Control parameters
  double total_time = 4.0;
  int trajectory_len = 200;
  double control_rate = 50.0;  // Hz
  double dt = 1.0 / control_rate;
  
  // Main control loop
  auto start_time = node->now();
  rclcpp::Rate rate(control_rate);
  
  while (rclcpp::ok()) {
    auto elapsed = (node->now() - start_time).seconds();
    if (elapsed > total_time) break;

    double t = elapsed * control_rate;
    
    // Set endpoint twist (same as your original)
    twist.vel.x(0.4 * 0.2 * cos(2 * M_PI * t / trajectory_len));
    twist.vel.y(0.5 * sin(2 * M_PI * t / trajectory_len));
    twist.vel.z(2 * 0.1 * cos(2 * M_PI * t / trajectory_len));
    
    // Convert cart to joint velocities
    ik_vel_solver_->CartToJnt(joint_positions, twist, joint_velocities);

    // Integrate joint velocities
    joint_positions.data += joint_velocities.data * dt;

    // Create and publish Float64MultiArray message
    std_msgs::msg::Float64MultiArray command_msg;
    command_msg.data.resize(chain.getNrOfJoints());
    std::memcpy(command_msg.data.data(), joint_positions.data.data(), 
               chain.getNrOfJoints() * sizeof(double));
    
    pub->publish(command_msg);
    
    // Debug output
    RCLCPP_INFO_THROTTLE(node->get_logger(), *node->get_clock(), 1000, 
                        "Publishing joint positions: [%.2f, %.2f, %.2f, %.2f, %.2f, %.2f]",
                        joint_positions.data[0], joint_positions.data[1],
                        joint_positions.data[2], joint_positions.data[3],
                        joint_positions.data[4], joint_positions.data[5]);

    rate.sleep();
    rclcpp::spin_some(node);
  }

  return 0;
}
