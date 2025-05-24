#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/chainiksolverpos_nr.hpp>
#include <kdl/chainiksolvervel_pinv.hpp>
#include <kdl/tree.hpp>
#include <kdl_parser/kdl_parser.hpp>
#include <rclcpp/rclcpp.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("send_trajectory");
  auto pub = node->create_publisher<trajectory_msgs::msg::JointTrajectory>(
    "/goliath_controller/joint_trajectory", 10);

  // Get robot description
  std::string robot_description;
  node->declare_parameter("robot_description", "");
  node->get_parameter("robot_description", robot_description);

  // Create kinematic chain
  KDL::Tree robot_tree;
  KDL::Chain chain;
  if (!kdl_parser::treeFromString(robot_description, robot_tree)) {
    RCLCPP_ERROR(node->get_logger(), "Failed to construct kdl tree");
    return 1;
  }
  if (!robot_tree.getChain("base_link", "tool0", chain)) {
    RCLCPP_ERROR(node->get_logger(), "Failed to get chain from tree");
    return 1;
  }

  // Create solvers
  auto fk_solver = std::make_shared<KDL::ChainFkSolverPos_recursive>(chain);
  auto ik_vel_solver = std::make_shared<KDL::ChainIkSolverVel_pinv>(chain);
  auto ik_pos_solver = std::make_shared<KDL::ChainIkSolverPos_NR>(chain, *fk_solver, *ik_vel_solver, 100, 1e-6);

  // Prepare trajectory message
  trajectory_msgs::msg::JointTrajectory trajectory_msg;
  trajectory_msg.header.stamp = node->now();
  
  // Add joint names
  for (size_t i = 0; i < chain.getNrOfSegments(); i++) {
    auto joint = chain.getSegment(i).getJoint();
    if (joint.getType() != KDL::Joint::Fixed) {
      trajectory_msg.joint_names.push_back(joint.getName());
    }
  }

  // Parameters
  const double target_z = 0.4;  // 40 cm height
  const double circle_radius = 0.3;  // 30 cm radius
  const double circle_center_x = 0.5;  // 50 cm from base
  const double total_time = 4.0;  // 4 seconds total
  const int num_points = 200;  // Number of trajectory points

  for (int i = 0; i < num_points; i++) {
    double t = static_cast<double>(i) / num_points * total_time;
    KDL::Frame target_pose;
    
    if (t < 1.0) {
      // Move linearly to starting position (0.5, 0, 0.4)
      double progress = t / 1.0;
      target_pose.p = KDL::Vector(
        circle_center_x * progress,
        0.0,
        target_z * progress
      );
    } else {
      // Circular motion in XY plane
      double angle = 2.0 * M_PI * (t - 1.0) / 3.0;  // Takes 3 seconds
      target_pose.p = KDL::Vector(
        circle_center_x + circle_radius * cos(angle),
        circle_radius * sin(angle),
        target_z
      );
    }
    
    // Keep orientation fixed
    target_pose.M = KDL::Rotation::Identity();
    
    // Solve IK
    KDL::JntArray joint_positions(chain.getNrOfJoints());
    if (ik_pos_solver->CartToJnt(joint_positions, target_pose, joint_positions) < 0) {
      RCLCPP_ERROR(node->get_logger(), "IK solution failed at point %d", i);
      continue;
    }
    
    // Create trajectory point
    trajectory_msgs::msg::JointTrajectoryPoint point;
    point.positions.resize(chain.getNrOfJoints());
    for (size_t j = 0; j < chain.getNrOfJoints(); j++) {
      point.positions[j] = joint_positions(j);
    }
    point.time_from_start = rclcpp::Duration::from_seconds(t);
    trajectory_msg.points.push_back(point);
  }

  // Publish trajectory
  pub->publish(trajectory_msg);
  RCLCPP_INFO(node->get_logger(), "Published trajectory with %zu points", trajectory_msg.points.size());

  rclcpp::spin(node);
  return 0;
}
