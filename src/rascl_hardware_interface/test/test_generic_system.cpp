#include <string>

#include "gtest/gtest.h"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rascl_hardware_interface/rascl_hardware_interface.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/state.hpp"

namespace rascl_hardware_interface {

class RASCLHardwareInterfaceTestPeer {
 public:
  static int64_t RadiansToCounts(const RASCLHardwareInterface& hardware,
                                 std::size_t joint_index, double radians) {
    return hardware.radians_to_counts(joint_index, radians);
  }

  static double CountsToRadians(const RASCLHardwareInterface& hardware,
                                std::size_t joint_index, int64_t counts) {
    return hardware.counts_to_radians(joint_index, counts);
  }
};

}  // namespace rascl_hardware_interface

namespace {

constexpr double kHalfPi = 1.57079632679489661923;

hardware_interface::InterfaceInfo MakeInterface(const std::string& name) {
  hardware_interface::InterfaceInfo interface;
  interface.name = name;
  return interface;
}

hardware_interface::ComponentInfo MakeJoint(const std::string& name, int slave_index,
                                            double counts_per_revolution,
                                            double initial_position,
                                            int64_t home_offset_counts = 0) {
  hardware_interface::ComponentInfo joint;
  joint.name = name;
  joint.type = "joint";

  joint.command_interfaces.push_back(MakeInterface(hardware_interface::HW_IF_POSITION));
  joint.state_interfaces.push_back(MakeInterface(hardware_interface::HW_IF_POSITION));
  joint.state_interfaces.push_back(MakeInterface(hardware_interface::HW_IF_VELOCITY));

  joint.parameters["slave_index"] = std::to_string(slave_index);
  joint.parameters["counts_per_revolution"] = std::to_string(counts_per_revolution);
  joint.parameters["direction"] = "1.0";
  joint.parameters["home_offset_counts"] = std::to_string(home_offset_counts);
  joint.parameters["min_position"] = "-3.141592653589793";
  joint.parameters["max_position"] = "3.141592653589793";
  joint.parameters["initial_position"] = std::to_string(initial_position);

  return joint;
}

hardware_interface::HardwareInfo MakeFakeHardwareInfo() {
  hardware_interface::HardwareInfo info;
  info.name = "RasclBotHardware";
  info.type = "system";
  info.hardware_plugin_name = "rascl_hardware_interface/RASCLHardwareInterface";

  info.hardware_parameters["use_fake_hardware"] = "true";
  info.hardware_parameters["host"] = "127.0.0.1";
  info.hardware_parameters["port"] = "15001";
  info.hardware_parameters["connect_retries"] = "1";
  info.hardware_parameters["connect_retry_delay_s"] = "0.0";
  info.hardware_parameters["command_deadband_counts"] = "4.0";
  info.hardware_parameters["control_mode"] = "csp";

  info.joints.push_back(MakeJoint("shoulder_joint", 0, 3211264.0, 0.0));
  info.joints.push_back(MakeJoint("upperarm_joint", 1, 3211264.0, 0.1));
  info.joints.push_back(MakeJoint("lowerarm_joint", 2, 3211264.0, -0.1));
  info.joints.push_back(MakeJoint("spur_gear_joint", 3, 1323008.0, 0.0));

  return info;
}

}  // namespace

TEST(RASCLHardwareInterfaceTest, InitializesAndRunsFakeHardwareLifecycle) {
  rascl_hardware_interface::RASCLHardwareInterface hardware;
  const auto info = MakeFakeHardwareInfo();

  EXPECT_EQ(hardware.on_init(info), hardware_interface::CallbackReturn::SUCCESS);

  const auto state_interfaces = hardware.export_state_interfaces();
  const auto command_interfaces = hardware.export_command_interfaces();
  EXPECT_EQ(state_interfaces.size(), info.joints.size() * 2);
  EXPECT_EQ(command_interfaces.size(), info.joints.size());

  const rclcpp_lifecycle::State state;
  EXPECT_EQ(hardware.on_configure(state), hardware_interface::CallbackReturn::SUCCESS);
  EXPECT_EQ(hardware.on_activate(state), hardware_interface::CallbackReturn::SUCCESS);
  EXPECT_EQ(hardware.read(rclcpp::Time(0), rclcpp::Duration::from_seconds(0.1)),
            hardware_interface::return_type::OK);
  EXPECT_EQ(hardware.write(rclcpp::Time(0), rclcpp::Duration::from_seconds(0.1)),
            hardware_interface::return_type::OK);
  EXPECT_EQ(hardware.on_deactivate(state), hardware_interface::CallbackReturn::SUCCESS);
  EXPECT_EQ(hardware.on_cleanup(state), hardware_interface::CallbackReturn::SUCCESS);
}

TEST(RASCLHardwareInterfaceTest, RejectsJointWithoutPositionCommandInterface) {
  auto info = MakeFakeHardwareInfo();
  info.joints[0].command_interfaces.clear();

  rascl_hardware_interface::RASCLHardwareInterface hardware;
  EXPECT_EQ(hardware.on_init(info), hardware_interface::CallbackReturn::ERROR);
}

TEST(RASCLHardwareInterfaceTest, RejectsJointWithoutVelocityStateInterface) {
  auto info = MakeFakeHardwareInfo();
  info.joints[0].state_interfaces.clear();
  info.joints[0].state_interfaces.push_back(MakeInterface(hardware_interface::HW_IF_POSITION));

  rascl_hardware_interface::RASCLHardwareInterface hardware;
  EXPECT_EQ(hardware.on_init(info), hardware_interface::CallbackReturn::ERROR);
}

TEST(RASCLHardwareInterfaceTest, AppliesReferenceSwitchToUrdfZeroCalibration) {
  auto info = MakeFakeHardwareInfo();
  info.joints[1].parameters["home_offset_counts"] = "-802816";
  // Drive 2 encoder counts run opposite to the lowerarm URDF axis.  The
  // direction and offset signs must be changed together so raw Home zero still
  // maps to +pi/2.
  info.joints[2].parameters["direction"] = "-1.0";
  info.joints[2].parameters["home_offset_counts"] = "802816";

  rascl_hardware_interface::RASCLHardwareInterface hardware;
  ASSERT_EQ(hardware.on_init(info), hardware_interface::CallbackReturn::SUCCESS);

  using Peer = rascl_hardware_interface::RASCLHardwareInterfaceTestPeer;
  EXPECT_NEAR(Peer::CountsToRadians(hardware, 0, 0), 0.0, 1e-12);
  EXPECT_NEAR(Peer::CountsToRadians(hardware, 1, 0), kHalfPi, 1e-12);
  EXPECT_NEAR(Peer::CountsToRadians(hardware, 2, 0), kHalfPi, 1e-12);
  EXPECT_NEAR(Peer::CountsToRadians(hardware, 3, 0), 0.0, 1e-12);

  EXPECT_EQ(Peer::RadiansToCounts(hardware, 0, 0.0), 0);
  EXPECT_EQ(Peer::RadiansToCounts(hardware, 1, 0.0), -802816);
  EXPECT_EQ(Peer::RadiansToCounts(hardware, 2, 0.0), 802816);
  EXPECT_EQ(Peer::RadiansToCounts(hardware, 3, 0.0), 0);
  EXPECT_EQ(Peer::RadiansToCounts(hardware, 1, kHalfPi), 0);
  EXPECT_EQ(Peer::RadiansToCounts(hardware, 2, kHalfPi), 0);
  EXPECT_EQ(Peer::RadiansToCounts(hardware, 2, -kHalfPi), 1605632);
}

TEST(RASCLHardwareInterfaceTest, RejectsUnsupportedControlMode) {
  auto info = MakeFakeHardwareInfo();
  info.hardware_parameters["control_mode"] = "torque";

  rascl_hardware_interface::RASCLHardwareInterface hardware;
  EXPECT_EQ(hardware.on_init(info), hardware_interface::CallbackReturn::ERROR);
}
