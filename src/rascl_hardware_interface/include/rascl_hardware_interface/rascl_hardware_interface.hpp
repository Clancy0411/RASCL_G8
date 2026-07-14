#ifndef RASCL_HARDWARE_INTERFACE__RASCL_HARDWARE_INTERFACE_HPP_
#define RASCL_HARDWARE_INTERFACE__RASCL_HARDWARE_INTERFACE_HPP_

#include <cstdint>
#include <mutex>
#include <string>
#include <vector>

#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp_lifecycle/state.hpp"

namespace rascl_hardware_interface {

/**
 * @brief Runtime configuration for one physical RASCL drive axis.
 *
 * All ROS 2 command/state values are expressed in SI units, i.e. radians for
 * the four actuated revolute joints. Conversion to and from Faulhaber drive
 * counts is done at the hardware-interface boundary.
 */
struct JointConfig {
  /** @brief ROS joint name as defined in the URDF and controller configuration. */
  std::string name;

  /** @brief EtherCAT slave index used by the Python bridge for this joint. */
  int slave_index{0};

  /** @brief Raw drive counts that correspond to one output-side revolution. */
  double counts_per_revolution{1.0};

  /** @brief Raw drive counts that correspond to one radian at the joint. */
  double counts_per_rad{1.0};

  /** @brief Sign used to align the drive count direction with the ROS joint axis. */
  double direction{1.0};

  /** @brief Raw count value that is treated as the ROS zero position. */
  int64_t home_offset_counts{0};

  /** @brief Minimum accepted ROS command in radians. */
  double min_position{-3.14159265358979323846};

  /** @brief Maximum accepted ROS command in radians. */
  double max_position{3.14159265358979323846};

  /** @brief Initial fake-hardware position or fallback command before activation. */
  double initial_position{0.0};
};

/**
 * @brief ros2_control SystemInterface for the four driven RASCL joints.
 *
 * The plugin satisfies the lifecycle expected by controller_manager and keeps
 * the controller-facing side in C++. EtherCAT access is delegated to the local
 * Python pysoem bridge through a small line-based TCP protocol. This keeps the
 * previously tested pysoem communication path while presenting a regular
 * ros2_control hardware plugin to ROS 2.
 */
class RASCLHardwareInterface : public hardware_interface::SystemInterface {
 public:
  RCLCPP_SHARED_PTR_DEFINITIONS(RASCLHardwareInterface)

  /** @brief Parse hardware and joint parameters from the URDF ros2_control tag. */
  hardware_interface::CallbackReturn on_init(const hardware_interface::HardwareInfo& info) override;

  /** @brief Open the bridge connection or initialize the fake-hardware state. */
  hardware_interface::CallbackReturn on_configure(
      const rclcpp_lifecycle::State& previous_state) override;

  /** @brief Enable all drives and synchronize ROS commands to current positions. */
  hardware_interface::CallbackReturn on_activate(
      const rclcpp_lifecycle::State& previous_state) override;

  /** @brief Stop command output and disable the drives when real hardware is used. */
  hardware_interface::CallbackReturn on_deactivate(
      const rclcpp_lifecycle::State& previous_state) override;

  /** @brief Release resources after deactivation or failed configuration. */
  hardware_interface::CallbackReturn on_cleanup(
      const rclcpp_lifecycle::State& previous_state) override;

  /** @brief Export joint position and velocity state interfaces. */
  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

  /** @brief Export joint position command interfaces. */
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  /** @brief Read actual joint states from EtherCAT or fake hardware. */
  hardware_interface::return_type read(const rclcpp::Time& time,
                                       const rclcpp::Duration& period) override;

  /** @brief Write position set-points to the drives or fake hardware. */
  hardware_interface::return_type write(const rclcpp::Time& time,
                                        const rclcpp::Duration& period) override;

 private:
  friend class RASCLHardwareInterfaceTestPeer;

  /** @brief Establish the TCP connection to the local pysoem bridge. */
  bool connect_to_bridge();

  /** @brief Close the TCP socket if it is currently open. */
  void close_socket();

  /** @brief Send one line-based bridge command and wait for its single-line reply. */
  bool send_command(const std::string& command, std::string& response);

  /** @brief Write a complete byte sequence to the TCP socket. */
  bool send_all(const std::string& data);

  /** @brief Read one newline-terminated bridge response. */
  bool read_line(std::string& line);

  /** @brief Convert a ROS joint angle in radians to a Faulhaber target count. */
  int64_t radians_to_counts(std::size_t joint_index, double radians) const;

  /** @brief Convert a Faulhaber actual count to a ROS joint angle in radians. */
  double counts_to_radians(std::size_t joint_index, int64_t counts) const;

  /** @brief Apply the configured software command limits for one joint. */
  double clamp_command(std::size_t joint_index, double command) const;

  /** @brief TCP socket connected to the local bridge, or -1 when disconnected. */
  int socket_fd_{-1};

  /** @brief Guards command/reply exchanges so read() and write() do not interleave. */
  std::mutex socket_mutex_;

  /** @brief Selects deterministic fake state updates instead of EtherCAT access. */
  bool use_fake_hardware_{false};

  /** @brief Selects CSP/PDO setpoint streaming instead of Profile Position MOVE_ALL. */
  bool use_csp_mode_{false};

  /** @brief Human-readable lower-level control mode parsed from the URDF. */
  std::string control_mode_{"csp"};

  /** @brief True after the first command has been synchronized with current state. */
  bool command_initialized_{false};

  /** @brief TCP host where the Python bridge listens. */
  std::string host_{"127.0.0.1"};

  /** @brief TCP port where the Python bridge listens. */
  int port_{15001};

  /** @brief Number of bridge connection attempts during configuration. */
  int connect_retries_{80};

  /** @brief Delay between bridge connection attempts in seconds. */
  double connect_retry_delay_s_{0.25};

  /** @brief Minimum target-count change required before sending Profile MOVE_ALL. */
  double command_deadband_counts_{4.0};

  /** @brief Per-joint conversion and limit parameters parsed from ros2_control. */
  std::vector<JointConfig> joint_configs_;

  /** @brief Controller-facing joint positions in radians. */
  std::vector<double> hw_positions_;

  /** @brief Controller-facing joint velocities in radians per second. */
  std::vector<double> hw_velocities_;

  /** @brief Latest position commands received from ros2_control controllers. */
  std::vector<double> hw_commands_;

  /** @brief Previous read-cycle positions used for velocity estimation. */
  std::vector<double> last_positions_;

  /** @brief Latest raw actual counts reported by the Faulhaber drives. */
  std::vector<int64_t> actual_counts_;

  /** @brief Last target counts sent to the bridge, used for command deadbanding. */
  std::vector<int64_t> last_command_counts_;
};

}  // namespace rascl_hardware_interface

#endif  // RASCL_HARDWARE_INTERFACE__RASCL_HARDWARE_INTERFACE_HPP_
