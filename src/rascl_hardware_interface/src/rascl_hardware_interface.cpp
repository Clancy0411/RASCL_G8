#include "rascl_hardware_interface/rascl_hardware_interface.hpp"

#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <unistd.h>

#include <algorithm>
#include <cerrno>
#include <chrono>
#include <cmath>
#include <cstdlib>
#include <cstring>
#include <limits>
#include <sstream>
#include <string>
#include <thread>
#include <unordered_map>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "rclcpp/rclcpp.hpp"

namespace rascl_hardware_interface {
namespace {

// Full revolution in radians, used to convert between joint angles and drive counts.
constexpr double kTwoPi = 6.283185307179586476925286766559;

// Fallback count conversions used only if the URDF does not provide joint values.
constexpr double kDefaultAxisCountsPerRevolution = 802816.0;      // 4096 * 196.
constexpr double kDefaultGripperCountsPerRevolution = 1323008.0;  // 4096 * 323.

// Return a ros2_control parameter value, or a safe default when it is omitted.
std::string GetParameterOr(const std::unordered_map<std::string, std::string>& parameters,
                           const std::string& name, const std::string& default_value) {
  const auto it = parameters.find(name);
  return it == parameters.end() ? default_value : it->second;
}

// Parse common textual boolean values used in launch and URDF parameters.
bool ParseBool(const std::string& value) {
  return value == "true" || value == "True" || value == "TRUE" || value == "1" || value == "yes" ||
         value == "on";
}

// Check that a URDF joint declares the state or command interface required here.
bool HasInterface(const std::vector<hardware_interface::InterfaceInfo>& interfaces,
                  const std::string& name) {
  return std::any_of(interfaces.begin(), interfaces.end(),
                     [&name](const hardware_interface::InterfaceInfo& interface) {
                       return interface.name == name;
                     });
}

}  // namespace

hardware_interface::CallbackReturn RASCLHardwareInterface::on_init(
    const hardware_interface::HardwareInfo& info) {
  // Let the base class parse the ros2_control hardware information first.

  if (hardware_interface::SystemInterface::on_init(info) !=
      hardware_interface::CallbackReturn::SUCCESS) {
    return hardware_interface::CallbackReturn::ERROR;
  }

  if (info_.joints.empty()) {
    RCLCPP_FATAL(rclcpp::get_logger("RASCLHardwareInterface"),
                 "No joints found in ros2_control tag.");
    return hardware_interface::CallbackReturn::ERROR;
  }

  // Hardware-level parameters configure the bridge endpoint and command filtering.
  host_ = GetParameterOr(info_.hardware_parameters, "host", "127.0.0.1");
  port_ = std::stoi(GetParameterOr(info_.hardware_parameters, "port", "15001"));
  connect_retries_ = std::stoi(GetParameterOr(info_.hardware_parameters, "connect_retries", "80"));
  connect_retry_delay_s_ =
      std::stod(GetParameterOr(info_.hardware_parameters, "connect_retry_delay_s", "0.25"));
  command_deadband_counts_ =
      std::stod(GetParameterOr(info_.hardware_parameters, "command_deadband_counts", "4.0"));
  use_fake_hardware_ =
      ParseBool(GetParameterOr(info_.hardware_parameters, "use_fake_hardware", "false"));

  // Each ros2_control joint maps to one Faulhaber drive selected by slave_index.
  joint_configs_.clear();
  joint_configs_.reserve(info_.joints.size());

  for (std::size_t i = 0; i < info_.joints.size(); ++i) {
    const auto& joint = info_.joints[i];

    if (!HasInterface(joint.command_interfaces, hardware_interface::HW_IF_POSITION)) {
      RCLCPP_FATAL(rclcpp::get_logger("RASCLHardwareInterface"),
                   "Joint '%s' needs a position command interface.", joint.name.c_str());
      return hardware_interface::CallbackReturn::ERROR;
    }
    if (!HasInterface(joint.state_interfaces, hardware_interface::HW_IF_POSITION) ||
        !HasInterface(joint.state_interfaces, hardware_interface::HW_IF_VELOCITY)) {
      RCLCPP_FATAL(rclcpp::get_logger("RASCLHardwareInterface"),
                   "Joint '%s' needs position and velocity state interfaces.", joint.name.c_str());
      return hardware_interface::CallbackReturn::ERROR;
    }

    // Conversion and limit parameters are kept per joint so that the arm axes and
    // the end-effector can use different gear ratios and count conventions.
    JointConfig config;
    config.name = joint.name;
    config.slave_index =
        std::stoi(GetParameterOr(joint.parameters, "slave_index", std::to_string(i)));
    const double fallback_counts =
        i < 3 ? kDefaultAxisCountsPerRevolution : kDefaultGripperCountsPerRevolution;
    config.counts_per_revolution = std::stod(
        GetParameterOr(joint.parameters, "counts_per_revolution", std::to_string(fallback_counts)));
    config.direction = std::stod(GetParameterOr(joint.parameters, "direction", "1.0"));
    if (std::abs(config.direction) < std::numeric_limits<double>::epsilon()) {
      config.direction = 1.0;
    }
    config.home_offset_counts =
        std::stoll(GetParameterOr(joint.parameters, "home_offset_counts", "0"));
    config.min_position =
        std::stod(GetParameterOr(joint.parameters, "min_position", "-3.141592653589793"));
    config.max_position =
        std::stod(GetParameterOr(joint.parameters, "max_position", "3.141592653589793"));
    config.initial_position =
        std::stod(GetParameterOr(joint.parameters, "initial_position", "0.0"));
    config.counts_per_rad = config.counts_per_revolution / kTwoPi;

    joint_configs_.push_back(config);
  }

  const std::size_t joint_count = joint_configs_.size();

  // Allocate controller-facing state and command buffers with one entry per joint.
  hw_positions_.assign(joint_count, 0.0);
  hw_velocities_.assign(joint_count, 0.0);
  hw_commands_.assign(joint_count, 0.0);
  last_positions_.assign(joint_count, 0.0);
  actual_counts_.assign(joint_count, 0);
  last_command_counts_.assign(joint_count, 0);

  for (std::size_t i = 0; i < joint_count; ++i) {
    // Before activation, initialize commands to a safe clamped value. Real hardware
    // activation later overwrites this with the measured position to avoid jumps.
    hw_positions_[i] = clamp_command(i, joint_configs_[i].initial_position);
    hw_commands_[i] = hw_positions_[i];
    last_positions_[i] = hw_positions_[i];
    actual_counts_[i] = radians_to_counts(i, hw_positions_[i]);
    last_command_counts_[i] = actual_counts_[i];
  }

  RCLCPP_INFO(rclcpp::get_logger("RASCLHardwareInterface"),
              "Initialized %zu RASCL joints. fake_hardware=%s bridge=%s:%d", joint_count,
              use_fake_hardware_ ? "true" : "false", host_.c_str(), port_);

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn RASCLHardwareInterface::on_configure(
    const rclcpp_lifecycle::State& /*previous_state*/) {
  if (use_fake_hardware_) {
    // Fake mode is fully local and therefore does not need the TCP bridge.
    RCLCPP_INFO(rclcpp::get_logger("RASCLHardwareInterface"), "Configured fake hardware.");
    return hardware_interface::CallbackReturn::SUCCESS;
  }

  RCLCPP_INFO(rclcpp::get_logger("RASCLHardwareInterface"),
              "Connecting to Faulhaber TCP bridge...");
  if (!connect_to_bridge()) {
    RCLCPP_ERROR(rclcpp::get_logger("RASCLHardwareInterface"),
                 "Could not connect to Faulhaber bridge.");
    return hardware_interface::CallbackReturn::ERROR;
  }

  std::string response;
  // Verify that the bridge is alive before controller_manager activates hardware.
  if (!send_command("PING", response) || response != "OK") {
    RCLCPP_ERROR(rclcpp::get_logger("RASCLHardwareInterface"), "Bridge PING failed. response='%s'",
                 response.c_str());
    return hardware_interface::CallbackReturn::ERROR;
  }

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn RASCLHardwareInterface::on_activate(
    const rclcpp_lifecycle::State& /*previous_state*/) {
  command_initialized_ = false;

  if (use_fake_hardware_) {
    // Fake hardware should start without a discontinuity between state and command.
    for (std::size_t i = 0; i < hw_commands_.size(); ++i) {
      hw_commands_[i] = hw_positions_[i];
      last_command_counts_[i] = radians_to_counts(i, hw_commands_[i]);
    }
    command_initialized_ = true;
    RCLCPP_INFO(rclcpp::get_logger("RASCLHardwareInterface"), "Activated fake hardware.");
    return hardware_interface::CallbackReturn::SUCCESS;
  }

  std::string response;
  // The bridge performs the CiA 402 state transitions for all physical drives.
  if (!send_command("ENABLE_ALL", response) || response.rfind("OK", 0) != 0) {
    RCLCPP_ERROR(rclcpp::get_logger("RASCLHardwareInterface"), "ENABLE_ALL failed. response='%s'",
                 response.c_str());
    return hardware_interface::CallbackReturn::ERROR;
  }

  if (read(rclcpp::Time(0), rclcpp::Duration::from_seconds(0.0)) !=
      hardware_interface::return_type::OK) {
    RCLCPP_ERROR(rclcpp::get_logger("RASCLHardwareInterface"), "Initial hardware read failed.");
    return hardware_interface::CallbackReturn::ERROR;
  }

  for (std::size_t i = 0; i < hw_commands_.size(); ++i) {
    // Synchronize the first command with measured hardware position. This prevents
    // controller activation from immediately commanding a stale or default target.
    hw_commands_[i] = hw_positions_[i];
    last_command_counts_[i] = actual_counts_[i];
  }
  command_initialized_ = true;

  RCLCPP_INFO(rclcpp::get_logger("RASCLHardwareInterface"), "Activated real RASCL hardware.");
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn RASCLHardwareInterface::on_deactivate(
    const rclcpp_lifecycle::State& /*previous_state*/) {
  if (!use_fake_hardware_) {
    // Disable drives on shutdown so the hardware does not keep holding commands.
    std::string response;
    if (!send_command("DISABLE_ALL", response)) {
      RCLCPP_WARN(rclcpp::get_logger("RASCLHardwareInterface"),
                  "DISABLE_ALL command could not be sent.");
    }
  }
  close_socket();
  command_initialized_ = false;
  RCLCPP_INFO(rclcpp::get_logger("RASCLHardwareInterface"), "Hardware deactivated.");
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn RASCLHardwareInterface::on_cleanup(
    const rclcpp_lifecycle::State& /*previous_state*/) {
  close_socket();
  command_initialized_ = false;
  // Positions are left untouched, but stale velocities must not be exported later.
  std::fill(hw_velocities_.begin(), hw_velocities_.end(), 0.0);
  RCLCPP_INFO(rclcpp::get_logger("RASCLHardwareInterface"), "Hardware cleanup finished.");
  return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> RASCLHardwareInterface::export_state_interfaces() {
  std::vector<hardware_interface::StateInterface> state_interfaces;
  state_interfaces.reserve(joint_configs_.size() * 2);

  for (std::size_t i = 0; i < joint_configs_.size(); ++i) {
    // ros2_control keeps references to these buffers during controller execution.
    state_interfaces.emplace_back(joint_configs_[i].name, hardware_interface::HW_IF_POSITION,
                                  &hw_positions_[i]);
    state_interfaces.emplace_back(joint_configs_[i].name, hardware_interface::HW_IF_VELOCITY,
                                  &hw_velocities_[i]);
  }

  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface>
RASCLHardwareInterface::export_command_interfaces() {
  std::vector<hardware_interface::CommandInterface> command_interfaces;
  command_interfaces.reserve(joint_configs_.size());

  for (std::size_t i = 0; i < joint_configs_.size(); ++i) {
    // ForwardCommandController writes desired joint positions into these buffers.
    command_interfaces.emplace_back(joint_configs_[i].name, hardware_interface::HW_IF_POSITION,
                                    &hw_commands_[i]);
  }

  return command_interfaces;
}

hardware_interface::return_type RASCLHardwareInterface::read(const rclcpp::Time& /*time*/,
                                                             const rclcpp::Duration& period) {
  const double dt = period.seconds();

  if (use_fake_hardware_) {
    // In fake mode, the state follows the current command without EtherCAT traffic.
    for (std::size_t i = 0; i < joint_configs_.size(); ++i) {
      last_positions_[i] = hw_positions_[i];
      hw_positions_[i] = clamp_command(i, hw_commands_[i]);
      hw_velocities_[i] = dt > 1e-6 ? (hw_positions_[i] - last_positions_[i]) / dt : 0.0;
      actual_counts_[i] = radians_to_counts(i, hw_positions_[i]);
    }
    return hardware_interface::return_type::OK;
  }

  std::string response;
  // GET_ALL returns pairs of raw position counts and CiA 402 status words.
  if (!send_command("GET_ALL", response)) {
    RCLCPP_ERROR(rclcpp::get_logger("RASCLHardwareInterface"), "GET_ALL command failed.");
    return hardware_interface::return_type::ERROR;
  }

  std::istringstream stream(response);
  std::string ok;
  stream >> ok;
  if (ok != "OK") {
    RCLCPP_ERROR(rclcpp::get_logger("RASCLHardwareInterface"), "GET_ALL returned error: '%s'",
                 response.c_str());
    return hardware_interface::return_type::ERROR;
  }

  for (std::size_t i = 0; i < joint_configs_.size(); ++i) {
    int64_t counts = 0;
    std::string status_word;
    if (!(stream >> counts >> status_word)) {
      RCLCPP_ERROR(rclcpp::get_logger("RASCLHardwareInterface"), "Malformed GET_ALL response: '%s'",
                   response.c_str());
      return hardware_interface::return_type::ERROR;
    }

    actual_counts_[i] = counts;
    last_positions_[i] = hw_positions_[i];
    // Velocity is estimated from successive position samples because the bridge
    // only exposes actual positions for the assignment controller interface.

    hw_positions_[i] = counts_to_radians(i, counts);
    hw_velocities_[i] = dt > 1e-6 ? (hw_positions_[i] - last_positions_[i]) / dt : 0.0;
  }

  return hardware_interface::return_type::OK;
}

hardware_interface::return_type RASCLHardwareInterface::write(const rclcpp::Time& /*time*/,
                                                              const rclcpp::Duration& /*period*/) {
  std::vector<int64_t> target_counts(joint_configs_.size(), 0);
  // Force the first write after activation, then suppress count-level duplicates.
  bool changed = !command_initialized_;

  for (std::size_t i = 0; i < joint_configs_.size(); ++i) {
    if (!std::isfinite(hw_commands_[i])) {
      RCLCPP_ERROR(rclcpp::get_logger("RASCLHardwareInterface"),
                   "Non-finite command for joint '%s'.", joint_configs_[i].name.c_str());
      return hardware_interface::return_type::ERROR;
    }

    // Clamp before conversion so invalid controller commands cannot exceed the
    // software limits configured in the URDF.
    hw_commands_[i] = clamp_command(i, hw_commands_[i]);
    target_counts[i] = radians_to_counts(i, hw_commands_[i]);
    if (std::llabs(target_counts[i] - last_command_counts_[i]) >=
        static_cast<int64_t>(std::ceil(command_deadband_counts_))) {
      changed = true;
    }
  }

  if (use_fake_hardware_) {
    last_command_counts_ = target_counts;
    command_initialized_ = true;
    return hardware_interface::return_type::OK;
  }

  if (!changed) {
    // Avoid repeated SDO writes when the controller keeps publishing the same target.
    return hardware_interface::return_type::OK;
  }

  std::ostringstream command;
  // The bridge command order is the same as the joint order parsed from the URDF.
  command << "MOVE_ALL";
  for (const int64_t target : target_counts) {
    command << " " << target;
  }

  std::string response;
  if (!send_command(command.str(), response) || response.rfind("OK", 0) != 0) {
    RCLCPP_ERROR(rclcpp::get_logger("RASCLHardwareInterface"),
                 "MOVE_ALL failed. command='%s' response='%s'", command.str().c_str(),
                 response.c_str());
    return hardware_interface::return_type::ERROR;
  }

  last_command_counts_ = target_counts;
  command_initialized_ = true;
  return hardware_interface::return_type::OK;
}

bool RASCLHardwareInterface::connect_to_bridge() {
  // Start from a clean descriptor so retries cannot reuse a half-open socket.
  close_socket();

  for (int attempt = 1; attempt <= connect_retries_; ++attempt) {
    socket_fd_ = ::socket(AF_INET, SOCK_STREAM, 0);
    if (socket_fd_ < 0) {
      RCLCPP_ERROR(rclcpp::get_logger("RASCLHardwareInterface"), "socket() failed: %s",
                   std::strerror(errno));
      return false;
    }

    sockaddr_in server_addr{};
    server_addr.sin_family = AF_INET;
    server_addr.sin_port = htons(static_cast<uint16_t>(port_));

    if (::inet_pton(AF_INET, host_.c_str(), &server_addr.sin_addr) <= 0) {
      RCLCPP_ERROR(rclcpp::get_logger("RASCLHardwareInterface"), "Invalid bridge host: %s",
                   host_.c_str());
      close_socket();
      return false;
    }

    // The launch file starts the Python bridge before ros2_control, but startup can
    // still take a moment; therefore connection attempts are retried.
    if (::connect(socket_fd_, reinterpret_cast<sockaddr*>(&server_addr), sizeof(server_addr)) ==
        0) {
      RCLCPP_INFO(rclcpp::get_logger("RASCLHardwareInterface"), "Connected to Faulhaber bridge.");
      return true;
    }

    close_socket();
    std::this_thread::sleep_for(std::chrono::duration<double>(connect_retry_delay_s_));
  }

  return false;
}

void RASCLHardwareInterface::close_socket() {
  if (socket_fd_ >= 0) {
    ::close(socket_fd_);
    socket_fd_ = -1;
  }
}

bool RASCLHardwareInterface::send_command(const std::string& command, std::string& response) {
  // The bridge protocol is request/response; serialize access to keep replies aligned.
  std::lock_guard<std::mutex> lock(socket_mutex_);

  if (socket_fd_ < 0 && !connect_to_bridge()) {
    return false;
  }

  if (!send_all(command + "\n")) {
    close_socket();
    return false;
  }

  if (!read_line(response)) {
    close_socket();
    return false;
  }

  return true;
}

bool RASCLHardwareInterface::send_all(const std::string& data) {
  const char* buffer = data.c_str();
  std::size_t remaining = data.size();

  while (remaining > 0) {
    // send() may write only part of the buffer, so keep going until it is complete.
    const ssize_t sent = ::send(socket_fd_, buffer, remaining, 0);
    if (sent <= 0) {
      return false;
    }
    buffer += sent;
    remaining -= static_cast<std::size_t>(sent);
  }

  return true;
}

bool RASCLHardwareInterface::read_line(std::string& line) {
  line.clear();
  char ch = 0;

  while (true) {
    const ssize_t received = ::recv(socket_fd_, &ch, 1, 0);
    if (received <= 0) {
      return false;
    }

    if (ch == '\n') {
      return true;
    }

    if (ch != '\r') {
      line.push_back(ch);
    }

    if (line.size() > 4096) {
      // Guard against malformed bridge responses without a newline terminator.
      return false;
    }
  }
}

int64_t RASCLHardwareInterface::radians_to_counts(std::size_t joint_index, double radians) const {
  // home_offset_counts defines the zero point; direction aligns the joint sign.
  const JointConfig& config = joint_configs_[joint_index];
  const double raw_counts = static_cast<double>(config.home_offset_counts) +
                            config.direction * radians * config.counts_per_rad;
  return static_cast<int64_t>(std::llround(raw_counts));
}

double RASCLHardwareInterface::counts_to_radians(std::size_t joint_index, int64_t counts) const {
  // This is the exact inverse convention of radians_to_counts().
  const JointConfig& config = joint_configs_[joint_index];
  return config.direction *
         (static_cast<double>(counts - config.home_offset_counts) / config.counts_per_rad);
}

double RASCLHardwareInterface::clamp_command(std::size_t joint_index, double command) const {
  // Joint limits are treated as a software safety layer before commanding drives.
  const JointConfig& config = joint_configs_[joint_index];
  return std::clamp(command, config.min_position, config.max_position);
}

}  // namespace rascl_hardware_interface

PLUGINLIB_EXPORT_CLASS(rascl_hardware_interface::RASCLHardwareInterface,
                       hardware_interface::SystemInterface)
