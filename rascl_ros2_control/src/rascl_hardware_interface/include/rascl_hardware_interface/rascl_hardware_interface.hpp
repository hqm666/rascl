// Copyright (c) 2022, Stogl Robotics Consulting UG (haftungsbeschränkt) (template)
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef MY_RASCL__TEST_HWI_HPP_
#define MY_RASCL__TEST_HWI_HPP_

#include <string>
#include <vector>

#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp_lifecycle/state.hpp"

#include "soem/soem.h"

namespace rascl_hardware_interface
{
class RASCL_HI : public hardware_interface::SystemInterface
{
public:

 RASCL_HI();
virtual ~RASCL_HI();

  uint8_t IOmap[128];

  // The Ethernet context
  ecx_contextt ctx;

  // Amount of milliseconds to wait before executing the next command
  uint16_t delay = 100;
  std::vector<int32_t> homing_positions;

  hardware_interface::CallbackReturn on_init(
    const hardware_interface::HardwareComponentInterfaceParams & info) override;

  hardware_interface::CallbackReturn on_configure(
    const rclcpp_lifecycle::State & previous_state) override;

  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  hardware_interface::return_type prepare_command_mode_switch (const std::vector< std::string > &, const std::vector< std::string > &) override;

  hardware_interface::return_type perform_command_mode_switch (const std::vector< std::string > &, const std::vector< std::string > &) override;

  hardware_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::return_type read(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

  hardware_interface::return_type write(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

  hardware_interface::CallbackReturn on_cleanup(const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::CallbackReturn on_shutdown(const rclcpp_lifecycle::State & previous_state) override;

  // double deg2rad(double deg);

  // int32_t deg2steps(double angle);

  // double steps2rad(int32_t steps, uint8_t slave);

  // int32_t rad2steps(double radian, uint8_t slave);
  
  // Helper function to validate status word against expected value with mask
  inline bool check_status_word(uint16_t status_word, uint16_t expected_mask, uint16_t expected_value) const {
    return (status_word & expected_mask) == expected_value;
  }

private:
  // Hardware parameters
  bool use_fake_hardware;
  std::string ethercat_adapter_;

  // Joint data storage
  std::vector<double> hw_commands_;
  std::vector<double> hw_states_;

  // Joint Limits
  std::vector<double> hw_position_min_;
  std::vector<double> hw_position_max_;
  std::vector<double> hw_position_initial_;

  // Joint names for logging/debugging
  std::vector<std::string> joint_names_;


  //uint16_t delay = 10;
  int32_t Home(uint16_t slave, uint8_t home_mode);
  void SetPosition(uint16_t slave, double angle);

  // Constants
  static constexpr size_t NUM_JOINTS = 4;
  static constexpr double DEFAULT_INITIAL_POSITION = 0.0;
  
  // Motor control state values
  static constexpr uint16_t MOTOR_ENABLE_VOLTAGE = 6;
  static constexpr uint16_t MOTOR_SWITCH_ON = 7;
  static constexpr uint16_t MOTOR_ENABLE_OPERATION = 15;
  static constexpr uint16_t MOTOR_START_HOMING = 31;
  static constexpr uint16_t MOTOR_START_MOTION = 63;
  
  // Operation modes
  static constexpr uint8_t MODE_HOMING = 6;
  static constexpr uint8_t MODE_CYCLIC_SYNC_POSITION = 8;
  static constexpr uint8_t HOMING_MODE_METHOD_20 = 20;
  static constexpr uint8_t HOMING_MODE_METHOD_26 = 26;
  
  // Homing configuration
  static constexpr uint32_t HOMING_SPEED_DEFAULT = 100;
  static constexpr uint32_t HOMING_SPEED_GRIPPER = 25;
  static constexpr uint16_t HOMING_STATUS_MASK = 0x1400;
  
  // Motor joint slave IDs
  static constexpr uint8_t SHOULDER_SLAVE_ID = 1;
  static constexpr uint8_t UPPERARM_SLAVE_ID = 2;
  static constexpr uint8_t LOWERARM_SLAVE_ID = 3;
  static constexpr uint8_t GRIPPER_SLAVE_ID = 4;
  
  // Motion configuration
  static constexpr uint32_t MAX_MOTOR_SPEED = 800;
  static constexpr int32_t POSITION_TOLERANCE = 5;
  static constexpr int DELAY_MULTIPLIER = 10;
  
  // Fake hardware simulation
  static constexpr double FAKE_MOVEMENT_RATE = 0.005;
  static constexpr double FAKE_POSITION_THRESHOLD = 0.1;
  
  // CANopen 402 Status word bit masks for validation
  static constexpr uint16_t STATUS_OPERATION_ENABLED_MASK = 0x6F;  // bits 0,1,2,3,5,6
  static constexpr uint16_t STATUS_OPERATION_ENABLED_VALUE = 0x27; // Operation Enable state
  static constexpr uint16_t STATUS_SWITCHED_ON_MASK = 0x6F;
  static constexpr uint16_t STATUS_SWITCHED_ON_VALUE = 0x23;       // Switched On state
  static constexpr uint16_t STATUS_READY_TO_SWITCH_ON_MASK = 0x6F;
  static constexpr uint16_t STATUS_READY_TO_SWITCH_ON_VALUE = 0x21; // Ready to Switch On state
  static constexpr uint16_t STATUS_FAULT_MASK = 0x08;              // bit 3
  static constexpr uint16_t STATUS_FAULT_VALUE = 0x08;             // Fault bit set

  // Mathematical and conversion constants
  static constexpr double PI = 3.14159265358979323846;
  double convert_consts[4] = {
    523792.0,
    523792.0,
    523792.0,
    210880.3
  };
  double convert_const = 523792.0;
  double gripper_const = 210880.3;

  double steps2rad_consts[4] = {
    0.00000190915478,
    0.00000190915478,
    0.00000190915478,
    0.000004742
  };

  double steps2rad_const = 0.00000190915478;

  // Preallocated variables for real-time operations (read/write)
  // These are allocated once during initialization to avoid memory allocation during control loop
  // NOTE: These are safe to use as member variables because read() and write() are called
  // sequentially by the controller manager, never concurrently
  int32_t read_act_pos_;
  int32_t read_pos_to_zero_;
  int read_size_act_pos_;
  
  double write_target_angle_;
  int32_t write_target_steps_;
  double write_min_;
  double write_max_;
};

}  // namespace rascl_hardware_interface

#endif  // MY_RASCL__TEST_HWI_HPP_
