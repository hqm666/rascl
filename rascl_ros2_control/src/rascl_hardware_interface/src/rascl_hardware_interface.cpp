#include <limits>
#include <vector>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"

#include <cstdio>
#include <iostream>
#include <cmath>
#include "../include/rascl_hardware_interface/rascl_hardware_interface.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "soem/soem.h"


/*
https://docs.ros.org/en/jazzy/p/lifecycle/

    on_configure() (in the configuring state): Create and initialize the publisher and timer.

    on_activate() (in the activating state): Activate the publisher and timer so that publishing can occur in the active state.

    on_deactivate() (in the deactivating state): Stop the publisher.

    on_cleanup() (in the cleaningUp state): Release the pointers to publisher and timer.

    on_shutdown() (in the shuttingDown state): In this case does the same as on_cleanup(), in general it would for example release system resources.

*/

using namespace std;

namespace rascl_hardware_interface
{
   RASCL_HI::RASCL_HI() {}
   RASCL_HI::~RASCL_HI() = default;

  hardware_interface::CallbackReturn RASCL_HI::on_init(const hardware_interface::HardwareComponentInterfaceParams & info)
  {
    // ============================================================
    // STEP 1: Call base class on_init()
    // ============================================================
    if (hardware_interface::SystemInterface::on_init(info) != CallbackReturn::SUCCESS)
    {
      return CallbackReturn::ERROR;
    }

    RCLCPP_INFO(rclcpp::get_logger("RASCL_HI"), "Initializing RASCL Hardware Interface...");

    // ============================================================
    // STEP 2: Validate hardware info
    // ============================================================
    if (info_.hardware_parameters.empty()) {
      RCLCPP_ERROR(rclcpp::get_logger("RASCL_HI"), "No hardware parameters provided!");
      return hardware_interface::CallbackReturn::ERROR;
    }

    // ============================================================
    // STEP 3: Parse and store hardware parameters
    // ============================================================
    auto fake_hw_param = info_.hardware_parameters.find("use_fake_hardware");
    use_fake_hardware = (fake_hw_param != info_.hardware_parameters.end() && fake_hw_param->second == "true");
    RCLCPP_INFO(rclcpp::get_logger("RASCL_HI"), "(on_init) info_hardware_parameters.use_fake_hardware: %s", use_fake_hardware ? "true" : "false");

    // read ethernet adapter parameter
    auto ethercat_name_param = info_.hardware_parameters.find("adapter");
    if (ethercat_name_param != info_.hardware_parameters.end()) {
      ethercat_adapter_ = ethercat_name_param->second;
      RCLCPP_INFO(
      rclcpp::get_logger("RASCL_HI"),
      "Ethercat adapter: %s", ethercat_adapter_.c_str());
    } else {
      RCLCPP_ERROR(
      rclcpp:: get_logger("RASCL_HI"),
      "Required parameter 'adapter' not found!");
      return hardware_interface::CallbackReturn::ERROR;
    }


    // ============================================================
    // STEP 4: Validate joint configuration
    // ============================================================
    RCLCPP_INFO(rclcpp::get_logger("RASCL_HI"), "(on_init) info_.joints.size(): %zu", info_.joints.size());
    // check we have 4 manipulator joints
    if (info_.joints.size() != NUM_JOINTS) {
      RCLCPP_ERROR(rclcpp::get_logger("RASCL_HI"), "Expected %zu joints, but got %zu", NUM_JOINTS, info_.joints.size());
      return hardware_interface::CallbackReturn::ERROR;
    }
    
    // check for each joint, if the hwi configuration was correctly set in the urdf robot description file
    for (const auto& joint : info_.joints) {
      RCLCPP_INFO(
      rclcpp:: get_logger("RASCL_HI"),
      "Configuring joint: %s", joint. name.c_str());

      // joint should have exactly one command interface (position)
      if (joint.command_interfaces.size() != 1) {
        RCLCPP_ERROR(
        rclcpp::get_logger("RASCL_HI"),
        "Joint '%s' has %zu command interfaces.  Expected 1.",
        joint. name.c_str(), joint.command_interfaces.size());
        return hardware_interface::CallbackReturn::ERROR;
      }
      // command interface should be position
      if (joint.command_interfaces[0].name != hardware_interface::HW_IF_POSITION) {
        RCLCPP_ERROR(rclcpp::get_logger("RASCL_HI"), "Joint '%s' has command interface '%s'. Expected 'position'.",
        joint.name.c_str(), joint.command_interfaces[0]. name.c_str());
        return hardware_interface::CallbackReturn::ERROR;
      }
      // joint should have exactly one state interface
      if (joint.state_interfaces.size() != 1) {
        RCLCPP_ERROR(
        rclcpp::get_logger("RASCL_HI"),
        "Joint '%s' has %zu state interfaces. Expected 1.",
        joint.name.c_str(), joint.state_interfaces.size());
        return hardware_interface::CallbackReturn:: ERROR;
      }

      // state interface should be position
      if (joint.state_interfaces[0].name != hardware_interface::HW_IF_POSITION) {
        RCLCPP_ERROR(
        rclcpp::get_logger("RASCL_HI"),
        "Joint '%s' has state interface '%s'.  Expected 'position'.",
        joint.name.c_str(), joint.state_interfaces[0]. name.c_str());
        return hardware_interface::CallbackReturn::ERROR;
      }
    }
    
    // ============================================================
    // STEP 5: Initialize internal state/command storage
    // ============================================================
    const size_t num_joints = info_.joints.size();

    hw_states_.resize(num_joints, numeric_limits<double>::quiet_NaN());
    hw_commands_.resize(num_joints, numeric_limits<double>::quiet_NaN());

    hw_position_min_.resize(num_joints, numeric_limits<double>::quiet_NaN());
    hw_position_max_.resize(num_joints, numeric_limits<double>::quiet_NaN());
    hw_position_initial_.resize(num_joints, numeric_limits<double>::quiet_NaN());
    joint_names_.resize(num_joints);

    homing_positions.resize(num_joints, numeric_limits<int32_t>::quiet_NaN());
    
    for (size_t i = 0; i < num_joints; ++i) {
      const auto& joint = info_.joints[i];
      joint_names_[i] = joint.name;

      // Get command interface params
      const auto& cmd_interface = joint.command_interfaces[0];

      // parse min limit
      auto param_it = cmd_interface.parameters.find("min");
      if (param_it != cmd_interface.parameters.end()) {
        hw_position_min_[i] = stod(param_it->second);
      } else {
        RCLCPP_WARN(
        rclcpp::get_logger("RASCL_HI"),
        "Joint '%s':  'min' parameter not found. Using -inf.",
        joint.name.c_str());
        hw_position_min_[i] = -std::numeric_limits<double>:: infinity();
      }

      // parse max limit
      param_it = cmd_interface.parameters.find("max");
      if (param_it != cmd_interface.parameters.end())
      {
        hw_position_max_[i] = std::stod(param_it->second);
      } else {
        RCLCPP_WARN(
          rclcpp:: get_logger("RASCL_HI"),
          "Joint '%s': 'max' parameter not found. Using +inf.",
          joint.name.c_str());
        hw_position_max_[i] = std::numeric_limits<double>::infinity();
      }

      // Parse initial value
      param_it = cmd_interface.parameters.find("initial_value");
      if (param_it != cmd_interface.parameters. end())
      {
        hw_position_initial_[i] = std::stod(param_it->second);
      } else {
        RCLCPP_WARN(
          rclcpp::get_logger("RASCL_HI"),
          "Joint '%s': 'initial_value' parameter not found. Using 0.0.",
          joint.name. c_str());
        hw_position_initial_[i] = DEFAULT_INITIAL_POSITION;
      }
      // Validate that initial value is within limits
      if (hw_position_initial_[i] < hw_position_min_[i] ||
          hw_position_initial_[i] > hw_position_max_[i])
      {
        RCLCPP_ERROR(
          rclcpp::get_logger("RASCL_HI"),
          "Joint '%s': initial_value (%.3f) is outside limits [%.3f, %.3f]",
          joint.name.c_str(), hw_position_initial_[i],
          hw_position_min_[i], hw_position_max_[i]);
        return hardware_interface::CallbackReturn::ERROR;
      }

      // Initialize state and command to initial value
      hw_states_[i] = hw_position_initial_[i];
      hw_commands_[i] = hw_position_initial_[i];

      RCLCPP_INFO(
        rclcpp:: get_logger("RASCL_HI"),
        "  Joint '%s': min=%.3f, max=%.3f, initial=%.3f",
        joint.name.c_str(),
        hw_position_min_[i],
        hw_position_max_[i],
        hw_position_initial_[i]);
    }
    
    for (size_t j = 0; j < info_.joints.size(); j++) {
      hardware_interface::ComponentInfo joint = info_.joints.at(j);
      RCLCPP_INFO(rclcpp::get_logger("RASCL_HI"), "(on_init) joint %zu with name: %s", j, joint.name.c_str());
    }
    
    return CallbackReturn::SUCCESS;
  }

  hardware_interface::return_type RASCL_HI::prepare_command_mode_switch (const vector< string > &, const vector< string > &) {
    return hardware_interface::return_type::OK;
  }

  hardware_interface::return_type RASCL_HI::perform_command_mode_switch (const vector< string > &, const vector< string > &) {
    return hardware_interface::return_type::OK;
  }

  hardware_interface::CallbackReturn RASCL_HI::on_configure(const rclcpp_lifecycle::State & /*previous_state*/)
{
  if (!use_fake_hardware) {
    RCLCPP_INFO(rclcpp::get_logger("RASCL_HI"), "(on_configure) Using REAL hardware");

    // connect to Motion Controller ciA402 
    if (ecx_init(&ctx, ethercat_adapter_.c_str())) 
    {
        RCLCPP_INFO(rclcpp::get_logger("RASCL_HI"), "(on_configure) Init succeeded");

        /* find and auto-config slaves */
        if (ecx_config_init(&ctx) > 0)
        {
            ecx_config_map_group(&ctx, IOmap, 0);

            ecx_configdc(&ctx);

            RCLCPP_INFO(rclcpp::get_logger("RASCL_HI"), "(on_configure) %d slave(s) found and configured.", ctx.slavecount);

            /* wait for all slaves to reach SAFE_OP state */
            ecx_statecheck(&ctx, 0, EC_STATE_SAFE_OP, EC_TIMEOUTSTATE * 4);

            // sets actual slave state (if slave number = 0, then write to all slaves)
            ecx_writestate(&ctx, 1);

            // cout << "### Slave List ###" << endl;
            // for (uint16_t i=0; i < (ctx.slavecount+1); i++) {
            //   auto config_adress = ctx.slavelist[i].configadr;
            //   uint16_t slave_state = ctx.slavelist[i].state;
            //   auto slave_name = ctx.slavelist[i].name;

            //   printf("Slave:%d\n Name: %s\n configadr: %u\n Output size: %dbits\n Input size: %dbits\n State: %d\n Delay: %d[ns]\n Has DC: %d\n",
            //   i, ctx.slavelist[i].name, (unsigned int)config_adress, ctx.slavelist[i].Obits, ctx.slavelist[i].Ibits,
            //   ctx.slavelist[i].state, ctx.slavelist[i].pdelay, ctx.slavelist[i].hasdc);
            //   //cout << slave_name << " configadr: " << config_adress
            // }
            // cout << "### End of List ###" << endl;

            RCLCPP_INFO(rclcpp::get_logger("RASCL_HI"), "(on_configure) Init done");
        } 
        else {
            RCLCPP_INFO(rclcpp::get_logger("RASCL_HI"), "(on_configure) No slaves found");
            RCLCPP_ERROR(
            rclcpp::get_logger("RASCL_HI"),
            "No ethercat slaves were found. Check the connection and if they are powered on.");
            return hardware_interface::CallbackReturn::ERROR;
        }
    }
  } else {
    RCLCPP_INFO(rclcpp::get_logger("RASCL_HI"), "(on_configure) Using FAKE hardware");
  }


  return CallbackReturn::SUCCESS;
}

  vector<hardware_interface::StateInterface> RASCL_HI::export_state_interfaces()
{
  // creates handles (pointers) that allow controllers to READ the current hardware state
  vector<hardware_interface::StateInterface> state_interfaces;
  for (size_t i = 0; i < info_.joints.size(); ++i)
  {
    state_interfaces.emplace_back(hardware_interface::StateInterface(
      info_.joints[i].name, hardware_interface::HW_IF_POSITION, &hw_states_[i]));
  }

  return state_interfaces;
}

  vector<hardware_interface::CommandInterface> RASCL_HI::export_command_interfaces()
{
  // creates handles (pointers) that allow controllers to WRITE commands
  vector<hardware_interface::CommandInterface> command_interfaces;
  for (size_t i = 0; i < info_.joints.size(); ++i)
  {
    command_interfaces.emplace_back(hardware_interface::CommandInterface(
      info_.joints[i].name, hardware_interface::HW_IF_POSITION, &hw_commands_[i]));
  }

  return command_interfaces;
}

  hardware_interface::CallbackReturn RASCL_HI::on_activate(const rclcpp_lifecycle::State & /*previous_state*/)
{

  if (use_fake_hardware) {
    RCLCPP_INFO(rclcpp::get_logger("RASCL_HI"), "activating fake hardware...");
    return CallbackReturn::SUCCESS;
  }

  uint16_t status_word;
  int size = sizeof(status_word);

  for(uint8_t slave = 1; slave <= ctx.slavecount; slave++){
    RCLCPP_INFO(rclcpp::get_logger("RASCL_HI"), "(on_activate) Activate slave number: %s", to_string(slave).c_str());

    ecx_SDOread(&ctx, slave, 0x6041, 0x00, false, &size, &status_word, EC_TIMEOUTRXM);
    RCLCPP_DEBUG(rclcpp::get_logger("RASCL_HI"), "(on_activate) Status before: 0x%X", status_word);

    //Enable Voltage
    uint16_t enable_voltage = MOTOR_ENABLE_VOLTAGE;
    ecx_SDOwrite(&ctx, slave, 0x6040, 0x00, false, sizeof(enable_voltage), &enable_voltage, EC_TIMEOUTRXM);
    this_thread::sleep_for(chrono::milliseconds(delay));
    ecx_SDOread(&ctx, slave, 0x6041, 0x00, false, &size, &status_word, EC_TIMEOUTRXM);
    // Only log if not in Ready to Switch On or higher state
    if (!check_status_word(status_word, STATUS_READY_TO_SWITCH_ON_MASK, STATUS_READY_TO_SWITCH_ON_VALUE) &&
        !check_status_word(status_word, STATUS_SWITCHED_ON_MASK, STATUS_SWITCHED_ON_VALUE) &&
        !check_status_word(status_word, STATUS_OPERATION_ENABLED_MASK, STATUS_OPERATION_ENABLED_VALUE)) {
      RCLCPP_WARN(rclcpp::get_logger("RASCL_HI"), "(on_activate) Unexpected status after enabling voltage: 0x%X", status_word);
    }

    //Switch on
    uint16_t switch_on = MOTOR_SWITCH_ON;
    ecx_SDOwrite(&ctx, slave, 0x6040, 0x00, false, sizeof(switch_on), &switch_on, EC_TIMEOUTRXM);
    this_thread::sleep_for(chrono::milliseconds(delay));
    ecx_SDOread(&ctx, slave, 0x6041, 0x00, false, &size, &status_word, EC_TIMEOUTRXM);
    // Only log if not in Switched On or Operation Enabled state
    if (!check_status_word(status_word, STATUS_SWITCHED_ON_MASK, STATUS_SWITCHED_ON_VALUE) &&
        !check_status_word(status_word, STATUS_OPERATION_ENABLED_MASK, STATUS_OPERATION_ENABLED_VALUE)) {
      RCLCPP_WARN(rclcpp::get_logger("RASCL_HI"), "(on_activate) Unexpected status after switching on: 0x%X", status_word);
    }

    //Enable Operation
    uint16_t enable_operation = MOTOR_ENABLE_OPERATION;
    ecx_SDOwrite(&ctx, slave, 0x6040, 0x00, false, sizeof(enable_operation), &enable_operation, EC_TIMEOUTRXM);
    this_thread::sleep_for(chrono::milliseconds(delay));
    ecx_SDOread(&ctx, slave, 0x6041, 0x00, false, &size, &status_word, EC_TIMEOUTRXM);
    // Only log if not in Operation Enabled state
    if (!check_status_word(status_word, STATUS_OPERATION_ENABLED_MASK, STATUS_OPERATION_ENABLED_VALUE)) {
      RCLCPP_WARN(rclcpp::get_logger("RASCL_HI"), "(on_activate) Unexpected status after enabling operation: 0x%X", status_word);
    }

    //Mode of operation = Cyclic Synchronous Position mode -> 8 (prefered over profile pos. mode -> 1)
    uint8_t mode = MODE_CYCLIC_SYNC_POSITION;
    ecx_SDOwrite(&ctx, slave, 0x6060, 0x00, false, sizeof(mode), &mode, EC_TIMEOUTRXM);
    this_thread::sleep_for(chrono::milliseconds(delay));
  }
  
  // homing all joints
  uint8_t mode = HOMING_MODE_METHOD_26;
  homing_positions[UPPERARM_SLAVE_ID - 1] = Home(UPPERARM_SLAVE_ID, mode);
  homing_positions[SHOULDER_SLAVE_ID - 1] = Home(SHOULDER_SLAVE_ID, mode);
  homing_positions[LOWERARM_SLAVE_ID - 1] = Home(LOWERARM_SLAVE_ID, mode);

  mode = HOMING_MODE_METHOD_20;
  homing_positions[GRIPPER_SLAVE_ID - 1] = Home(GRIPPER_SLAVE_ID, mode);

  RCLCPP_INFO(rclcpp::get_logger("RASCL_HI"), "(on_activate) Homing sequence done, results:");
  for(size_t i = 0; i < NUM_JOINTS; i++) {
    RCLCPP_INFO(rclcpp::get_logger("RASCL_HI"), "(on_activate) Slave: %d: %d", static_cast<int>(i + 1), homing_positions[i]);
  }

  // Set mode of operation = Cyclic Synchronous Position mode -> 8 (prefered over profile pos. mode -> 1)
  mode = MODE_CYCLIC_SYNC_POSITION;
  for(uint8_t slave = 1; slave <= ctx.slavecount; slave++){
    ecx_SDOwrite(&ctx, slave, 0x6060, 0x00, false, sizeof(mode), &mode, EC_TIMEOUTRXM);
    this_thread::sleep_for(chrono::milliseconds(delay));
  }
  
  return CallbackReturn::SUCCESS;
}

  hardware_interface::CallbackReturn RASCL_HI::on_cleanup(const rclcpp_lifecycle::State & /*previous_state*/){
    RCLCPP_INFO(rclcpp::get_logger("RASCL_HI"), "(on_cleanup) on_cleanup called");
    return CallbackReturn::SUCCESS;
  }

  hardware_interface::CallbackReturn RASCL_HI::on_deactivate(const rclcpp_lifecycle::State & /*previous_state*/)
  {
    // TODO(anyone): prepare the robot to stop receiving commands

    // bring motion controller into state not receiving any commands -> check Statechart
    if (use_fake_hardware) {
      RCLCPP_INFO(rclcpp::get_logger("RASCL_HI"), "deactivating fake hardware...");
      return CallbackReturn::SUCCESS;
    }

    // uint16_t status_word;
    // int size = sizeof(status_word);

    // Reduce the time of homing motion
    SetPosition(1, -0.3);
    SetPosition(2, -0.5);
    SetPosition(3, -0.3);
    SetPosition(4, -0.3);


    // for (uint8_t slave = 1; slave <= hw_states_.size(); slave++) {
    // //uint8_t slave = 3;

    //   cout << "Deactivating slave: " << to_string(slave) << endl;

    //   //Disable Operation
    //   uint16_t disable_operation = 7;
    //   ecx_SDOwrite(&ctx, slave, 0x6040, 0x00, false, sizeof(disable_operation), &disable_operation, EC_TIMEOUTRXM);
    //   this_thread::sleep_for(chrono::milliseconds(delay));
    //   ecx_SDOread(&ctx, slave, 0x6041, 0x00, false, &size, &status_word, EC_TIMEOUTRXM);
    //   cout << "Status after disabling operation: 0x" << hex << uppercase << status_word << endl;

    //   //Disable Voltage
    //   uint16_t disable_voltage = 0;
    //   ecx_SDOwrite(&ctx, slave, 0x6040, 0x00, false, sizeof(disable_voltage), &disable_voltage, EC_TIMEOUTRXM);
    //   this_thread::sleep_for(chrono::milliseconds(delay));
    //   ecx_SDOread(&ctx, slave, 0x6041, 0x00, false, &size, &status_word, EC_TIMEOUTRXM);
    //   cout << "Status after disabling voltage: 0x" << hex << uppercase << status_word << endl;
    // }

    // close the ecx context gracefully
    ecx_statecheck(&ctx, 0, EC_STATE_INIT, EC_TIMEOUTSTATE * 4);

    ecx_close(&ctx);

    return CallbackReturn::SUCCESS;
  }

  hardware_interface::CallbackReturn RASCL_HI::on_shutdown(const rclcpp_lifecycle::State & previous_state){

  // https://github.com/ros-controls/ros2_control_demos/blob/c30d02557a036e087e66ff57a9943a65579012be/example_14/hardware/rrbot_actuator_without_feedback.cpp
  // implementation of on_cleanup was performed like this in demo 14
  return on_cleanup(previous_state);
}

  hardware_interface::return_type RASCL_HI::read(const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  if (use_fake_hardware) {
    const double fake_movement = FAKE_MOVEMENT_RATE;
    double act_pos = 0.0;
    double target_pos = 0.0;
    double new_pos = 0.0;
    // simulate a movement towards the goal position
    for (uint8_t slave = 1; slave <= hw_states_.size(); slave++) {
      act_pos = hw_states_[(slave - 1)];
      target_pos = hw_commands_[(slave - 1)];
      // move, if far away from goal state
      if (std::abs(act_pos - target_pos) > FAKE_POSITION_THRESHOLD) {
        new_pos = act_pos + ((act_pos - target_pos) * fake_movement);
        hw_states_[(slave - 1)] = new_pos;
      } else {
        // else target reached
        hw_states_[(slave-1)] = target_pos;
      }
    }
    return hardware_interface::return_type::OK;
  }

  // Use preallocated variables for real-time performance
  read_size_act_pos_ = sizeof(read_act_pos_);
  for (uint8_t slave = 1; slave <= ctx.slavecount; slave++) {
    // EC_TIMEOUTRXM = 700 000 nu s = 700 ms
    ecx_SDOread(&ctx, slave, 0x6064, 0x00, false, &read_size_act_pos_, &read_act_pos_, EC_TIMEOUTRXM);
    read_pos_to_zero_ = read_act_pos_;
    read_pos_to_zero_ -= homing_positions[slave - 1];
    hw_states_[slave - 1] = read_pos_to_zero_ * steps2rad_consts[slave - 1];
  }

  return hardware_interface::return_type::OK;
}

  hardware_interface::return_type RASCL_HI::write(const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  if (use_fake_hardware) {
    // do nothing (no real hardware to apply the command to)
    return hardware_interface::return_type::OK;
  }
  
  // Use preallocated variables for real-time performance
  for (uint8_t slave = 1; slave <= ctx.slavecount; slave++) {
    write_target_angle_ = hw_commands_[(slave-1)];
    write_min_ = hw_position_min_[slave-1];
    write_max_ = hw_position_max_[slave-1];
    
    // Clamp to limits (use DEBUG level to avoid real-time violations)
    if(write_target_angle_ < write_min_) {
      write_target_angle_ = write_min_;
      RCLCPP_DEBUG(rclcpp::get_logger("RASCL_HI"), "(write) target_angle set to min: %f", write_min_);
    } else if(write_target_angle_ > write_max_) {
      write_target_angle_ = write_max_;
      RCLCPP_DEBUG(rclcpp::get_logger("RASCL_HI"), "(write) target_angle set to max: %f", write_max_);
    }
    
    write_target_steps_ = (int32_t) round(write_target_angle_ * convert_consts[slave - 1]);
    write_target_steps_ += homing_positions[slave - 1];
    ecx_SDOwrite(&ctx, slave, 0x607A, 0x00, false, sizeof(write_target_steps_), &write_target_steps_, EC_TIMEOUTRXM);    
  }
  return hardware_interface::return_type::OK;
}


  int32_t RASCL_HI::Home(uint16_t slave, uint8_t home_mode) {
    uint16_t status_word;
    int size = sizeof(status_word);


    //Enable Voltage
    uint16_t enable_voltage = MOTOR_ENABLE_VOLTAGE;
    ecx_SDOwrite(&ctx, slave, 0x6040, 0x00, false, sizeof(enable_voltage), &enable_voltage, EC_TIMEOUTRXM);
    this_thread::sleep_for(chrono::milliseconds(delay));
    ecx_SDOread(&ctx, slave, 0x6041, 0x00, false, &size, &status_word, EC_TIMEOUTRXM);
    // Only log if not in expected state
    if (!check_status_word(status_word, STATUS_READY_TO_SWITCH_ON_MASK, STATUS_READY_TO_SWITCH_ON_VALUE) &&
        !check_status_word(status_word, STATUS_SWITCHED_ON_MASK, STATUS_SWITCHED_ON_VALUE) &&
        !check_status_word(status_word, STATUS_OPERATION_ENABLED_MASK, STATUS_OPERATION_ENABLED_VALUE)) {
      RCLCPP_WARN(rclcpp::get_logger("RASCL_HI"), "(Home) Unexpected status after enabling voltage: 0x%X", status_word);
    }

    //Switch on
    uint16_t switch_on = MOTOR_SWITCH_ON;
    ecx_SDOwrite(&ctx, slave, 0x6040, 0x00, false, sizeof(switch_on), &switch_on, EC_TIMEOUTRXM);
    this_thread::sleep_for(chrono::milliseconds(delay));
    ecx_SDOread(&ctx, slave, 0x6041, 0x00, false, &size, &status_word, EC_TIMEOUTRXM);
    // Only log if not in expected state
    if (!check_status_word(status_word, STATUS_SWITCHED_ON_MASK, STATUS_SWITCHED_ON_VALUE) &&
        !check_status_word(status_word, STATUS_OPERATION_ENABLED_MASK, STATUS_OPERATION_ENABLED_VALUE)) {
      RCLCPP_WARN(rclcpp::get_logger("RASCL_HI"), "(Home) Unexpected status after switching on: 0x%X", status_word);
    }

    //Enable Operation
    uint16_t enable_operation = MOTOR_ENABLE_OPERATION;
    ecx_SDOwrite(&ctx, slave, 0x6040, 0x00, false, sizeof(enable_operation), &enable_operation, EC_TIMEOUTRXM);
    this_thread::sleep_for(chrono::milliseconds(delay));
    ecx_SDOread(&ctx, slave, 0x6041, 0x00, false, &size, &status_word, EC_TIMEOUTRXM);
    // Only log if not in Operation Enabled state
    if (!check_status_word(status_word, STATUS_OPERATION_ENABLED_MASK, STATUS_OPERATION_ENABLED_VALUE)) {
      RCLCPP_WARN(rclcpp::get_logger("RASCL_HI"), "(Home) Unexpected status after enabling operation: 0x%X", status_word);
    }
    
    //Mode of operation = Homing Mode
    uint8_t homing_mode = MODE_HOMING;
    ecx_SDOwrite(&ctx, slave, 0x6060, 0x00, false, sizeof(homing_mode), &homing_mode, EC_TIMEOUTRXM);
    this_thread::sleep_for(chrono::milliseconds(delay));
    ecx_SDOread(&ctx, slave, 0x6041, 0x00, false, &size, &status_word, EC_TIMEOUTRXM);
    RCLCPP_DEBUG(rclcpp::get_logger("RASCL_HI"), "(Home) Status after setting Homing Mode: 0x%X", status_word);

    //Homing Method
    uint8_t homing_method = home_mode;
    ecx_SDOwrite(&ctx, slave, 0x6098, 0x00, false, sizeof(homing_method), &homing_method, EC_TIMEOUTRXM);
    this_thread::sleep_for(chrono::milliseconds(delay));

    //Homing Speed
    uint32_t homing_speed = HOMING_SPEED_DEFAULT;
    if(slave == GRIPPER_SLAVE_ID) homing_speed = HOMING_SPEED_GRIPPER;

    ecx_SDOwrite(&ctx, slave, 0x6099, 0x02, false, sizeof(homing_speed), &homing_speed, EC_TIMEOUTRXM);
    this_thread::sleep_for(chrono::milliseconds(delay));

    //Start Homing
    uint16_t start_homing = MOTOR_START_HOMING;
    ecx_SDOwrite(&ctx, slave, 0x6040, 0x00, false, sizeof(start_homing), &start_homing, EC_TIMEOUTRXM);
    this_thread::sleep_for(chrono::milliseconds(delay));

    // Wait for Home to be reached
    uint16_t mask = HOMING_STATUS_MASK;
    ecx_SDOread(&ctx, slave, 0x6041, 0x00, false, &size, &status_word, EC_TIMEOUTRXM);
    status_word = status_word & mask;
    while(status_word != mask)
    {
        ecx_SDOread(&ctx, slave, 0x6041, 0x00, false, &size, &status_word, EC_TIMEOUTRXM);
        status_word = status_word & mask;
        //cout << "Inside while-loop (Home). Waiting for: 0x" << hex << uppercase << mask << "\tReceiving: 0x" << status_word << endl;
        this_thread::sleep_for(chrono::milliseconds(delay));
    }

    RCLCPP_INFO(rclcpp::get_logger("RASCL_HI"), "(Home) Home reached");

    // Get actual position after homing
    int32_t act_pos = 0;
    int size_act_pos = sizeof(act_pos);
    ecx_SDOread(&ctx, slave, 0x6064, 0x00, false, &size_act_pos, &act_pos, EC_TIMEOUTRXM);
    RCLCPP_INFO(rclcpp::get_logger("RASCL_HI"), "(Home) Home position for slave %d: %d", slave, act_pos);

    return act_pos;
  }

// Drives the motor to a specified angle in rad
  void RASCL_HI::SetPosition(uint16_t slave, double angle){

    double target_angle = 0.0;
    int32_t target_steps = 0;
    double min, max;

    target_angle = angle;
    min = hw_position_min_[slave-1];
    max = hw_position_max_[slave-1];
    if(min <= target_angle && target_angle <= max) {
      RCLCPP_DEBUG(rclcpp::get_logger("RASCL_HI"), "(SetPosition) slave: %s target_angle %f within limits %f and %f", to_string(slave).c_str(), target_angle, min, max);
    } else if(target_angle < min) {
      target_angle = min;
      RCLCPP_WARN(rclcpp::get_logger("RASCL_HI"), "(SetPosition) slave: %s target_angle set to min: %f", to_string(slave).c_str(), min);
    } else if(target_angle > max) {
      target_angle = max;
      RCLCPP_WARN(rclcpp::get_logger("RASCL_HI"), "(SetPosition) slave: %s target_angle set to max: %f", to_string(slave).c_str(), max);
    }
    //target_steps = rad2steps(target_angle, slave - 1);
    target_steps = (int32_t) round(target_angle * convert_consts[slave - 1]);
    target_steps += homing_positions[slave - 1];
  
    int32_t act_pos = 0;
    int size_act_pos = sizeof(act_pos);
    ecx_SDOread(&ctx, slave, 0x6064, 0x00, false, &size_act_pos, &act_pos, EC_TIMEOUTRXM);
    RCLCPP_DEBUG(rclcpp::get_logger("RASCL_HI"), "(SetPosition) slave: %s target position: %d act position: %d", to_string(slave).c_str(), target_steps, act_pos);

    //Mode of operation = Cyclic Synchronous Position mode -> 8 (prefered over profile pos. mode -> 1)
    uint8_t mode = MODE_CYCLIC_SYNC_POSITION;
    ecx_SDOwrite(&ctx, slave, 0x6060, 0x00, false, sizeof(mode), &mode, EC_TIMEOUTRXM);
    this_thread::sleep_for(chrono::milliseconds(delay));

    //Maximum Motor Speed
    uint32_t max_speed = MAX_MOTOR_SPEED;
    ecx_SDOwrite(&ctx, slave, 0x6080, 0x00, false, sizeof(max_speed), &max_speed, EC_TIMEOUTRXM);
    this_thread::sleep_for(chrono::milliseconds(delay));

    //Write Target Position
    ecx_SDOwrite(&ctx, slave, 0x607A, 0x00, false, sizeof(target_steps), &target_steps, EC_TIMEOUTRXM);
    this_thread::sleep_for(chrono::milliseconds(delay));

    //Start Motion
    uint16_t start_motion = MOTOR_START_MOTION;
    ecx_SDOwrite(&ctx, slave, 0x6040, 0x00, false, sizeof(start_motion), &start_motion, EC_TIMEOUTRXM);
    this_thread::sleep_for(chrono::milliseconds(delay));

    // Wait for the target to be reached
    
    while(std::abs(act_pos - target_steps) > POSITION_TOLERANCE)
    {
        ecx_SDOread(&ctx, slave, 0x6064, 0x00, false, &size_act_pos, &act_pos, EC_TIMEOUTRXM);
        RCLCPP_DEBUG(rclcpp::get_logger("RASCL_HI"), "(SetPosition) slave: %s Inside while-loop (SetPosition). Waiting for: %d Receiving: %d", to_string(slave).c_str(), target_steps, act_pos);
        this_thread::sleep_for(chrono::milliseconds(DELAY_MULTIPLIER * delay));
    }
    RCLCPP_DEBUG(rclcpp::get_logger("RASCL_HI"), "(SetPosition) slave: %s exiting while-loop", to_string(slave).c_str());
}

}  // namespace rascl_hardware_interface

PLUGINLIB_EXPORT_CLASS(
  rascl_hardware_interface::RASCL_HI, hardware_interface::SystemInterface)
