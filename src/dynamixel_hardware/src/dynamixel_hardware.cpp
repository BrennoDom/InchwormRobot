// Copyright 2020 Yutaka Kondo <yutaka.kondo@youtalk.jp>
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

#include "dynamixel_hardware/dynamixel_hardware.hpp"
#include <iostream>
#include <algorithm>
#include <array>
#include <limits>
#include <string>
#include <vector>

#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"

double normalize( const double value, const double start, const double end ) 
{
  const double width       = end - start   ;   
  const double offsetValue = value - start ;   

  return ( offsetValue - ( floor( offsetValue / width ) * width ) ) + start ;
}

double denormalize(const double normalizedValue, const double start, const double end, const double referencePoint) 
{
    const double width = end - start;

    // Find the closest multiple of the width that could have been subtracted
    double n = round((referencePoint - normalizedValue) / width);

    // Reconstruct the original value
    return normalizedValue + n * width;
}


namespace dynamixel_hardware
{
constexpr const char * kDynamixelHardware = "DynamixelHardware";
constexpr uint8_t kGoalPositionIndex = 0;
constexpr uint8_t kGoalVelocityIndex = 1;
constexpr uint8_t kPresentPositionVelocityCurrentIndex = 0;
constexpr const char * kStatusTorque = "Torque_Enable";
constexpr const char * kStatusHardware = "Hardware_Error_Status";
constexpr const char * kGoalPositionItem = "Goal_Position";
constexpr const char * kGoalVelocityItem = "Goal_Velocity";
constexpr const char * kMovingSpeedItem = "Moving_Speed";
constexpr const char * kPresentPositionItem = "Present_Position";
constexpr const char * kPresentVelocityItem = "Present_Velocity";
constexpr const char * kPresentSpeedItem = "Present_Speed";
constexpr const char * kPresentCurrentItem = "Present_Current";
constexpr const char * kPresentLoadItem = "Present_Load";
constexpr const char * const kExtraJointParameters[] = {
  "Profile_Velocity",
  "Profile_Acceleration",
  "Position_P_Gain",
  "Position_I_Gain",
  "Position_D_Gain",
  "Velocity_P_Gain",
  "Velocity_I_Gain",
  "Feedforward_2nd_Gain",
  "Feedforward_1st_Gain",
};

CallbackReturn DynamixelHardware::on_init(const hardware_interface::HardwareInfo & info)
{
  RCLCPP_DEBUG(rclcpp::get_logger(kDynamixelHardware), "configure");
  if (hardware_interface::SystemInterface::on_init(info) != CallbackReturn::SUCCESS)
  {
    return CallbackReturn::ERROR;
  }

  joints_.resize(info_.joints.size(), Joint());
  joint_ids_.resize(info_.joints.size(), 0);
  joint_offsets_.resize(info_.joints.size(),0);
  joint_p_encoder_.resize(info_.joints.size(),0);

  for (uint i = 0; i < info_.joints.size(); i++) {
    joint_ids_[i] = std::stoi(info_.joints[i].parameters.at("id"));
    joints_[i].state.position = std::numeric_limits<double>::quiet_NaN();
    joints_[i].state.velocity = std::numeric_limits<double>::quiet_NaN();
    joints_[i].state.effort = std::numeric_limits<double>::quiet_NaN();
    joints_[i].command.position = std::numeric_limits<double>::quiet_NaN();
    joints_[i].command.velocity = std::numeric_limits<double>::quiet_NaN();
    joints_[i].command.effort = std::numeric_limits<double>::quiet_NaN();
    joints_[i].prev_command.position = joints_[i].command.position;
    joints_[i].prev_command.velocity = joints_[i].command.velocity;
    joints_[i].prev_command.effort = joints_[i].command.effort;
    joint_offsets_[i] = std::stod(info_.joints[i].parameters.at("offset"));
    joint_p_encoder_[i] = std::numeric_limits<double>::quiet_NaN();
    RCLCPP_INFO(rclcpp::get_logger(kDynamixelHardware), "joint_id %d: %d", i, joint_ids_[i]);
  }


  CommandsIO = std::stoi((info_.hardware_parameters.at("IOutputs")));
  StatesIO = std::stoi((info_.hardware_parameters.at("IOInputs")));

  if (
    info_.hardware_parameters.find("use_dummy") != info_.hardware_parameters.end() &&
    info_.hardware_parameters.at("use_dummy") == "true") {
    use_dummy_ = true;
    RCLCPP_INFO(rclcpp::get_logger(kDynamixelHardware), "dummy mode");
    return CallbackReturn::SUCCESS;
  }

  auto usb_port = info_.hardware_parameters.at("usb_port");
  auto baud_rate = std::stoi(info_.hardware_parameters.at("baud_rate"));
  const char * log = nullptr;

  RCLCPP_INFO(rclcpp::get_logger(kDynamixelHardware), "usb_port: %s", usb_port.c_str());
  RCLCPP_INFO(rclcpp::get_logger(kDynamixelHardware), "baud_rate: %d", baud_rate);

  if (!dynamixel_workbench_.init(usb_port.c_str(), baud_rate, &log)) {
   // RCLCPP_FATAL(rclcpp::get_logger(kDynamixelHardware), "%s", log);
   // return CallbackReturn::ERROR;
  }


  
  for (uint i = 0; i < info_.joints.size(); ++i) {
    uint16_t model_number = 0;
    RCLCPP_INFO(rclcpp::get_logger(kDynamixelHardware), "Rebooting %dº Dynamixel",i);
    if (!dynamixel_workbench_.reboot(joint_ids_[i], &log)) {
        RCLCPP_FATAL(rclcpp::get_logger(kDynamixelHardware), "%s", log);
        return CallbackReturn::ERROR;
      }
  }

  for (uint i = 0; i < info_.joints.size(); ++i) {
    uint16_t model_number = 0;

    if (!dynamixel_workbench_.ping(joint_ids_[i], &model_number, &log)) {
      RCLCPP_FATAL(rclcpp::get_logger(kDynamixelHardware), "%s", log);
      return CallbackReturn::ERROR;
    }
  }

  set_control_mode(ControlMode::Position, true);
  set_joint_params();

  const ControlItem * goal_position =
    dynamixel_workbench_.getItemInfo(joint_ids_[0], kGoalPositionItem);
  if (goal_position == nullptr) {
    return CallbackReturn::ERROR;
  }

  const ControlItem * goal_velocity =
    dynamixel_workbench_.getItemInfo(joint_ids_[0], kGoalVelocityItem);
  if (goal_velocity == nullptr) {
    goal_velocity = dynamixel_workbench_.getItemInfo(joint_ids_[0], kMovingSpeedItem);
  }
  if (goal_velocity == nullptr) {
    return CallbackReturn::ERROR;
  }

  const ControlItem * present_position =
    dynamixel_workbench_.getItemInfo(joint_ids_[0], kPresentPositionItem);
  if (present_position == nullptr) {
    return CallbackReturn::ERROR;
  }

  const ControlItem * present_velocity =
    dynamixel_workbench_.getItemInfo(joint_ids_[0], kPresentVelocityItem);
  if (present_velocity == nullptr) {
    present_velocity = dynamixel_workbench_.getItemInfo(joint_ids_[0], kPresentSpeedItem);
  }
  if (present_velocity == nullptr) {
    return CallbackReturn::ERROR;
  }

  const ControlItem * present_current =
    dynamixel_workbench_.getItemInfo(joint_ids_[0], kPresentCurrentItem);
  if (present_current == nullptr) {
    present_current = dynamixel_workbench_.getItemInfo(joint_ids_[0], kPresentLoadItem);
  }
  if (present_current == nullptr) {
    return CallbackReturn::ERROR;
  }


 const ControlItem * status_torque =
    dynamixel_workbench_.getItemInfo(joint_ids_[0], kStatusTorque);
  if (goal_position == nullptr) {
    return CallbackReturn::ERROR;
  }
   const ControlItem * status_hardware =
    dynamixel_workbench_.getItemInfo(joint_ids_[0], kStatusHardware);
  if (goal_position == nullptr) {
    return CallbackReturn::ERROR;
  }
  control_items_[kGoalPositionItem] = goal_position;
  control_items_[kGoalVelocityItem] = goal_velocity;
  control_items_[kPresentPositionItem] = present_position;
  control_items_[kPresentVelocityItem] = present_velocity;
  control_items_[kPresentCurrentItem] = present_current;
  control_items_[kStatusTorque] = status_torque;
  control_items_[kStatusHardware] = status_hardware;


  if (!dynamixel_workbench_.addSyncWriteHandler(
        control_items_[kGoalPositionItem]->address, control_items_[kGoalPositionItem]->data_length,
        &log)) {
    RCLCPP_FATAL(rclcpp::get_logger(kDynamixelHardware), "%s", log);
    return CallbackReturn::ERROR;
  }

  if (!dynamixel_workbench_.addSyncWriteHandler(
        control_items_[kGoalVelocityItem]->address, control_items_[kGoalVelocityItem]->data_length,
        &log)) {
    RCLCPP_FATAL(rclcpp::get_logger(kDynamixelHardware), "%s", log);
    return CallbackReturn::ERROR;
  }

  uint16_t start_address = std::min(
    control_items_[kPresentPositionItem]->address, control_items_[kPresentCurrentItem]->address);
  uint16_t read_length = control_items_[kPresentPositionItem]->data_length +
                         control_items_[kPresentVelocityItem]->data_length +
                         control_items_[kPresentCurrentItem]->data_length + 2;
  if (!dynamixel_workbench_.addSyncReadHandler(start_address, read_length, &log)) {
    RCLCPP_FATAL(rclcpp::get_logger(kDynamixelHardware), "%s", log);
    return CallbackReturn::ERROR;
  }

  if (!dynamixel_workbench_.addSyncReadHandler(
        control_items_[kStatusTorque]->address, control_items_[kStatusTorque]->data_length + 2,
        &log)) {
    RCLCPP_FATAL(rclcpp::get_logger(kDynamixelHardware), "%s", log);
  return CallbackReturn::ERROR;
  }

  if (!dynamixel_workbench_.addSyncReadHandler(
        control_items_[kStatusHardware]->address, control_items_[kStatusHardware]->data_length + 2,
        &log)) {
    RCLCPP_FATAL(rclcpp::get_logger(kDynamixelHardware), "%s", log);
  return CallbackReturn::ERROR;
  }
  return CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> DynamixelHardware::export_state_interfaces()
{

  
  RCLCPP_DEBUG(rclcpp::get_logger(kDynamixelHardware), "export_state_interfaces");
  std::vector<hardware_interface::StateInterface> state_interfaces;
  for (uint i = 0; i < info_.joints.size(); i++) {
    state_interfaces.emplace_back(hardware_interface::StateInterface(
      info_.joints[i].name, hardware_interface::HW_IF_POSITION, &joints_[i].state.position));
    state_interfaces.emplace_back(hardware_interface::StateInterface(
      info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &joints_[i].state.velocity));
    state_interfaces.emplace_back(hardware_interface::StateInterface(
      info_.joints[i].name, hardware_interface::HW_IF_EFFORT, &joints_[i].state.effort));
  }

  Inputs.resize(StatesIO);
  size_t ct = 0;
  for (size_t i = 0; i < info_.gpios.size(); i++)
  {
    for (auto state_if : info_.gpios.at(i).state_interfaces)
    {
      state_interfaces.emplace_back(hardware_interface::StateInterface(
        info_.gpios.at(i).name, state_if.name, &Inputs[ct++]));
    }
  }

  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> DynamixelHardware::export_command_interfaces()
{
  RCLCPP_DEBUG(rclcpp::get_logger(kDynamixelHardware), "export_command_interfaces");
  std::vector<hardware_interface::CommandInterface> command_interfaces;
  for (uint i = 0; i < info_.joints.size(); i++) {
    command_interfaces.emplace_back(hardware_interface::CommandInterface(
      info_.joints[i].name, hardware_interface::HW_IF_POSITION, &joints_[i].command.position));
    command_interfaces.emplace_back(hardware_interface::CommandInterface(
      info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &joints_[i].command.velocity));
  }

  Outputs.resize(CommandsIO);
  size_t ct = 0;
  for (size_t i = 0; i < info_.gpios.size(); i++)
  {
    for (auto command_if : info_.gpios.at(i).command_interfaces)
    {
      command_interfaces.emplace_back(hardware_interface::CommandInterface(
        info_.gpios.at(i).name, command_if.name, &Outputs[ct++]));
    }
  }


  return command_interfaces;
}

CallbackReturn DynamixelHardware::on_activate(const rclcpp_lifecycle::State & /* previous_state */)
{
  RCLCPP_DEBUG(rclcpp::get_logger(kDynamixelHardware), "start");
  for (uint i = 0; i < joints_.size(); i++) {
    if (use_dummy_ && std::isnan(joints_[i].state.position)) {
      joints_[i].state.position = 0.0;
      joints_[i].state.velocity = 0.0;
      joints_[i].state.effort = 0.0;
    }
  }

  
  read(rclcpp::Time{}, rclcpp::Duration(0, 0));
  reset_command();
  write(rclcpp::Time{}, rclcpp::Duration(0, 0));

  return CallbackReturn::SUCCESS;
}

CallbackReturn DynamixelHardware::on_deactivate(const rclcpp_lifecycle::State & /* previous_state */)
{
  RCLCPP_DEBUG(rclcpp::get_logger(kDynamixelHardware), "stop");
  return CallbackReturn::SUCCESS;
}

return_type DynamixelHardware::read(const rclcpp::Time & /* time */, const rclcpp::Duration & /* period */)
{
  if (use_dummy_) {
    for (uint i = 0; i < Outputs.size(); i++)
    {
      Inputs[i] = Outputs[i];
    }

    for (uint i = Outputs.size(); i < Inputs.size(); i++)
    {
      unsigned int seed = time(NULL) + 1;
      Inputs[i] = static_cast<float>(rand_r(&seed));
      seed = time(NULL) + 2;
    }
    // random inputs

   

    for (uint i = 0; i < Inputs.size(); i++)
    {
      RCLCPP_INFO(
        rclcpp::get_logger("RRBotSystemWithGPIOHardware"), "Read %.1f from GP input %d!",
        Inputs[i], i);
    }
    return return_type::OK;
  }

  std::vector<uint8_t> ids(info_.joints.size(), 0);
  std::vector<int32_t> positions(info_.joints.size(), 0);
  std::vector<int32_t> velocities(info_.joints.size(), 0);
  std::vector<int32_t> currents(info_.joints.size(), 0);
  std::vector<int32_t> sts_enabled(info_.joints.size(), 0);
  std::vector<int32_t> sts_hw(info_.joints.size(), 0);

  std::copy(joint_ids_.begin(), joint_ids_.end(), ids.begin());
  const char * log = nullptr;

  if (!dynamixel_workbench_.syncRead(
      kPresentPositionVelocityCurrentIndex, ids.data(), ids.size(), &log))
  {
    RCLCPP_ERROR(rclcpp::get_logger(kDynamixelHardware), "%s", log);
  }

  if (!dynamixel_workbench_.getSyncReadData(
      kPresentPositionVelocityCurrentIndex, ids.data(), ids.size(),
      control_items_[kPresentCurrentItem]->address,
      control_items_[kPresentCurrentItem]->data_length, currents.data(), &log))
  {
    RCLCPP_ERROR(rclcpp::get_logger(kDynamixelHardware), "%s", log);
  }

  if (!dynamixel_workbench_.getSyncReadData(
      kPresentPositionVelocityCurrentIndex, ids.data(), ids.size(),
      control_items_[kPresentVelocityItem]->address,
      control_items_[kPresentVelocityItem]->data_length, velocities.data(), &log))
  {
    RCLCPP_ERROR(rclcpp::get_logger(kDynamixelHardware), "%s", log);
  }

  if (!dynamixel_workbench_.getSyncReadData(
      kPresentPositionVelocityCurrentIndex, ids.data(), ids.size(),
      control_items_[kPresentPositionItem]->address,
      control_items_[kPresentPositionItem]->data_length, positions.data(), &log))
  {
    RCLCPP_ERROR(rclcpp::get_logger(kDynamixelHardware), "%s", log);
  }

  if (!dynamixel_workbench_.syncRead(
      1, ids.data(), ids.size(), &log))
  {
    RCLCPP_ERROR(rclcpp::get_logger(kDynamixelHardware), "%s", log);
  }

  if (!dynamixel_workbench_.getSyncReadData(
      1, ids.data(), ids.size(),
      control_items_[kStatusTorque]->address,
      control_items_[kStatusTorque]->data_length, sts_enabled.data(), &log))
  {
    RCLCPP_ERROR(rclcpp::get_logger(kDynamixelHardware), "%s", log);
  }

  if (!dynamixel_workbench_.syncRead(
      2, ids.data(), ids.size(), &log))
  {
    RCLCPP_ERROR(rclcpp::get_logger(kDynamixelHardware), "%s", log);
  }

  if (!dynamixel_workbench_.getSyncReadData(
      2, ids.data(), ids.size(),
      control_items_[kStatusHardware]->address,
      control_items_[kStatusHardware]->data_length, sts_hw.data(), &log))
  {
    RCLCPP_ERROR(rclcpp::get_logger(kDynamixelHardware), "%s", log);
  }
  

//
//  if (!dynamixel_workbench_.getSyncReadData(
//        kPresentPositionVelocityCurrentIndex, ids.data(), ids.size(),
//        control_items_[kStatusHardware]->address,
//        control_items_[kStatusHardware]->data_length, sts_hw.data(), &log)) {
//    RCLCPP_ERROR(rclcpp::get_logger(kDynamixelHardware), "%s", log);
//  }

  for (uint i = 0; i < ids.size(); i++) {
    
    joint_p_encoder_[i] = (dynamixel_workbench_.convertValue2Radian(ids[i], positions[i]) + joint_offsets_[i]);
    joints_[i].state.position =  normalize(joint_p_encoder_[i],-3.14,3.14);
    //joints_[i].state.position =  denormalize(joints_[i].state.position,-3.14,3.14,joint_p_encoder_[i]);
    //joints_[i].state.position = joint_p_encoder_[i];
    joints_[i].state.velocity = dynamixel_workbench_.convertValue2Velocity(ids[i], velocities[i]);
    joints_[i].state.effort = dynamixel_workbench_.convertValue2Current(currents[i]);

    Inputs[i] = sts_enabled[i];
    Inputs[i+8] = Outputs[i+8];
    Inputs[i+16] = sts_hw[i];
    
  }






  return return_type::OK;
}

return_type DynamixelHardware::write(const rclcpp::Time & /* time */, const rclcpp::Duration & /* period */)
{
  if (use_dummy_) {
    for (auto & joint : joints_) {
      joint.prev_command.position = joint.command.position;
      joint.state.position = joint.command.position;
    }
    return return_type::OK;
  }

  // Velocity control
  if (std::any_of(
        joints_.cbegin(), joints_.cend(), [](auto j) { return j.command.velocity != j.prev_command.velocity; })) {
    set_control_mode(ControlMode::Velocity);
    if(mode_changed_)
    {
      set_joint_params();
    }
    set_joint_velocities();
    return return_type::OK;
  }
  
  // Position control
  if (std::any_of(
        joints_.cbegin(), joints_.cend(), [](auto j) { return j.command.position != j.prev_command.position; })) {
    set_control_mode(ControlMode::Position);
    if(mode_changed_)
    {
      set_joint_params();
    }
    set_joint_positions();
    return return_type::OK;
  }

  // Effort control
  if (std::any_of(
               joints_.cbegin(), joints_.cend(), [](auto j) { return j.command.effort != 0.0; })) {
    RCLCPP_ERROR(rclcpp::get_logger(kDynamixelHardware), "Effort control is not implemented");
    return return_type::ERROR;
  }

  const char * log = nullptr;
  std::vector<double> OutputOld;
  OutputOld.resize(Outputs.size());
  
  for (uint i = 0; i < info_.joints.size(); ++i)
  {



    if (Outputs[i] == 1.0 && Inputs[i]!= 1.0){
      reset_command();
      Outputs[i+8] = 0.0;
      if (!dynamixel_workbench_.torqueOn(joint_ids_[i], &log)) {
        RCLCPP_FATAL(rclcpp::get_logger(kDynamixelHardware), "%s", log);
        return return_type::ERROR;
      }
    }
    else if(Outputs[i] == 0.0 && Inputs[i]!= 0.0){
      reset_command();
      if (!dynamixel_workbench_.torqueOff(joint_ids_[i], &log)) {
        RCLCPP_FATAL(rclcpp::get_logger(kDynamixelHardware), "%s", log);
        return return_type::ERROR;
      }
    }
    else if(Outputs[i+8] == 1.0 && Inputs[i]!= 1.0){
      reset_command();
      Outputs[i] = 0.0;
      if (!dynamixel_workbench_.reboot(joint_ids_[i], &log)) {
        RCLCPP_FATAL(rclcpp::get_logger(kDynamixelHardware), "%s", log);
        return return_type::ERROR;
      }else{Outputs[i+8] == 0.0;}
    }


  }
  // if all command values are unchanged, then remain in existing control mode and set corresponding command values
  switch (control_mode_) {
    case ControlMode::Velocity:
      set_joint_velocities();
      return return_type::OK;
      break;
    case ControlMode::Position:
      set_joint_positions();
      return return_type::OK;
      break;
    default: // effort, etc
      RCLCPP_ERROR(rclcpp::get_logger(kDynamixelHardware), "Control mode not implemented");
      return return_type::ERROR;
      break;
  }

}

return_type DynamixelHardware::enable_torque(const bool enabled)
{
  const char * log = nullptr;

  if (enabled && !torque_enabled_) {
    for (uint i = 0; i < info_.joints.size(); ++i) {
      if (!dynamixel_workbench_.torqueOff(joint_ids_[i], &log)) {
        RCLCPP_FATAL(rclcpp::get_logger(kDynamixelHardware), "%s", log);
        return return_type::ERROR;
      }
    }
    reset_command();
    RCLCPP_INFO(rclcpp::get_logger(kDynamixelHardware), "Torque enabled");
  } else if (!enabled && torque_enabled_) {
    for (uint i = 0; i < info_.joints.size(); ++i) {
      if (!dynamixel_workbench_.torqueOff(joint_ids_[i], &log)) {
        RCLCPP_FATAL(rclcpp::get_logger(kDynamixelHardware), "%s", log);
        return return_type::ERROR;
      }
    }
    RCLCPP_INFO(rclcpp::get_logger(kDynamixelHardware), "Torque disabled");
  }

  torque_enabled_ = enabled;
  return return_type::OK;
}

return_type DynamixelHardware::set_control_mode(const ControlMode & mode, const bool force_set)
{
  const char * log = nullptr;
  mode_changed_ = false;

  if (mode == ControlMode::Velocity && (force_set || control_mode_ != ControlMode::Velocity)) {
    bool torque_enabled = torque_enabled_;
    if (torque_enabled) {
      enable_torque(true);
    }

    for (uint i = 0; i < joint_ids_.size(); ++i) {
      if (!dynamixel_workbench_.setVelocityControlMode(joint_ids_[i], &log)) {
        RCLCPP_FATAL(rclcpp::get_logger(kDynamixelHardware), "%s", log);
        return return_type::ERROR;
      }
    }
    RCLCPP_INFO(rclcpp::get_logger(kDynamixelHardware), "Velocity control");
    if(control_mode_ != ControlMode::Velocity)
    {
      mode_changed_ = true;
      control_mode_ = ControlMode::Velocity;
    }

    if (torque_enabled) {
      enable_torque(true);
    }
    return return_type::OK;
  }

  if (mode == ControlMode::Position && (force_set || control_mode_ != ControlMode::Position)) {
    bool torque_enabled = torque_enabled_;
    if (torque_enabled) {
      enable_torque(true);
    }

    for (uint i = 0; i < joint_ids_.size(); ++i) {
      if (!dynamixel_workbench_.setExtendedPositionControlMode(joint_ids_[i], &log)) {
        RCLCPP_FATAL(rclcpp::get_logger(kDynamixelHardware), "%s", log);
        return return_type::ERROR;
      }
    }
    RCLCPP_INFO(rclcpp::get_logger(kDynamixelHardware), "Position control");
    if(control_mode_ != ControlMode::Position)
    {
      mode_changed_ = true;
      control_mode_ = ControlMode::Position;
    }

    if (torque_enabled) {
      enable_torque(true);
    }
    return return_type::OK;
  }
  
  if (control_mode_ != ControlMode::Velocity && control_mode_ != ControlMode::Position) {
    RCLCPP_FATAL(
      rclcpp::get_logger(kDynamixelHardware), "Only position/velocity control are implemented");
    return return_type::ERROR;
  }

  return return_type::OK;
}

return_type DynamixelHardware::reset_command()
{
  for (uint i = 0; i < joints_.size(); i++) {
    joints_[i].command.position = joints_[i].state.position;
    joints_[i].command.velocity = 0.0;
    joints_[i].command.effort = 0.0;
    joints_[i].prev_command.position = joints_[i].command.position;
    joints_[i].prev_command.velocity = joints_[i].command.velocity;
    joints_[i].prev_command.effort = joints_[i].command.effort;
  }

  return return_type::OK;
}

CallbackReturn DynamixelHardware::set_joint_positions()
{
  const char * log = nullptr;
  std::vector<int32_t> commands(info_.joints.size(), 0);
  std::vector<uint8_t> ids(info_.joints.size(), 0);

  std::copy(joint_ids_.begin(), joint_ids_.end(), ids.begin());
  for (uint i = 0; i < ids.size(); i++) {
    joints_[i].prev_command.position = joints_[i].command.position;
    
    commands[i] = dynamixel_workbench_.convertRadian2Value(
      ids[i], static_cast<float>(((denormalize(joints_[i].command.position,-3.14,3.14,joint_p_encoder_[i])) - joint_offsets_[i] )));
    
  }
  if (!dynamixel_workbench_.syncWrite(
        kGoalPositionIndex, ids.data(), ids.size(), commands.data(), 1, &log)) {
    RCLCPP_ERROR(rclcpp::get_logger(kDynamixelHardware), "%s", log);
  }
  return CallbackReturn::SUCCESS;
}

CallbackReturn DynamixelHardware::set_joint_velocities()
{
  const char * log = nullptr;
  std::vector<int32_t> commands(info_.joints.size(), 0);
  std::vector<uint8_t> ids(info_.joints.size(), 0);

  std::copy(joint_ids_.begin(), joint_ids_.end(), ids.begin());
  for (uint i = 0; i < ids.size(); i++) {
    joints_[i].prev_command.velocity = joints_[i].command.velocity;
    commands[i] = dynamixel_workbench_.convertVelocity2Value(
      ids[i], static_cast<float>(joints_[i].command.velocity));
  }
  if (!dynamixel_workbench_.syncWrite(
        kGoalVelocityIndex, ids.data(), ids.size(), commands.data(), 1, &log)) {
    RCLCPP_ERROR(rclcpp::get_logger(kDynamixelHardware), "%s", log);
    
  }
  return CallbackReturn::SUCCESS;
}

CallbackReturn DynamixelHardware::set_joint_params()
{
  const char * log = nullptr;
  for (uint i = 0; i < info_.joints.size(); ++i) {
    for (auto paramName : kExtraJointParameters) {
      if (info_.joints[i].parameters.find(paramName) != info_.joints[i].parameters.end()) {
        auto value = std::stoi(info_.joints[i].parameters.at(paramName));
        if (!dynamixel_workbench_.itemWrite(joint_ids_[i], paramName, value, &log)) {
          RCLCPP_FATAL(rclcpp::get_logger(kDynamixelHardware), "%s", log);
          return CallbackReturn::ERROR;
        }
        RCLCPP_INFO(rclcpp::get_logger(kDynamixelHardware), "%s set to %d for joint %d", paramName, value, i);
      }
    }
  }
  return CallbackReturn::SUCCESS;
}

}  // namespace dynamixel_hardware

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(dynamixel_hardware::DynamixelHardware, hardware_interface::SystemInterface)