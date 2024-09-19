/*
# MIT License

# Copyright (c) 2022 Kristopher Krasnosky

# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:

# The above copyright notice and this permission notice shall be included in all
# copies or substantial portions of the Software.

# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
# SOFTWARE.
*/

#ifndef GUI_ESCALADOR_H
#define GUI_ESCALADOR_H

#include <QWidget>
#include <rclcpp/rclcpp.hpp>
#include <qtimer.h>
#include <std_msgs/msg/string.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/int32_multi_array.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <sensor_msgs/msg/joy.hpp>
#include <control_msgs/msg/interface_value.hpp>
#include <std_msgs/msg/int8.hpp>
#include <escalador_interfaces/srv/change_base.hpp>
#include <escalador_interfaces/msg/actual_base.hpp>
#include <escalador_interfaces/msg/base.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <trajectory_msgs/msg/joint_trajectory_point.hpp>
#include <std_msgs/msg/bool.hpp>

// Map PS5 dualsense buttons
#define BUTTON_CROSS        msg->buttons[0]
#define BUTTON_CIRCLE       msg->buttons[1]
#define BUTTON_TRIANGLE     msg->buttons[4]
#define BUTTON_SQUARE       msg->buttons[3]
#define BUTTON_L1           msg->buttons[4]
#define BUTTON_R1           msg->buttons[5]
#define BUTTON_L2           msg->buttons[6]
#define BUTTON_R2           msg->buttons[7]
#define AXIS_LEFT_X         msg->axes[0]
#define AXIS_LEFT_Y         msg->axes[1]
#define BUTTON_L2_TRIGGER   msg->axes[5]
#define AXIS_RIGHT_X        msg->axes[3]
#define AXIS_RIGHT_Y        msg->axes[2]
#define BUTTON_R2_TRIGGER   msg->axes[4]
#define BUTTON_LEFT_RIGHT   msg->axes[6]
#define BUTTON_UP_DOWN      msg->axes[7]

namespace Ui {
class GuiEscalador;
}

class GuiEscalador : public QWidget
{
  Q_OBJECT

public:
  explicit GuiEscalador(QWidget *parent = nullptr);
  ~GuiEscalador();
  void chatterCallback(const std_msgs::msg::String::SharedPtr msg);
  void JointsCallback(const sensor_msgs::msg::JointState::SharedPtr Joints);
  void EndEffectorCallback(const std_msgs::msg::Float32MultiArray::SharedPtr Param);
  void JoyCallback(const sensor_msgs::msg::Joy::SharedPtr msg);
  void StatusDynamixelCallback(const control_msgs::msg::InterfaceValue::SharedPtr msg);
  void ActualBaseCallback(const std_msgs::msg::Int8 msg);

  bool FlagInit = false, LastRef = false;
  double pos_BASE1,pos_BASE2;
  int ActualBase;
  int J1_TE,J2_TE,J3_TE,J4_TE,J5_TE,J6_TE,J7_TE,J8_TE;
  int J1_R,J2_R,J3_R,J4_R,J5_R,J6_R,J7_R,J8_R;
  int J1_Status,J2_Status,J3_Status,J4_Status,J5_Status,J6_Status,J7_Status,J8_Status;
  rclcpp::Node::SharedPtr node_;
public slots:
  void spinOnce();

private slots:



  void on_slider_x_sliderReleased();

  void on_slider_y_sliderReleased();

  void on_slider_z_sliderReleased();

  void on_slider_roll_sliderReleased();

  void on_slider_pitch_sliderReleased();

  void on_slider_yaw_sliderReleased();

  void on_tab_customContextMenuRequested(const QPoint &pos);



  void on_ButtonEN_J1_clicked();

  void on_ButtonEN_J2_clicked();

  void on_ButtonEN_J3_clicked();

  void on_ButtonEN_J4_clicked();

  void on_ButtonEN_J5_clicked();

  void on_ButtonEN_J6_clicked();

  void on_ButtonEN_J7_clicked();

  void on_ButtonEN_J8_clicked();

  void on_Enable_ALL_clicked();

  void on_pushButton_10_clicked();

  void on_Base_Change_clicked();

  void on_pushButton_9_clicked();

  void on_BASE_1_change_clicked();

  void on_BASE_2_change_clicked();

  void on_ChangeKin_clicked();

private:
  Ui::GuiEscalador *ui;
  rclcpp::Executor::SharedPtr exec_;
  QTimer *ros_timer;


  //rclcpp::Subscription<std_msgs::msg::String>::SharedPtr chatter_sub_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr Joint_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr Joy_sub_;
  rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr EndEffec_sub_;
  rclcpp::Subscription<control_msgs::msg::InterfaceValue>::SharedPtr Status_Dynamixel;
  rclcpp::Subscription<std_msgs::msg::Int8>::SharedPtr Callback_ACT_BASE;

  //rclcpp::Publisher<std_msgs::msg::String>::SharedPtr  hello_pub_;
  //rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr  Joint_pub_;
  rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr Base1Trajectory;
  rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr Base2Trajectory;
  rclcpp::Publisher<std_msgs::msg::Int32MultiArray>::SharedPtr Vel_xe_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr Dynamixel_commands;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr Reference_commands;
  rclcpp::Client<escalador_interfaces::srv::ChangeBase>::SharedPtr ClientServerBase;
};

#endif
