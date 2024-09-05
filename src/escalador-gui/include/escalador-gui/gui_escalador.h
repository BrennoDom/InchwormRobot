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
#include <sensor_msgs/msg/joy.hpp>

// Map PS5 dualsense buttons
#define BUTTON_CROSS        msg->buttons[0]
#define BUTTON_CIRCLE       msg->buttons[1]
#define BUTTON_TRIANGLE     msg->buttons[2]
#define BUTTON_SQUARE       msg->buttons[3]
#define BUTTON_L1           msg->buttons[4]
#define BUTTON_R1           msg->buttons[5]
#define BUTTON_L2           msg->buttons[6]
#define BUTTON_R2           msg->buttons[7]
#define AXIS_LEFT_X         msg->axes[0]
#define AXIS_LEFT_Y         msg->axes[1]
#define BUTTON_L2_TRIGGER   msg->axes[2]
#define AXIS_RIGHT_X        msg->axes[3]
#define AXIS_RIGHT_Y        msg->axes[4]
#define BUTTON_R2_TRIGGER   msg->axes[5]
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

private:
  Ui::GuiEscalador *ui;
  rclcpp::Executor::SharedPtr exec_;
  QTimer *ros_timer;


  //rclcpp::Subscription<std_msgs::msg::String>::SharedPtr chatter_sub_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr Joint_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr Joy_sub_;
  rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr EndEffec_sub_;
  //rclcpp::Publisher<std_msgs::msg::String>::SharedPtr  hello_pub_;
  //rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr  Joint_pub_;
  rclcpp::Publisher<std_msgs::msg::Int32MultiArray>::SharedPtr Vel_xe_;
};

#endif
