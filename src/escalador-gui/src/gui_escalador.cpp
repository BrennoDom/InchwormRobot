#include "gui_escalador.h"
#include "ui_gui_escalador.h"
#include <QApplication>
#include <memory>
#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "std_msgs/msg/int32_multi_array.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"
#include <control_msgs/msg/interface_value.hpp>




using namespace std::chrono_literals;

/* This example creates a subclass of Node and uses std::bind() to register a
* member function as a callback from the timer. */

GuiEscalador::GuiEscalador(QWidget *parent) :
    QWidget(parent),
    ui(new Ui::GuiEscalador)
{
   ui-> setupUi(this);
   printf("ok");

  using std::placeholders::_1;

  // instantiate the node
  node_ = std::make_shared<rclcpp::Node>("gui_escalador");

  // create publisher
  //node_->declare_parameter("chatter_topic", "~/chatter");
  //std::string listen_topic = node_->get_parameter("chatter_topic").get_parameter_value().get<std::string>();
//  hello_pub_ = node_->create_publisher<std_msgs::msg::String>(listen_topic, 10);

  //publisher of vel end effector

  node_ -> declare_parameter("vel_end", "/set_ve");
  std::string topic_vel_ee = node_->get_parameter("vel_end").get_parameter_value().get<std::string>();
  Vel_xe_ = node_->create_publisher<std_msgs::msg::Int32MultiArray>(topic_vel_ee, 10);
  node_ -> declare_parameter("dynamixel_command", "/gpio_controller/commands");
  std::string topic_dynamixel_command = node_->get_parameter("dynamixel_command").get_parameter_value().get<std::string>();
  Dynamixel_commands = node_ -> create_publisher<std_msgs::msg::Float64MultiArray>(topic_dynamixel_command,10);
  node_ -> declare_parameter("SrvBase", "/SrvChangeBase");
  std::string SrvChangeBase = node_->get_parameter("SrvBase").get_parameter_value().get<std::string>();
  ClientServerBase = node_ -> create_client<escalador_interfaces::srv::ChangeBase>(SrvChangeBase);
  node_ -> declare_parameter("BASE1Points","BASE1_CONTROLLER/joint_trajectory");
  std::string BASE1_Traj = node_->get_parameter("BASE1Points").get_parameter_value().get<std::string>();
  Base1Trajectory = node_->create_publisher<trajectory_msgs::msg::JointTrajectory>(BASE1_Traj, 10);
  node_ -> declare_parameter("BASE2Points","BASE2_CONTROLLER/joint_trajectory");
  std::string BASE2_Traj = node_->get_parameter("BASE2Points").get_parameter_value().get<std::string>();
  Base2Trajectory = node_->create_publisher<trajectory_msgs::msg::JointTrajectory>(BASE2_Traj, 10);
  node_ -> declare_parameter("RefKin","/act_ref");
  std::string topicChangRef = node_->get_parameter("RefKin").get_parameter_value().get<std::string>();
  Reference_commands = node_->create_publisher<std_msgs::msg::Bool>(topicChangRef,10);

  // create subscriber
  node_->declare_parameter("joint_sub", "/joint_states");
  std::string joint_topic = node_->get_parameter("joint_sub").get_parameter_value().get<std::string>();
  Joint_sub_ = node_ -> create_subscription<sensor_msgs::msg::JointState>(
              joint_topic, 10, std::bind(&GuiEscalador::JointsCallback, this, _1));


  EndEffec_sub_ = node_ -> create_subscription<std_msgs::msg::Float32MultiArray>(
              "end_effector_pos",10, std::bind(&GuiEscalador::EndEffectorCallback, this, _1));

  Joy_sub_ = node_ -> create_subscription<sensor_msgs::msg::Joy>(
              "/joy", 10, std::bind(&GuiEscalador::JoyCallback, this, _1));

  Status_Dynamixel = node_ -> create_subscription<control_msgs::msg::InterfaceValue>(
              "/gpio_controller/inputs", 10, std::bind(&GuiEscalador::StatusDynamixelCallback, this, _1));

  Callback_ACT_BASE = node_ -> create_subscription<std_msgs::msg::Int8>(
              "/act_base", 10, std::bind(&GuiEscalador::ActualBaseCallback, this, _1));


  //node_->declare_parameter("hello_topic", "~/chatter");
  //std::string hello_topic = node_->get_parameter("hello_topic").get_parameter_value().get<std::string>();
  //chatter_sub_ = node_->create_subscription<std_msgs::msg::String>(
   //     hello_topic, 1, std::bind(&GuiEscalador::chatterCallback, this, _1));

  // setup the timer that will signal ros stuff to happen
  ros_timer = new QTimer(this);
  connect(ros_timer, SIGNAL(timeout()), this, SLOT(spinOnce()));
  ros_timer->start(10);  // set the rate to 10ms  You can change this if you want to increase/decrease update rate

  // tell the window to display itself
  this->show();
    
    
    
  
}

GuiEscalador::~GuiEscalador()
{
  delete ui;
  delete ros_timer;

}
void GuiEscalador::spinOnce(){
    if(rclcpp::ok()){


        rclcpp::spin_some(node_);
        std_msgs::msg::Int32MultiArray vels_ve;
        vels_ve.data = {ui->slider_x->value(),ui->slider_y->value(),ui->slider_z->value(),ui->slider_roll->value(),ui->slider_pitch->value(),ui->slider_yaw->value()};
        //ss << "hello world ";//<< ui->hi_num->value();
        //msg.data = "hello world ";

        Vel_xe_->publish(vels_ve);

        //ui->hi_num->setValue(ui->hi_num->value()+1);
        switch (J1_TE){
            case 0:
                ui->ButtonEN_J1->setText("Enable");
                ui->ButtonEN_J1->setChecked(false);

                if (J1_Status > 0){
                   ui->STATUS_J1->setStyleSheet("QLabel{ background-color : red; color : white; }");
                }
                else{
                   ui->STATUS_J1->setStyleSheet("");
                }
                break;
            case 1:
                   ui->ButtonEN_J1->setChecked(true);
                   ui->ButtonEN_J1->setText("Disable");
                   ui->STATUS_J1->setStyleSheet("QLabel{ background-color : green; color : white; }");
                break;
        }
        switch (J2_TE){
            case 0:
                ui->ButtonEN_J2->setText("Enable");
                ui->ButtonEN_J2->setChecked(false);
                if (J2_Status > 0){
                   ui->STATUS_J2->setStyleSheet("QLabel{ background-color : red; color : white; }");
                }
                else{
                   ui->STATUS_J2->setStyleSheet("");
                }
                break;
            case 1:
                   ui->ButtonEN_J2->setText("Disable");
                   ui->ButtonEN_J2->setChecked(true);
                   ui->STATUS_J2->setStyleSheet("QLabel{ background-color : green; color : white; }");
                break;
        }
        switch (J3_TE){
            case 0:
                ui->ButtonEN_J3->setText("Enable");
                ui->ButtonEN_J3->setChecked(false);
                if (J3_Status > 0){
                   ui->STATUS_J3->setStyleSheet("QLabel{ background-color : red; color : white; }");
                }
                else{
                   ui->STATUS_J3->setStyleSheet("");
                }
                break;
            case 1:
                   ui->ButtonEN_J3->setText("Disable");
                   ui->ButtonEN_J3->setChecked(true);
                   ui->STATUS_J3->setStyleSheet("QLabel{ background-color : green; color : white; }");
                break;
        }
        switch (J4_TE){
            case 0:
                ui->ButtonEN_J4->setText("Enable");
                ui->ButtonEN_J4->setChecked(false);
                if (J4_Status > 0){
                   ui->STATUS_J4->setStyleSheet("QLabel{ background-color : red; color : white; }");
                }
                else{
                   ui->STATUS_J4->setStyleSheet("");
                }
                break;
            case 1:
                   ui->ButtonEN_J4->setText("Disable");
                   ui->ButtonEN_J4->setChecked(true);
                   ui->STATUS_J4->setStyleSheet("QLabel{ background-color : green; color : white; }");
                break;
        }
        switch (J5_TE){
            case 0:
                ui->ButtonEN_J5->setText("Enable");
                ui->ButtonEN_J5->setChecked(false);
                if (J5_Status > 0){
                   ui->STATUS_J5->setStyleSheet("QLabel{ background-color : red; color : white; }");
                }
                else{
                   ui->STATUS_J5->setStyleSheet("");
                }
                break;
            case 1:
                   ui->ButtonEN_J5->setText("Disable");
                   ui->ButtonEN_J5->setChecked(true);
                   ui->STATUS_J5->setStyleSheet("QLabel{ background-color : green; color : white; }");
                break;
        }
        switch (J6_TE){
            case 0:
                ui->ButtonEN_J6->setText("Enable");
                ui->ButtonEN_J6->setChecked(false);
                if (J6_Status > 0){
                   ui->STATUS_J6->setStyleSheet("QLabel{ background-color : red; color : white; }");
                }
                else{
                   ui->STATUS_J6->setStyleSheet("");
                }
                break;
            case 1:
                   ui->ButtonEN_J6->setText("Disable");
                   ui->ButtonEN_J6->setChecked(true);
                   ui->STATUS_J6->setStyleSheet("QLabel{ background-color : green; color : white; }");
                break;
        }
        switch (J7_TE){
            case 0:
                ui->ButtonEN_J7->setText("Enable");
                ui->ButtonEN_J7->setChecked(false);
                if (J7_Status > 0){
                   ui->STATUS_BASE1->setStyleSheet("QLabel{ background-color : red; color : white; }");
                }
                else{
                   ui->STATUS_BASE1->setStyleSheet("");
                }
                break;
            case 1:
                   ui->ButtonEN_J7->setText("Disable");
                   ui->ButtonEN_J7->setChecked(true);
                   ui->STATUS_BASE1->setStyleSheet("QLabel{ background-color : green; color : white; }");
                break;
        }
        switch (J8_TE){
            case 0:
                ui->ButtonEN_J8->setText("Enable");
                ui->ButtonEN_J8->setChecked(false);
                if (J8_Status > 0){
                   ui->STATUS_BASE2->setStyleSheet("QLabel{ background-color : red; color : white; }");
                }
                else{
                   ui->STATUS_BASE2->setStyleSheet("");
                }
                break;
            case 1:
                   ui->ButtonEN_J8->setText("Disable");
                   ui->ButtonEN_J8->setChecked(true);
                   ui->STATUS_BASE2->setStyleSheet("QLabel{ background-color : green; color : white; }");
                break;
        }












    }
    else{
        QApplication::quit();
    }
}
void GuiEscalador::StatusDynamixelCallback(const control_msgs::msg::InterfaceValue::SharedPtr msg){
//

    J1_TE       = msg->values[0];
    J2_TE       = msg->values[1];
    J3_TE       = msg->values[2];
    J4_TE       = msg->values[3];
    J5_TE       = msg->values[4];
    J6_TE       = msg->values[5];
    J7_TE       = msg->values[6];
    J8_TE       = msg->values[7];
    J1_R        = msg->values[8];
    J2_R        = msg->values[9];
    J3_R        = msg->values[10];
    J4_R        = msg->values[11];
    J5_R        = msg->values[12];
    J6_R        = msg->values[13];
    J7_R        = msg->values[14];
    J8_R        = msg->values[15];
    J1_Status   = msg->values[16]*1000;
    J2_Status   = msg->values[17]*1000;
    J3_Status   = msg->values[18]*1000;
    J4_Status   = msg->values[19]*1000;
    J5_Status   = msg->values[20]*1000;
    J6_Status   = msg->values[21]*1000;
    J7_Status   = msg->values[22]*1000;
    J8_Status   = msg->values[23]*1000;
}

void GuiEscalador::ActualBaseCallback(const std_msgs::msg::Int8 msg){
    ActualBase = msg.data;

    if (ActualBase == 0 && LastRef==false){
        ui->label_kin_ref->setText("Normal");
    }
    else if (ActualBase == 0 && LastRef==true) {
        ui->label_kin_ref->setText("Inverted");
    }
    else if (ActualBase == 1 && LastRef==true) {
        ui->label_kin_ref->setText("Normal");
    }
    else if (ActualBase == 1 && LastRef==false) {
        ui->label_kin_ref->setText("Inverted");
    }
    else{ ui->label_kin_ref->setText("NONE");}

    if (ActualBase == 0){
      ui->label_act_base->setText("Base 0");
    }else{
      ui->label_act_base->setText("Base 1");
    }
}
void GuiEscalador::chatterCallback(const std_msgs::msg::String::SharedPtr msg){
  auto qstring_msg = QString::fromStdString( msg->data.c_str() );

  //ui->chatter->setText(qstring_msg);
}
void GuiEscalador::EndEffectorCallback(const std_msgs::msg::Float32MultiArray::SharedPtr Param){
    ui->label_x->setText(QString::number(Param->data[0] * 1000,'f',1));
    ui->label_y->setText(QString::number(Param->data[1] * 1000,'f',1));
    ui->label_z->setText(QString::number(Param->data[2] * 1000,'f',1));
    ui->label_roll->setText(QString::number(Param->data[3] * 57.295779513,'f',1));
    ui->label_pitch->setText(QString::number(Param->data[4] * 57.295779513,'f',1));
    ui->label_yaw->setText(QString::number(Param->data[5] * 57.295779513,'f',1));
}
void GuiEscalador::JointsCallback(const sensor_msgs::msg::JointState::SharedPtr Joints){




    ui->slider_j1->setValue((Joints->position[0]*57.295779513));
    ui->slider_j2->setValue((Joints->position[1]*57.295779513));
    ui->slider_j3->setValue((Joints->position[2]*57.295779513));
    ui->slider_j4->setValue((Joints->position[3]*57.295779513));
    ui->slider_j5->setValue((Joints->position[4]*57.295779513));
    ui->slider_j6->setValue((Joints->position[5]*57.295779513));
    ui->slider_BASE1->setValue((Joints->position[6]*57.295779513));
    ui->slider_BASE2->setValue((Joints->position[7]*57.295779513));
    pos_BASE1 = Joints->position[6];
    pos_BASE2 = Joints->position[7];
    ui->label_j1->setText(QString::number((Joints->position[0]*57.295779513),'f',1));
    ui->label_j2->setText(QString::number((Joints->position[1]*57.295779513),'f',1));
    ui->label_j3->setText(QString::number((Joints->position[2]*57.295779513),'f',1));
    ui->label_j4->setText(QString::number((Joints->position[3]*57.295779513),'f',1));
    ui->label_j5->setText(QString::number((Joints->position[4]*57.295779513),'f',1));
    ui->label_j6->setText(QString::number((Joints->position[5]*57.295779513),'f',1));
    ui->label_BASE1->setText(QString::number((Joints->position[6]*57.295779513),'f',1));
    ui->label_BASE2->setText(QString::number((Joints->position[7]*57.295779513),'f',1));

}

void GuiEscalador::JoyCallback(const sensor_msgs::msg::Joy::SharedPtr msg){

    double offset = ((BUTTON_L2_TRIGGER) - (BUTTON_R2_TRIGGER)) * 50;
    
    ui->slider_x->setValue(offset * BUTTON_SQUARE);
    ui->slider_y->setValue(offset * BUTTON_TRIANGLE);
    ui->slider_z->setValue(offset * BUTTON_CROSS);
    ui->slider_roll->setValue(offset * ((BUTTON_UP_DOWN == -1) ? 1 : 0));
    ui->slider_pitch->setValue(offset * ((BUTTON_LEFT_RIGHT == -1) ? 1 : 0));
    ui->slider_yaw->setValue(offset * ((BUTTON_LEFT_RIGHT == 1) ? 1 : 0));

}
int main(int argc, char * argv[])
{  
  rclcpp::init(argc, argv);
  QApplication app(argc, argv);
  auto w   = std::make_shared<GuiEscalador>();
  app.exec();

  return 0;
}





void GuiEscalador::on_slider_x_sliderReleased()
{
    ui -> slider_x->setValue(0);
}
void GuiEscalador::on_slider_y_sliderReleased()
{
    ui -> slider_y->setValue(0);
}



void GuiEscalador::on_slider_z_sliderReleased()
{
    ui -> slider_z->setValue(0);
}


void GuiEscalador::on_slider_roll_sliderReleased()
{
    ui -> slider_roll->setValue(0);
}


void GuiEscalador::on_slider_pitch_sliderReleased()
{
    ui -> slider_pitch->setValue(0);
}


void GuiEscalador::on_slider_yaw_sliderReleased()
{
    ui -> slider_yaw->setValue(0);
}




void GuiEscalador::on_ButtonEN_J1_clicked()
{
    std_msgs::msg::Float64MultiArray commands_dynamixel;
    float command;
    switch (J1_TE){
        case 0:
            command = 1.0;
            break;
        case 1:
            command = 0.0;
            break;
    }
    commands_dynamixel.data = {command,double(J2_TE),double(J3_TE),double(J4_TE),double(J5_TE),double(J6_TE),double(J7_TE),double(J8_TE),0,0,0,0,0,0,0,0};
    Dynamixel_commands -> publish(commands_dynamixel);
}


void GuiEscalador::on_ButtonEN_J2_clicked()
{
    std_msgs::msg::Float64MultiArray commands_dynamixel;
    float command;
    switch (J2_TE){
        case 0:
            command = 1.0;
            break;
        case 1:
            command = 0.0;
            break;
    }
    commands_dynamixel.data = {double(J1_TE),command,double(J3_TE),double(J4_TE),double(J5_TE),double(J6_TE),double(J7_TE),double(J8_TE),0,0,0,0,0,0,0,0};
    Dynamixel_commands -> publish(commands_dynamixel);
}


void GuiEscalador::on_ButtonEN_J3_clicked()
{
    std_msgs::msg::Float64MultiArray commands_dynamixel;
    float command;
    switch (J3_TE){
        case 0:
            command = 1.0;
            break;
        case 1:
            command = 0.0;
            break;
    }
    commands_dynamixel.data = {double(J1_TE),double(J2_TE),command,double(J4_TE),double(J5_TE),double(J6_TE),double(J7_TE),double(J8_TE),0,0,0,0,0,0,0,0};
    Dynamixel_commands -> publish(commands_dynamixel);
}


void GuiEscalador::on_ButtonEN_J4_clicked()
{
    std_msgs::msg::Float64MultiArray commands_dynamixel;
    float command;
    switch (J4_TE){
        case 0:
            command = 1.0;
            break;
        case 1:
            command = 0.0;
            break;
    }
    commands_dynamixel.data = {double(J1_TE),double(J2_TE),double(J3_TE),command,double(J5_TE),double(J6_TE),double(J7_TE),double(J8_TE),0,0,0,0,0,0,0,0};
    Dynamixel_commands -> publish(commands_dynamixel);
}


void GuiEscalador::on_ButtonEN_J5_clicked()
{
    std_msgs::msg::Float64MultiArray commands_dynamixel;
    float command;
    switch (J5_TE){
        case 0:
            command = 1.0;
            break;
        case 1:
            command = 0.0;
            break;
    }
    commands_dynamixel.data = {double(J1_TE),double(J2_TE),double(J3_TE),double(J4_TE),command,double(J6_TE),double(J7_TE),double(J8_TE),0,0,0,0,0,0,0,0};
    Dynamixel_commands -> publish(commands_dynamixel);
}


void GuiEscalador::on_ButtonEN_J6_clicked()
{
    std_msgs::msg::Float64MultiArray commands_dynamixel;
    float command;
    switch (J6_TE){
        case 0:
            command = 1.0;
            break;
        case 1:
            command = 0.0;
            break;
    }
    commands_dynamixel.data = {double(J1_TE),double(J2_TE),double(J3_TE),double(J4_TE),double(J5_TE),command,double(J7_TE),double(J8_TE),0,0,0,0,0,0,0,0};
    Dynamixel_commands -> publish(commands_dynamixel);
}


void GuiEscalador::on_ButtonEN_J7_clicked()
{
    std_msgs::msg::Float64MultiArray commands_dynamixel;
    float command;
    switch (J7_TE){
        case 0:
            command = 1.0;
            break;
        case 1:
            command = 0.0;
            break;
    }
    commands_dynamixel.data = {double(J1_TE),double(J2_TE),double(J3_TE),double(J4_TE),double(J5_TE),double(J6_TE),command,double(J8_TE),0,0,0,0,0,0,0,0};
    Dynamixel_commands -> publish(commands_dynamixel);
}


void GuiEscalador::on_ButtonEN_J8_clicked()
{
    std_msgs::msg::Float64MultiArray commands_dynamixel;
    float command;
    switch (J8_TE){
        case 0:
            command = 1.0;
            break;
        case 1:
            command = 0.0;
            break;
    }
    commands_dynamixel.data = {double(J1_TE),double(J2_TE),double(J3_TE),double(J4_TE),double(J5_TE),double(J6_TE),double(J7_TE),command,0,0,0,0,0,0,0,0};
    Dynamixel_commands -> publish(commands_dynamixel);
}


void GuiEscalador::on_Enable_ALL_clicked()
{
    std_msgs::msg::Float64MultiArray commands_dynamixel;
    float command;
    if (J1_TE==1 && J2_TE==1 && J3_TE==1 && J4_TE==1 && J5_TE==1 && J6_TE==1 && J7_TE==1 && J8_TE==1){
        command = 0.0;
    }else{
        command = 1.0;
    }
    commands_dynamixel.data = {command,command,command,command,command,command,command,command,0,0,0,0,0,0,0,0};
    Dynamixel_commands -> publish(commands_dynamixel);
}


void GuiEscalador::on_pushButton_10_clicked()
{
    std_msgs::msg::Float64MultiArray commands_dynamixel;
    float RB_1,RB_2,RB_3,RB_4,RB_5,RB_6,RB_7,RB_8;
    ((J1_Status > 0) ? RB_1=1.0 : RB_1=0.0);
    ((J2_Status > 0) ? RB_2=1.0 : RB_2=0.0);
    ((J3_Status > 0) ? RB_3=1.0 : RB_3=0.0);
    ((J4_Status > 0) ? RB_4=1.0 : RB_4=0.0);
    ((J5_Status > 0) ? RB_5=1.0 : RB_5=0.0);
    ((J6_Status > 0) ? RB_6=1.0 : RB_6=0.0);
    ((J7_Status > 0) ? RB_7=1.0 : RB_7=0.0);
    ((J8_Status > 0) ? RB_8=1.0 : RB_8=0.0);
    commands_dynamixel.data = {double(J1_TE),double(J2_TE),double(J3_TE),double(J4_TE),double(J5_TE),double(J6_TE),double(J7_TE),double(J8_TE),RB_1,RB_2,RB_3,RB_4,RB_5,RB_6,RB_7,RB_8};
    Dynamixel_commands -> publish(commands_dynamixel);
}


void GuiEscalador::on_Base_Change_clicked()
{
    auto command=false;
    if (ActualBase == 0){
        command = true;
    }else{
        command = false;
    }

    auto request = std::make_shared<escalador_interfaces::srv::ChangeBase::Request>();
      request->change = command;

    auto result = ClientServerBase->async_send_request(request);
      // Wait for the result.
      if (rclcpp::spin_until_future_complete(node_, result) ==
        rclcpp::FutureReturnCode::SUCCESS)
      {
      if(command == false){
          ui->label_act_base->setText("Base 0");
          LastRef = false;
      }else{
          ui->label_act_base->setText("Base 1");
          LastRef = true;
      }
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"),"Success!");

      } else {
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Failed to call service add_two_ints");
      }

}


void GuiEscalador::on_BASE_1_change_clicked()
{
    double command=1.57;
    if (pos_BASE1>0.2){
        command = 0.0 ;
    }else{
        command = 1.57;
    }
    trajectory_msgs::msg::JointTrajectory commands_joints;
    trajectory_msgs::msg::JointTrajectoryPoint commands_points;
    commands_points.time_from_start.sec = 2;
    commands_points.positions = {command};
    commands_joints.joint_names = {"BASE1"};
    commands_joints.points = {commands_points};

    Base1Trajectory -> publish(commands_joints);
}


void GuiEscalador::on_BASE_2_change_clicked()
{
    double command=1.57;
    if (pos_BASE2>0.2){
        command = 0.0 ;
    }else{
        command = 1.57;
    }
    trajectory_msgs::msg::JointTrajectory commands_joints;
    trajectory_msgs::msg::JointTrajectoryPoint commands_points;
    commands_points.time_from_start.sec = 2;
    commands_points.positions = {command};
    commands_joints.joint_names = {"BASE2"};
    commands_joints.points = {commands_points};

    Base2Trajectory -> publish(commands_joints);
}


void GuiEscalador::on_ChangeKin_clicked()
{
    std_msgs::msg::Bool commandKin;
    if (LastRef==false){
        LastRef = true;
    }else{
        LastRef = false;
    }
    commandKin.data = LastRef;
    Reference_commands ->publish(commandKin);
}
void GuiEscalador::on_pushButton_9_clicked(){}
void GuiEscalador::on_tab_customContextMenuRequested(QPoint const&){};
