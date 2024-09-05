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


  // create subscriber
  node_->declare_parameter("joint_sub", "/joint_states");
  std::string joint_topic = node_->get_parameter("joint_sub").get_parameter_value().get<std::string>();
  Joint_sub_ = node_ -> create_subscription<sensor_msgs::msg::JointState>(
              joint_topic, 10, std::bind(&GuiEscalador::JointsCallback, this, _1));


  EndEffec_sub_ = node_ -> create_subscription<std_msgs::msg::Float32MultiArray>(
              "end_effector_pos",10, std::bind(&GuiEscalador::EndEffectorCallback, this, _1));

  Joy_sub_ = node_ -> create_subscription<sensor_msgs::msg::Joy>(
              "/joy", 10, std::bind(&GuiEscalador::JoyCallback, this, _1));

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
    }
    else{
        QApplication::quit();
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

    ui->label_j1->setText(QString::number((Joints->position[0]*57.295779513),'f',1));
    ui->label_j2->setText(QString::number((Joints->position[1]*57.295779513),'f',1));
    ui->label_j3->setText(QString::number((Joints->position[2]*57.295779513),'f',1));
    ui->label_j4->setText(QString::number((Joints->position[3]*57.295779513),'f',1));
    ui->label_j5->setText(QString::number((Joints->position[4]*57.295779513),'f',1));
    ui->label_j6->setText(QString::number((Joints->position[5]*57.295779513),'f',1));

}

void GuiEscalador::JoyCallback(const sensor_msgs::msg::Joy::SharedPtr msg){

    double offset = ((BUTTON_L2_TRIGGER - 1) - (BUTTON_R2_TRIGGER - 1)) * 50;

    ui->slider_x->setValue(offset * BUTTON_CROSS);
    ui->slider_y->setValue(offset * BUTTON_SQUARE);
    ui->slider_z->setValue(offset * BUTTON_CIRCLE);
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

