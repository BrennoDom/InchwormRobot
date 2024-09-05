#include <chrono>
#include <functional>
#include <memory>
#include <sstream>
#include <string>

#include <algorithm>

#include "eigen3/Eigen/Core"
#include "eigen3/Eigen/SVD"

#include <kdl/rigidbodyinertia.hpp>
#include <kdl/chain.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/chainjnttojacsolver.hpp>
#include <kdl/chainiksolverpos_lma.hpp>
#include <kdl/tree.hpp>
#include <kdl/chaindynparam.hpp>

#include <kdl_parser/kdl_parser.hpp>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <trajectory_msgs/msg/joint_trajectory_point.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>

#include <std_msgs/msg/float64.hpp>
#include <std_msgs/msg/int8.hpp>
#include <std_msgs/msg/float64.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/int32_multi_array.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <std_msgs/msg/header.hpp>
#include <sensor_msgs/msg/joint_state.hpp>


const double PI = 3.14159;
bool flag = false;

using namespace std::chrono_literals;

//Funcao de calculo da pseudo inversa

template <class MatT>
Eigen::Matrix<typename MatT::Scalar, MatT::ColsAtCompileTime, MatT::RowsAtCompileTime> 
pinv(const MatT &mat,
     typename MatT::Scalar lambda = typename MatT::Scalar{2e-1}) // choose appropriately
{
  typedef typename MatT::Scalar Scalar;
  auto svd = mat.jacobiSvd(Eigen::ComputeFullU | Eigen::ComputeFullV);
  const auto &singularValues = svd.singularValues();
  Eigen::Matrix<Scalar, MatT::ColsAtCompileTime, MatT::RowsAtCompileTime> dampedSingularValuesInv(
      mat.cols(), mat.rows());
  dampedSingularValuesInv.setZero();
  std::for_each(singularValues.data(), singularValues.data() + singularValues.size(),
                [&, i = 0](const Scalar &s) mutable {
                  dampedSingularValuesInv(i, i) = s / (s * s + lambda * lambda);
                  ++i;
                });
  return svd.matrixV() * dampedSingularValuesInv * svd.matrixU().adjoint();
}




class IKSolver: public rclcpp::Node
{
	using joint_vector_t = Eigen::Vector<double, 6>;
 	using cartesian_vector_t = Eigen::Vector<double, 6>;
	public:
	
		IKSolver();
		void publisher();
	private:
		
		Eigen::Matrix<double,6,1> dxe_;
		Eigen::Matrix<double,6,1> xe_;
		Eigen::Matrix<double,6,1> xd_;
		Eigen::Matrix<double,6,1> actual_dxe_;
		Eigen::Matrix<double,2,1> base_q_;
		
		bool flagOK;
		void robotBaseCallback(const std_msgs::msg::Int8::SharedPtr msg);
		void robotJointCallback(const sensor_msgs::msg::JointState::SharedPtr Joints);
		void VelsCallback(const std_msgs::msg::Int32MultiArray::SharedPtr Vel);
\

		double x, y, z, roll, pitch, yaw;
		rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr robotEndPub;
    
		rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr robotActualJoint;
		rclcpp::Subscription<std_msgs::msg::Int8>::SharedPtr robotActualRobot;
		rclcpp::Subscription<std_msgs::msg::Int32MultiArray>::SharedPtr robotvelSub;
		rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr robotvelPub;
		rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr robotposPub;
		rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr JTrajectory;
		
		KDL::Frame FKkin, FKNext, FKAct;
		KDL::Chain chain_;
		KDL::JntArray q_, q_nxt;

		joint_vector_t dq_,dq_ant;
		
		
		std::unique_ptr<KDL::ChainJntToJacSolver> jacobian_solver_;
		std::unique_ptr<KDL::ChainFkSolverPos_recursive> foward_solver_;
  		KDL::Jacobian jacobian_;
		int actualBase_, oldBase;
		bool initialcondOK = false;
		bool commandDrives = false;
		std::string robotDescription_1;
		std::string robotDescription_2;
		std::vector<double,std::allocator<double>> act_joints={0.0,0.0,0.0,0.0,0.0,0.0};
		std::vector<double,std::allocator<double>> act_velJoints={0.0,0.0,0.0,0.0,0.0,0.0};
		void robotDescriptionCB1(const std_msgs::msg::String::SharedPtr robotDescription);
		void robotDescriptionCB2(const std_msgs::msg::String::SharedPtr robotDescription);
		rclcpp::Subscription<std_msgs::msg::String>::SharedPtr robotDescriptionSubscriber_1;
		rclcpp::Subscription<std_msgs::msg::String>::SharedPtr robotDescriptionSubscriber_2;
		std::string linkSource;
		std::string linkEnd;
		rclcpp::TimerBase::SharedPtr timer_;
		rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr robotJointPub;

		trajectory_msgs::msg::JointTrajectoryPoint trajPosJoint;
		trajectory_msgs::msg::JointTrajectory Postrajectory;


		
		//
		
		
    
		
};

IKSolver::IKSolver(): Node("ik_kinematics_node")
{


	dxe_ << 0.0,0.0,0.0,0.0,0.0,0.0;
	
	rclcpp::QoS qos(rclcpp::KeepLast(1));
	qos.transient_local();


	robotActualJoint = this ->create_subscription<sensor_msgs::msg::JointState>(
	"/joint_states", 10, std::bind(&IKSolver::robotJointCallback, this, std::placeholders::_1));

	robotJointPub = this -> create_publisher<sensor_msgs::msg::JointState>(
	"/joint_states", 10);

	robotActualRobot = this->create_subscription<std_msgs::msg::Int8>(
    "act_base", 10, std::bind(&IKSolver::robotBaseCallback, this, std::placeholders::_1));

	robotvelPub = this->create_publisher<std_msgs::msg::Float64MultiArray>(
	"/velocity_controller/commands", 10);

	robotposPub = this->create_publisher<std_msgs::msg::Float64MultiArray>(
	"/position_controller/commands", 10);

	robotJointPub = this -> create_publisher<sensor_msgs::msg::JointState>(
	"/joint_states", 10);

	robotvelSub = this->create_subscription<std_msgs::msg::Int32MultiArray>(
	"set_ve", 10, std::bind(&IKSolver::VelsCallback,this, std::placeholders::_1));

	robotEndPub = this->create_publisher<std_msgs::msg::Float32MultiArray>(
	"end_effector_pos",10);

	JTrajectory = this->create_publisher<trajectory_msgs::msg::JointTrajectory>("joint_trajectory_controller/joint_trajectory",10);

	robotDescriptionSubscriber_1= this->create_subscription<std_msgs::msg::String>("/robot1/robot_description",qos,std::bind(&IKSolver::robotDescriptionCB1,this,std::placeholders::_1));
	robotDescriptionSubscriber_2= this->create_subscription<std_msgs::msg::String>("/robot2/robot_description",qos,std::bind(&IKSolver::robotDescriptionCB2,this,std::placeholders::_1));



	while(robotDescription_1.empty() || robotDescription_2.empty())
		{
                   RCLCPP_WARN_STREAM_SKIPFIRST_THROTTLE(get_logger(),*get_clock(),1000,"Waiting for robot model on /robot_description.");
                   rclcpp::spin_some(get_node_base_interface());
	}


	timer_ = this->create_wall_timer(
      1ms, std::bind(&IKSolver::publisher, this));


		
	



}
double normalize( const double value, const double start, const double end ) 
{
  const double width       = end - start   ;   // 
  const double offsetValue = value - start ;   // value relative to 0

  return ( offsetValue - ( floor( offsetValue / width ) * width ) ) + start ;
  // + start to reset back to start of original range
}


void IKSolver::robotJointCallback(const sensor_msgs::msg::JointState::SharedPtr Joints)
{
	act_joints = Joints->position;
	act_velJoints = Joints->velocity;

	flagOK = true;
	
}
void IKSolver::VelsCallback(const std_msgs::msg::Int32MultiArray::SharedPtr Vel)
{
	dxe_ << Vel->data[0]/1000.0 , Vel->data[1]/1000.0, Vel->data[2]/1000.0, Vel->data[3]/100.0, Vel->data[4]/100.0, Vel->data[5]/100.0;
	
	RCLCPP_INFO_STREAM(get_logger(),dxe_);
	
}
void IKSolver::robotBaseCallback(const std_msgs::msg::Int8::SharedPtr msg)
{

	
	//RCLCPP_INFO(this->get_logger(), "I heard: '%d'", msg->data);
	actualBase_ = msg->data;
	
}

void IKSolver::robotDescriptionCB1(const std_msgs::msg::String::SharedPtr robotDescription)
{

//	RCLCPP_INFO(this->get_logger(), "ok", robotDescription->data);
	robotDescription_1=robotDescription->data;
}

void eul_ZYZ(const double beta, const double alpha, Eigen::Matrix3d &T)
{
  T << 
    0, -sin(alpha), cos(alpha) * sin(beta),
    0, cos(alpha), sin(alpha) * sin(beta),
    1, 0, cos(beta);
}

void RPY_MT(const double roll, const double pitch, Eigen::Matrix3d &T)
{
  T << 
    1, 0, sin(pitch),
    0, cos(roll), -cos(pitch)*sin(roll),
    0, sin(roll), cos(pitch)*cos(roll);
}

void eul_kin_ZYZ_dot(const double beta, const double alpha,const double beta_dot, const double alpha_dot,Eigen::Matrix3d &T_dot)
{	
    T_dot <<
      0, -cos(alpha) * alpha_dot, -sin(alpha) * sin(beta) * alpha_dot + cos(alpha) * cos(beta) * beta_dot,
      0, -sin(alpha) * alpha_dot, cos(alpha) * sin(beta) * alpha_dot + cos(beta) * sin(alpha) * beta_dot,
      0, 0, -sin(beta) * beta_dot;
}

void GainMatrix(const double Kx, const double Ky, const double Kz, const double Kroll, const double Kpitch, const double Kyaw, Eigen::Matrix<double,6,6> &GainMK)
{
	GainMK <<
		 Kx,   0.0,  0.0,     0.0,        0.0,    0.0,
		0.0,    Ky,  0.0,     0.0,        0.0,    0.0,
		0.0,   0.0,   Kz,     0.0,        0.0,    0.0,
		0.0,   0.0,  0.0,   Kroll,        0.0,    0.0,
		0.0,   0.0,  0.0,     0.0,     Kpitch,    0.0,
		0.0,   0.0,  0.0,     0.0,        0.0,   Kyaw; 
}
void MatrixAnalytical(const double beta, const double alpha, Eigen::Matrix<double,6,6> &TM_)
{
	double M11,M12,M13,M21,M22,M23,M31,M32,M33;
	double sa,sb,ca,cb;
	sa = sin(alpha);
	sb = sin(beta);
	ca = cos(alpha);
	cb = cos(beta);

	M11 = (-(ca*cb))/(((ca*ca)*sb) + ((sa*sa)*sb));
	M12 = (-(ca*sa))/(((ca*ca)*sb) + ((sa*sa)*sb));
	M13 = 1.0;

	M21 = (-sb)/((ca*ca) + (sa*sa));
	M22 = (ca)/((ca*ca) + (sa*sa));
	M23 = 0.0;

	M31 = (ca)/(((ca*ca)*sb) + ((sa*sa)*(sb)));
	M32 = (sa)/(((ca*ca)*sb) + ((sa*sa)*(sb)));
	M33 = 0.0;

	TM_ <<
		1.0,   1.0,  1.0,     0.0,        0.0,    0.0,
		1.0,   1.0,  1.0,     0.0,        0.0,    0.0,
		1.0,   1.0,  1.0,     0.0,        0.0,    0.0,
		0.0,   0.0,  0.0,     M11, 		  M12, 	  M13,
		0.0,   0.0,  0.0,     M21, 		  M22, 	  M23,
		0.0,   0.0,  0.0,     M31, 		  M32, 	  M33;
}
void IKSolver::robotDescriptionCB2(const std_msgs::msg::String::SharedPtr robotDescription)
{

//	RCLCPP_INFO(this->get_logger(), "ok", robotDescription->data);
	robotDescription_2=robotDescription->data;
}
void IKSolver::publisher()
{
	Eigen::MatrixXd M_TA, M_TA_dot_, JA_;
	Eigen::Matrix3d M_B;

	auto jointStates_ = sensor_msgs::msg::JointState();
	auto veljointPub = std_msgs::msg::Float64MultiArray();
	auto posjointPub = std_msgs::msg::Float64MultiArray();
	auto end_effector_p_R_ = std_msgs::msg::Float32MultiArray();
	joint_vector_t dq_, act_dq_;
	float Base1,Base2;
	float d_b1,d_b2;
	Eigen::Matrix<double,6,6> AnalyticalJac, inverseJac, GainsK;
	Eigen::Matrix<double,6,1> q_Next, q_act;
	Eigen::Matrix<double,6,1> ed_;
	Eigen::Matrix<double,6,1> e_;

	KDL::Tree tree;


	//Analytical Transformation
	//
	//				 | I     0	 |
	//          Ve = | 			 | * J(q)
	//				 | 0    B⁻¹(T)|
	//									T(rpy)
	//
	M_TA = Eigen::MatrixXd::Zero(6,6);
	M_TA.block<3,3>(0,0) = Eigen::Matrix<double, 3, 3>::Identity();
	
	
		switch(actualBase_){
			

			case 0:

					RCLCPP_INFO(this->get_logger(), "actualBase_: '%d'", actualBase_);

					linkSource = "robot1/LINK_1";
					linkEnd = "robot1/end-effector";
					
					if (!kdl_parser::treeFromString(robotDescription_1,tree))
							RCLCPP_ERROR_STREAM(get_logger(),"Failed to construct KDL tree.");
							
					if ((!tree.getChain(linkSource,linkEnd,chain_)))  {
							RCLCPP_ERROR_STREAM(get_logger(),"Failed to get chain from KDL tree.");
						
						}
					jacobian_solver_ = std::make_unique<KDL::ChainJntToJacSolver>(chain_);
					foward_solver_ = std::make_unique<KDL::ChainFkSolverPos_recursive>(chain_);
					jacobian_.resize(chain_.getNrOfJoints());
					q_.resize(chain_.getNrOfJoints());
					q_(0)=act_joints[0];
					q_(1)=act_joints[1];
					q_(2)=act_joints[2];
					q_(3)=act_joints[3];
					q_(4)=act_joints[4];
					q_(5)=act_joints[5];
					Base1 = act_joints[6]*57.2958;
					Base2 = act_joints[7]*57.2958;
					Base1 = normalize(Base1,-360,360);
					Base2 = normalize(Base2,-360,360);
					
					

					jacobian_solver_->JntToJac(q_, jacobian_);
					foward_solver_ ->JntToCart(q_,FKkin);
					x = FKkin.p.x();
					y = FKkin.p.y();
					z = FKkin.p.z();
					FKkin.M.GetRPY(roll,pitch,yaw);
					inverseJac = pinv(jacobian_.data);
					dq_ = inverseJac * dxe_;
					
					//q_Next = q_.data + dq_*0.01;
					
					q_Next[0] = q_act[0] + (dq_[0] *0.001);
					q_Next[1] = q_act[1] + (dq_[1] *0.001);
					q_Next[2] = q_act[2] + (dq_[2] *0.001);
					q_Next[3] = q_act[3] + (dq_[3] *0.001);
					q_Next[4] = q_act[4] + (dq_[4] *0.001);
					q_Next[5] = q_act[5] + (dq_[5] *0.001);


					commandDrives = false;
					for (int i = 0; i < 6; i++) {
						if (dxe_[i] != 0) {
							commandDrives = true;
						}
					} 
					if (commandDrives){
						for (int i = 0; i < 6; i++) {
							q_act[i] = q_Next[i];
							
						} 
					}
					else{
						for (int i = 0; i < 6; i++) {
							q_act[i] = act_joints[i];
						} 
					}

					RCLCPP_INFO_STREAM(get_logger(),(q_Next));
					//RCLCPP_INFO_STREAM(get_logger(),(Base2));
					d_b1 = 0.0;
					d_b2 = 0.0;
					/*
					while (abs(Base2) <= 85 || abs(Base2) >= 95 ){
							d_b2 = 1.00;
							flag = false;
							break;
						}
					while ((((Base1) <= -1 || (Base1) >= 5 ) ) && (d_b2 == 0.0)){		
							if (flag != true){
								veljointPub.data = {0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0};
								robotvelPub -> publish(veljointPub);	
								sleep(1);
								flag = true;
							}
							d_b1 = -1.00;
							break;
					}
					*/
					//std::cout << q_.data;
					jointStates_.header.stamp = rclcpp::Node::get_clock() -> now();
					Postrajectory.header.stamp = rclcpp::Node::get_clock() -> now();					
					trajPosJoint.time_from_start.nanosec = 1000000;
					Postrajectory.joint_names = {"J1","J2","J3","J4","J5","J6","BASE1","BASE2"};
					jointStates_.name = {"J1","J2","J3","J4","J5","J6"};
					jointStates_.velocity= {dq_[0],dq_[1],dq_[2],dq_[3],dq_[4],dq_[5]};
					veljointPub.data = {dq_[0],dq_[1],dq_[2],dq_[3],dq_[4],dq_[5]};
					jointStates_.position= {q_Next[0],q_Next[1],q_Next[2],q_Next[3],q_Next[4],q_Next[5]};
					posjointPub.data= {q_Next[0],q_Next[1],q_Next[2],q_Next[3],q_Next[4],q_Next[5]};
					trajPosJoint.positions = {q_Next[0],q_Next[1],q_Next[2],q_Next[3],q_Next[4],q_Next[5],act_joints[6], act_joints[7]};
					Postrajectory.points = {trajPosJoint};
				break;
			case 1:

					//RCLCPP_INFO(this->get_logger(), "actualBase_: '%d'", actualBase_);
					
					linkSource = "robot2/LINK_7";
					linkEnd = "robot2/end-effector";

					if (!kdl_parser::treeFromString(robotDescription_2,tree))
							RCLCPP_ERROR_STREAM(get_logger(),"Failed to construct KDL tree.");
							
					if ((!tree.getChain(linkSource,linkEnd,chain_)))  {
							RCLCPP_ERROR_STREAM(get_logger(),"Failed to get chain from KDL tree.");
						
						}

					jacobian_solver_ = std::make_unique<KDL::ChainJntToJacSolver>(chain_);
					foward_solver_ = std::make_unique<KDL::ChainFkSolverPos_recursive>(chain_);
					jacobian_.resize(chain_.getNrOfJoints());
					q_.resize(chain_.getNrOfJoints());
					q_(0)=act_joints[5];
					q_(1)=act_joints[4];
					q_(2)=act_joints[3];
					q_(3)=act_joints[2];
					q_(4)=act_joints[1];
					q_(5)=act_joints[0];
					Base1 = act_joints[6]*57.2958;
					Base2 = act_joints[7]*57.2958;
					Base1 = normalize(Base1,-360,360);
					Base2 = normalize(Base2,-360,360);
					RCLCPP_INFO_STREAM(get_logger(),(Base1));
					RCLCPP_INFO_STREAM(get_logger(),(Base2));
					d_b1 = 0.0;
					d_b2 = 0.0;
					/*
					while (abs(Base1) <= 85 || abs(Base1) >= 95 ){
							d_b1 = 1.0;
							flag = false;
							break;
						}
					while ((((Base2) <= -5 || (Base2) >= 5 ) ) && (d_b1 == 0.0)){
							if (flag != true){
								//veljointPub.data = {0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0};
								//robotvelPub -> publish(veljointPub);	
								sleep(1);
								flag = true;
							}
							d_b2 = -1.0;
							break;
					}*/
					jacobian_solver_->JntToJac(q_, jacobian_);
					foward_solver_ ->JntToCart(q_,FKkin);
					x = FKkin.p.x();
					y = FKkin.p.y();
					z = FKkin.p.z();
					FKkin.M.GetRPY(roll,pitch,yaw);
					//KDL::changeBase()
					inverseJac = pinv(jacobian_.data);
					dq_ = inverseJac * dxe_;
					
					//q_Next = q_.data + dq_*0.01;
					q_Next[0] = q_act[0] + (dq_[0] *0.001);
					q_Next[1] = q_act[1] + (dq_[1] *0.001);
					q_Next[2] = q_act[2] + (dq_[2] *0.001);
					q_Next[3] = q_act[3] + (dq_[3] *0.001);
					q_Next[4] = q_act[4] + (dq_[4] *0.001);
					q_Next[5] = q_act[5] + (dq_[5] *0.001);


					commandDrives = false;
					for (int i = 0; i < 6; i++) {
						if (dxe_[i] != 0) {
							commandDrives = true;
						}
					} 
					if (commandDrives){
						for (int i = 0; i < 6; i++) {
							q_act[i] = q_Next[i];
							
						} 
					}
					else{
						q_act[0] = act_joints[5];
						q_act[1] = act_joints[4];
						q_act[2] = act_joints[3];
						q_act[3] = act_joints[2];
						q_act[4] = act_joints[1];
						q_act[5] = act_joints[0];
					}
				
					//RCLCPP_INFO_STREAM(get_logger(),q_.data);
					
					jointStates_.header.stamp = rclcpp::Node::get_clock() -> now();
					Postrajectory.header.stamp = rclcpp::Node::get_clock() -> now();
					Postrajectory.header.frame_id = "" ;
					trajPosJoint.time_from_start.nanosec = 1000000;
					Postrajectory.joint_names = {"J1","J2","J3","J4","J5","J6","BASE1","BASE2"};
					jointStates_.name = {"J1","J2","J3","J4","J5","J6"};
					jointStates_.velocity= {dq_[5],dq_[4],dq_[3],dq_[2],dq_[1],dq_[0]};
					veljointPub.data = {dq_[5],dq_[4],dq_[3],dq_[2],dq_[1],dq_[0]};
					jointStates_.position= {q_Next[5],q_Next[4],q_Next[3],q_Next[2],q_Next[1],q_Next[0]};
					posjointPub.data= {q_Next[5],q_Next[4],q_Next[3],q_Next[2],q_Next[1],q_Next[0]};
					trajPosJoint.positions = {q_Next[5],q_Next[4],q_Next[3],q_Next[2],q_Next[1],q_Next[0],act_joints[6],act_joints[7]};
					Postrajectory.points = {trajPosJoint};

				break;
				
		}


		//std::cout << "x:" << x << " y:" << y << " z:" << z << " roll:" << roll << " pitch:" << pitch << " yaw:" << yaw << std::endl;
		end_effector_p_R_.data = {(float)x,(float)y,(float)z,(float)roll,(float)pitch,(float)yaw};
		
		//RCLCPP_INFO(this->get_logger(),end_effector_p_R_);
		//RCLCPP_INFO_STREAM(get_logger(),end_effector_p_R_.data[0]);
		robotEndPub -> publish(end_effector_p_R_);  
		
		if (flagOK && commandDrives) {
			//robotvelPub -> publish(veljointPub);	
			//JTrajectory -> publish(Postrajectory);
			robotposPub -> publish(posjointPub);
			commandDrives = false;
		}
		
		//robotJointPub -> publish(jointStates_);	
		

		
		
}


int main(int argc,char* argv[])
{
	rclcpp::init(argc,argv);
	rclcpp::spin(std::make_shared<IKSolver>());
	rclcpp::shutdown();
	return 0;
}


