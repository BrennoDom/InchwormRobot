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

#include <kdl_parser/kdl_parser.hpp>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>

#include <std_msgs/msg/float64.hpp>
#include <std_msgs/msg/int8.hpp>
#include <std_msgs/msg/float64.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/int32_multi_array.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
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
		
		KDL::Frame FKkin, FKNext, FKAct;
		KDL::Chain chain_;
		KDL::JntArray q_, q_nxt, q_act;

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

		//
		
		
    
		
};

IKSolver::IKSolver(): Node("ik_kinematics_node")
{


	dxe_ << 0.0,0.0,0.0,0.0,0.0,0.0;
	actual_dxe_ << 0.0,0.0,0.0,0.0,0.0,0.0;
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

void IKSolver::robotDescriptionCB2(const std_msgs::msg::String::SharedPtr robotDescription)
{

//	RCLCPP_INFO(this->get_logger(), "ok", robotDescription->data);
	robotDescription_2=robotDescription->data;
}
void IKSolver::publisher()
{
	Eigen::MatrixXd M_TA, M_TA_dot_, JA_;
	Eigen::Matrix3d M_B;

	bool command;
	auto jointStates_ = sensor_msgs::msg::JointState();
	auto veljointPub = std_msgs::msg::Float64MultiArray();
	auto posjointPub = std_msgs::msg::Float64MultiArray();
	auto end_effector_p_R_ = std_msgs::msg::Float32MultiArray();
	joint_vector_t dq_, act_dq_;
	float Base1,Base2;
	float d_b1,d_b2;
	Eigen::Matrix<double,6,6>  inverseJac, GainsK;
	Eigen::Matrix<double,6,1> q_Next;
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
					q_nxt.resize(chain_.getNrOfJoints());
					q_act.resize(chain_.getNrOfJoints());
					q_act(0)=act_joints[0];
					q_act(1)=act_joints[1];
					q_act(2)=act_joints[2];
					q_act(3)=act_joints[3];
					q_act(4)=act_joints[4];
					q_act(5)=act_joints[5];
					if (!initialcondOK || commandDrives) {						
						q_(0)=act_joints[0];
						q_(1)=act_joints[1];
						q_(2)=act_joints[2];
						q_(3)=act_joints[3];
						q_(4)=act_joints[4];
						q_(5)=act_joints[5];
						initialcondOK = true;
					}
					if (dxe_(0) == 0 && dxe_(1) == 0 && dxe_(2) == 0 && dxe_(3) == 0 && dxe_(4) == 0 && dxe_(5) == 0){
						commandDrives = false;
					}else{
						commandDrives = true;
					}
					Base1 = act_joints[6]*57.2958;
					Base2 = act_joints[7]*57.2958;
					Base1 = normalize(Base1,-360,360);
					Base2 = normalize(Base2,-360,360);
					jacobian_solver_->JntToJac(q_act, jacobian_);
					foward_solver_ ->JntToCart(q_,FKkin);
					foward_solver_ ->JntToCart(q_act,FKAct);
					x = FKAct.p.x();
					y = FKAct.p.y();
					z = FKAct.p.z();
					FKAct.M.GetRPY(roll,pitch,yaw);
					xe_(0) = FKkin.p.x();
					xe_(1) = FKkin.p.y();
					xe_(2) = FKkin.p.z();
					FKkin.M.GetRPY(xe_(3),xe_(4),xe_(5));
					RPY_MT(roll,pitch,M_B);
					M_TA.block<3,3>(3,3) = pinv(M_B);
					JA_ = M_TA * jacobian_.data;
					inverseJac = pinv(jacobian_.data);
					GainMatrix(0,0,0,0,0,0,GainsK);
					act_dq_(0) = act_velJoints[0];
					act_dq_(1) = act_velJoints[1];
					act_dq_(2) = act_velJoints[2];
					act_dq_(3) = act_velJoints[3];
					act_dq_(4) = act_velJoints[4];
					act_dq_(5) = act_velJoints[5];

					//std::cout << xd_ << std::endl;

					e_(0) = xd_[0] - xe_[0];
					e_(1) = xd_[1] - xe_[1];
					e_(2) = xd_[2] - xe_[2];
					e_(3) = xd_[3] - xe_[3];
					e_(4) = xd_[4] - xe_[4];
					e_(5) = xd_[5] - xe_[5];
					
					dq_ = inverseJac*(dxe_ + GainsK*e_);
					dq_ant = dq_;
					
					/*
					xd_(0) = x + (dxe_[0]*0.001);
					xd_(1) = y + (dxe_[1]*0.001);
					xd_(2) = z + (dxe_[2]*0.001);
					xd_(3) = roll + (dxe_[3]*0.001);
					xd_(4) = pitch + (dxe_[4]*0.001);
					xd_(5) = yaw + (dxe_[5]*0.001);*/

					q_Next = q_.data + dq_ * 0.001;
					q_nxt(0) = q_Next[0];
					q_nxt(1) = q_Next[1];
					q_nxt(2) = q_Next[2];
					q_nxt(3) = q_Next[3];
					q_nxt(4) = q_Next[4];
					q_nxt(5) = q_Next[5];
					foward_solver_ ->JntToCart(q_nxt,FKNext);
					xd_(0) = FKNext.p.x();
					xd_(1) = FKNext.p.y();
					xd_(2) = FKNext.p.z();
					FKNext.M.GetRPY(xd_(3),xd_(4),xd_(5));

					std::cout << (GainsK*e_) << std::endl;
					//RCLCPP_INFO_STREAM(get_logger(),(Base1));
					//RCLCPP_INFO_STREAM(get_logger(),(Base2));
					d_b1 = 0.0;
					d_b2 = 0.0;
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
					//std::cout << flag;
					jointStates_.header.stamp = rclcpp::Node::get_clock() -> now();
					jointStates_.name = {"J1","J2","J3","J4","J5","J6"};
					jointStates_.velocity= {dq_[0],dq_[1],dq_[2],dq_[3],dq_[4],dq_[5]};
					veljointPub.data = {dq_[0],dq_[1],dq_[2],dq_[3],dq_[4],dq_[5],d_b1,d_b2};
					jointStates_.position= {q_Next[0],q_Next[1],q_Next[2],q_Next[3],q_Next[4],q_Next[5]};
					posjointPub.data= {q_Next[0],q_Next[1],q_Next[2],q_Next[3],q_Next[4],q_Next[5],act_joints[6], act_joints[7]};
					
				break;
			case 1:

					RCLCPP_INFO(this->get_logger(), "actualBase_: '%d'", actualBase_);
					
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
					
					while (abs(Base1) <= 85 || abs(Base1) >= 95 ){
							d_b1 = 1.0;
							flag = false;
							break;
						}
					while ((((Base2) <= -5 || (Base2) >= 5 ) ) && (d_b1 == 0.0)){
							if (flag != true){
								veljointPub.data = {0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0};
								robotvelPub -> publish(veljointPub);	
								sleep(1);
								flag = true;
							}
							d_b2 = -1.0;
							break;
					}
					jacobian_solver_->JntToJac(q_, jacobian_);
					foward_solver_ ->JntToCart(q_,FKkin);
					x = FKkin.p.x();
					y = FKkin.p.y();
					z = FKkin.p.z();
					FKkin.M.GetRPY(roll,pitch,yaw);
					//KDL::changeBase()
					inverseJac = pinv(jacobian_.data);
					dq_ = inverseJac * dxe_;
					
					q_Next = q_.data + dq_*0.001;
					//RCLCPP_INFO_STREAM(get_logger(),q_.data);
					
					jointStates_.header.stamp = rclcpp::Node::get_clock() -> now();
					jointStates_.name = {"J1","J2","J3","J4","J5","J6"};
					jointStates_.velocity= {dq_[5],dq_[4],dq_[3],dq_[2],dq_[1],dq_[0]};
					veljointPub.data = {dq_[5],dq_[4],dq_[3],dq_[2],dq_[1],dq_[0],d_b1,d_b2};
					jointStates_.position= {q_Next[5],q_Next[4],q_Next[3],q_Next[2],q_Next[1],q_Next[0]};
					posjointPub.data= {q_Next[5],q_Next[4],q_Next[3],q_Next[2],q_Next[1],q_Next[0],act_joints[6],act_joints[7]};
				break;
				
		}


		//std::cout << "x:" << x << " y:" << y << " z:" << z << " roll:" << roll << " pitch:" << pitch << " yaw:" << yaw << std::endl;
		end_effector_p_R_.data = {(float)x,(float)y,(float)z,(float)roll,(float)pitch,(float)yaw};
		
		//RCLCPP_INFO(this->get_logger(),end_effector_p_R_);
		//RCLCPP_INFO_STREAM(get_logger(),end_effector_p_R_.data[0]);
		robotEndPub -> publish(end_effector_p_R_);  
		if (flagOK && initialcondOK && commandDrives){
			//robotposPub -> publish(posjointPub);
			
			//robotvelPub -> publish(veljointPub);
			
		}else{
			RCLCPP_INFO(this->get_logger(),"waiting reading from joints");
		}
		//robotposPub -> publish(posjointPub);
				//REAL
		//robotJointPub -> publish(jointStates_);		//SIMULATE
		robotvelPub -> publish(veljointPub);		//REAL
		
		
	/*	
		for (int i = 0; i < 4; i++){
			for (int j = 0; j < 4; j++) {
				double a = FKkin(i, j);
				if (a < 0.0001 && a > -0.001) {
					a = 0.0;
				}
				std::cout << std::setprecision(4) << a << "\t\t";
			}
			std::cout << std::endl;
		}*/

		
		
}


int main(int argc,char* argv[])
{
	rclcpp::init(argc,argv);
	rclcpp::spin(std::make_shared<IKSolver>());
	rclcpp::shutdown();
	return 0;
}


