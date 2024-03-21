
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
#include <sensor_msgs/msg/joint_state.hpp>

const double PI = 3.14159;


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

		
		void robotBaseCallback(const std_msgs::msg::Int8::SharedPtr msg);
		rclcpp::Subscription<std_msgs::msg::Int8>::SharedPtr robotActualRobot;

		KDL::Frame goal_;
		KDL::Chain chain_;
		KDL::ChainIkSolverPos_LMA *ikSolverPos_;
		KDL::JntArray q_;

		joint_vector_t dq_;
		
		std::unique_ptr<KDL::ChainJntToJacSolver> jacobian_solver_;
  		KDL::Jacobian jacobian_;
		int actualBase_;
		std::string robotDescription_;
		void robotDescriptionCB(const std_msgs::msg::String::SharedPtr robotDescription);
		
		rclcpp::Subscription<std_msgs::msg::String>::SharedPtr robotDescriptionSubscriber_;
		
		std::string linkSource;
		std::string linkEnd;
		rclcpp::TimerBase::SharedPtr timer_;
		rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr robotJointPub;
		
		
		
};

IKSolver::IKSolver(): Node("ik_kinematics_node")
{


	
	rclcpp::QoS qos(rclcpp::KeepLast(1));
	qos.transient_local();


	
	robotActualRobot = this->create_subscription<std_msgs::msg::Int8>(
    "act_base", 10, std::bind(&IKSolver::robotBaseCallback, this, std::placeholders::_1));

	robotDescriptionSubscriber_= this->create_subscription<std_msgs::msg::String>("robot_description",qos,std::bind(&IKSolver::robotDescriptionCB,this,std::placeholders::_1));




	robotJointPub = this -> create_publisher<sensor_msgs::msg::JointState>(
	"/joint_states", 10);

	timer_ = this->create_wall_timer(
      100ms, std::bind(&IKSolver::publisher, this));


		
	



}


void IKSolver::publisher()
{

	while(robotDescription_.empty())
		{
                   RCLCPP_WARN_STREAM_SKIPFIRST_THROTTLE(get_logger(),*get_clock(),1000,"Waiting for robot model on /robot_description.");
                   rclcpp::spin_some(get_node_base_interface());
	}

	switch(actualBase_){

		case 0:
				RCLCPP_INFO(this->get_logger(), "actualBase_: '%d'", actualBase_);
				linkSource = "robot1/LINK_1";
				linkEnd = "robot1/end-effector";
			break;
		case 1:
				RCLCPP_INFO(this->get_logger(), "actualBase_: '%d'", actualBase_);
				std::cout <<actualBase_;
				linkSource = "robot2/LINK_7";
				linkEnd = "robot2/end-effector";
				
			
		}

	KDL::Tree tree;
	if (!kdl_parser::treeFromString(robotDescription_,tree))
		RCLCPP_ERROR_STREAM(get_logger(),"Failed to construct KDL tree.");
		
	if ((!tree.getChain(linkSource,linkEnd,chain_)))  {
		RCLCPP_ERROR_STREAM(get_logger(),"Failed to get chain from KDL tree.");
	
	}
	
	jacobian_solver_ = std::make_unique<KDL::ChainJntToJacSolver>(chain_);
	jacobian_.resize(chain_.getNrOfJoints());
	q_.resize(chain_.getNrOfJoints());

	jacobian_solver_->JntToJac(q_, jacobian_);
	
	Eigen::Matrix<double,6,1> dxe_;
	dxe_ << 0.0 , 0.0 , 10.0, 0.0, 0.0, 0.0;
	Eigen::Matrix<double,6,1> L;
	L << 1.0 , 1.0 , 1.0, 0.01, 0.01, 0.01;
	Eigen::Matrix<double,6,1> q_Next;
	q_Next << 0.0 , 0.0 , 0.0, 0.0, 0.0, 0.0;
	

	// A copy of chain_ is not created inside!
	
	ikSolverPos_=new KDL::ChainIkSolverPos_LMA(chain_,L);
	Eigen::Matrix<double,6,6>  inverseJac = pinv(jacobian_.data);


	joint_vector_t dq_ = inverseJac * dxe_;
	
	ikSolverPos_->display_information=false;

	q_Next = q_.data + dq_*0.100;
	RCLCPP_INFO_STREAM(get_logger(),q_.data);
	auto jointStates_ = sensor_msgs::msg::JointState();
	jointStates_.header.stamp = rclcpp::Node::get_clock() -> now();
	jointStates_.name = {"J1","J2","J3","J4","J5","J6"};
	jointStates_.velocity= {dq_[0],dq_[1],dq_[2],dq_[3],dq_[4],dq_[5]};
	jointStates_.position= {0.0,q_Next[1],q_Next[2],q_Next[3],q_Next[4],q_Next[5]};
	//robotJointPub -> publish(jointStates_);
}
void IKSolver::robotBaseCallback(const std_msgs::msg::Int8::SharedPtr msg)
{

	RCLCPP_INFO(this->get_logger(), "I heard: '%d'", msg->data);
	actualBase_ = msg->data;
	
}

void IKSolver::robotDescriptionCB(const std_msgs::msg::String::SharedPtr robotDescription)
{
	RCLCPP_INFO(this->get_logger(), "ok", robotDescription->data);
	robotDescription_=robotDescription->data;
}



int main(int argc,char* argv[])
{
	rclcpp::init(argc,argv);
	rclcpp::spin(std::make_shared<IKSolver>());
	rclcpp::shutdown();
	return 0;
}


