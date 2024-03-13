
#include <chrono>
#include <functional>
#include <memory>
#include <sstream>
#include <string>

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
#include <std_msgs/msg/string.hpp>

const double PI = 3.14159;


using namespace std::chrono_literals;

unsigned int v = 2;

class IKSolver: public rclcpp::Node
{
	using joint_vector_t = Eigen::Vector<double, 6>;
 	using cartesian_vector_t = Eigen::Vector<double, 6>;
	public:
		IKSolver(void);
		~IKSolver(void);

	private:
		rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr Inversepub;

		KDL::Frame goal_;
		KDL::Chain chain_;
		KDL::ChainIkSolverPos_LMA *ikSolverPos_;
		KDL::JntArray q_;
		std::string robotDescription_;
		std::unique_ptr<KDL::ChainJntToJacSolver> jacobian_solver_;
  		KDL::Jacobian jacobian_;


		void robotDescriptionCB(const std_msgs::msg::String::SharedPtr robotDescription);
};

IKSolver::IKSolver(void): Node("ik_kinematics_node"), q_(2)
{
	Inversepub=create_publisher<std_msgs::msg::Float64>("inverse_controller/command",100);
	
	rclcpp::QoS qos(rclcpp::KeepLast(1));
	qos.transient_local();
	auto robotDescriptionSubscriber_=create_subscription<std_msgs::msg::String>("robot_description",qos,std::bind(&IKSolver::robotDescriptionCB,this,std::placeholders::_1));
	while(robotDescription_.empty())
	{
                RCLCPP_WARN_STREAM_SKIPFIRST_THROTTLE(get_logger(),*get_clock(),1000,"Waiting for robot model on /robot_description.");
                rclcpp::spin_some(get_node_base_interface());
	}

	KDL::Tree tree;
	if (!kdl_parser::treeFromString(robotDescription_,tree))
		RCLCPP_ERROR_STREAM(get_logger(),"Failed to construct KDL tree.");
		
	if (!tree.getChain("world","robot1/end-effector",chain_))
		RCLCPP_ERROR_STREAM(get_logger(),"Failed to get chain from KDL tree.");
	

	
	jacobian_solver_ = std::make_unique<KDL::ChainJntToJacSolver>(chain_);
    jacobian_.resize(chain_.getNrOfJoints());
	q_.resize(chain_.getNrOfJoints());

	jacobian_solver_->JntToJac(q_, jacobian_);
	
	Eigen::Matrix<double,6,1> L;
	L << 1.0 , 1.0 , 1.0, 0.01, 0.01, 0.01;
	
	// A copy of chain_ is not created inside!
	ikSolverPos_=new KDL::ChainIkSolverPos_LMA(chain_,L);
	auto teste = chain_.getSegment(1).getInertia();
	RCLCPP_INFO_STREAM(get_logger(),jacobian_.data);
	ikSolverPos_->display_information=false;
}


IKSolver::~IKSolver(void)
{
	delete ikSolverPos_;
}

void IKSolver::robotDescriptionCB(const std_msgs::msg::String::SharedPtr robotDescription)
{
	robotDescription_=robotDescription->data;
}

int main(int argc,char* argv[])
{
	rclcpp::init(argc,argv);
	rclcpp::spin(std::make_shared<IKSolver>());
	rclcpp::shutdown();
	return 0;
}