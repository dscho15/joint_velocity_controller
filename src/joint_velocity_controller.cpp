#include <ros/ros.h>

#include <std_msgs/Float64MultiArray.h>
#include <fstream>

#include <Eigen/Dense>
#include <Eigen/Core>
#include <Eigen/LU>
#include <Eigen/SVD>

#include <kdl_parser/kdl_parser.hpp>
#include <kdl/chaindynparam.hpp>
#include <kdl/jntarray.hpp>
#include <kdl/chainjnttojacsolver.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/chainiksolverpos_nr.hpp>
#include <kdl/chainiksolvervel_pinv.hpp>

constexpr int JOINTS = 7;
KDL::Tree* tree;
KDL::Chain* chain;

Eigen::Matrix<double, 3, 3> RPY(double alpha, double beta, double gamma) 
{
	static Eigen::Matrix<double, 3, 3> R;
	R(0, 0) = std::cos(alpha)*std::cos(beta); R(0, 1) = std::cos(alpha)*std::sin(beta)*std::sin(gamma) - std::sin(alpha)*std::cos(gamma); R(0, 2) = std::cos(alpha)*std::sin(beta)*std::cos(gamma) + std::sin(alpha)*std::sin(gamma);
	R(1, 0) = std::sin(alpha)*std::cos(beta); R(1, 1) = std::sin(alpha)*std::sin(beta)*std::sin(gamma) + std::cos(alpha)*std::cos(gamma); R(1, 2) = std::sin(alpha)*std::sin(beta)*std::cos(gamma) - std::cos(alpha)*std::sin(gamma);
	R(2, 0) = -std::sin(beta);                R(2, 1) = std::cos(beta)*std::sin(gamma);                                                   R(2, 2) = std::cos(beta)*std::cos(gamma);
	return R;
}

Eigen::Vector3d inverse_RPY(Eigen::Matrix<double, 3, 3> R) 
{
    static Eigen::Vector3d rpy;
	static double alpha, beta, gamma;

	beta = std::atan2(-R(2, 0), std::sqrt(R(0,0)*R(0,0) + R(1, 0)*R(1, 0)));

	if (std::cos(beta) != 0) 
	{
		alpha = std::atan2(R(1, 0) / std::cos(beta), R(0, 0) / std::cos(beta));
		gamma = std::atan2(R(2, 1) / std::cos(beta), R(2, 2) / std::cos(beta));
	} 
	else if (beta < 0) 
	{
		alpha = 0;
		gamma = std::atan2(R(0, 1), R(1, 1));
	} 
	else 
	{
		alpha = 0;
		gamma = -std::atan2(R(0, 1), R(1, 1));
	}

    rpy << alpha, beta, gamma;
    return rpy;
}

Eigen::VectorXd cartesian_pose(const Eigen::VectorXd q_cur_)
{
    static KDL::ChainFkSolverPos_recursive fk_solver(*chain);
    static KDL::Frame x_cur;
    static KDL::JntArray q_cur(JOINTS);
    static Eigen::VectorXd x_err(6);

    // Load data into KDL framework
    q_cur.data = q_cur_; 

    // Determine the Cartesian coords
    fk_solver.JntToCart(q_cur, x_cur);

    // Load into matrices
    Eigen::Matrix3d R_cur = Eigen::Map<Eigen::Matrix<double, 3, 3, Eigen::RowMajor>>(x_cur.M.data);

    // Compute Xerror
    x_err.block(0, 0, 3, 1) << x_cur.p.data[0], x_cur.p.data[1], x_cur.p.data[2];
    x_err.block(3, 0, 3, 1) << inverse_RPY(R_cur);

    return x_err;
}

Eigen::VectorXd x_error_euler(const Eigen::VectorXd q_cur_, const Eigen::VectorXd q_d_)
{
    static KDL::ChainFkSolverPos_recursive fk_solver(*chain);
    static KDL::Frame x_cur, x_d;
    static KDL::JntArray q_cur(JOINTS), q_d(JOINTS);
    static Eigen::VectorXd x_err(6);

    // Load data into KDL framework
    q_cur.data = q_cur_; 
    q_d.data = q_d_;

    // Determine the Cartesian coords
    fk_solver.JntToCart(q_cur, x_cur);
    fk_solver.JntToCart(q_d, x_d);

    // Load into matrices
    Eigen::Matrix3d R_cur = Eigen::Map<Eigen::Matrix<double, 3, 3, Eigen::RowMajor>>(x_cur.M.data);
    Eigen::Matrix3d R_d =  Eigen::Map<Eigen::Matrix<double, 3, 3, Eigen::RowMajor>>(x_d.M.data);

    // Compute Xerror
    x_err.block(0, 0, 3, 1) << x_d.p.data[0] - x_cur.p.data[0], x_d.p.data[1] - x_cur.p.data[1], x_d.p.data[2] - x_cur.p.data[2];
    x_err.block(3, 0, 3, 1) << inverse_RPY(R_d) - inverse_RPY(R_cur);
    
    ROS_INFO_STREAM(x_err);

    return x_err;
}

Eigen::VectorXd x_error_quat(const Eigen::VectorXd q_cur_, const Eigen::VectorXd q_d_)
{
    static KDL::ChainFkSolverPos_recursive fk_solver(*chain);
    static KDL::Frame x_cur, x_d;
    static KDL::JntArray q_cur(JOINTS), q_d(JOINTS);
    static Eigen::VectorXd x_err(6);

    // Load data into KDL framework
    q_cur.data = q_cur_; 
    q_d.data = q_d_;

    // Determine the Cartesian coords
    fk_solver.JntToCart(q_cur, x_cur);
    fk_solver.JntToCart(q_d, x_d);

    // Load into matrices
    Eigen::Matrix3d R_cur = Eigen::Map<Eigen::Matrix<double, 3, 3, Eigen::RowMajor>>(x_cur.M.data);
    Eigen::Matrix3d R_d =  Eigen::Map<Eigen::Matrix<double, 3, 3, Eigen::RowMajor>>(x_d.M.data);

    // Define quaternions 
    Eigen::Quaterniond quat_cur(R_cur);
    Eigen::Quaterniond quat_d(R_d);
    Eigen::Quaterniond quat_e;

    // Calculate error quaternion
    quat_e = quat_d * quat_cur.inverse();

    // Compute Xerror
    x_err.block(0, 0, 3, 1) << x_d.p.data[0] - x_cur.p.data[0], x_d.p.data[1] - x_cur.p.data[1], x_d.p.data[2] - x_cur.p.data[2];
    x_err.block(3, 0, 3, 1) << quat_e.x(), quat_e.y(), quat_e.z();

    return x_err;
}

Eigen::Matrix<double, 6, -1> jacobi_geo_matrix(const Eigen::VectorXd & q_)
{
    static KDL::ChainJntToJacSolver jacobian_solve(*chain);
    static KDL::Jacobian jacobian(JOINTS);
    static KDL::JntArray q(JOINTS);
    q.data = q_;
    jacobian_solve.JntToJac(q, jacobian);
    return jacobian.data;
}

Eigen::Matrix<double, 6, -1> jacobi_analyt_matrix(const Eigen::VectorXd & q_)
{
    static KDL::ChainFkSolverPos_recursive fk_solver(*chain);
    static Eigen::Matrix<double, 6, -1> jacobi_analyt;
    static Eigen::Matrix<double, 6,  6> T;
    static KDL::JntArray q_cur(JOINTS);
    static Eigen::Matrix3d R;
    static KDL::Frame x_cur;
    
    // Forward kinematic solver
    q_cur.data = q_;
    fk_solver.JntToCart(q_cur, x_cur);
    Eigen::Matrix3d R_cur = Eigen::Map<Eigen::Matrix<double, 3, 3, Eigen::RowMajor>>(x_cur.M.data);
    auto RPY = inverse_RPY(R_cur);

    // Find current ZYX Euler angles
	double alpha = RPY(0);
	double beta = RPY(1);
	double gamma = RPY(2);

    // Compute rotation_matrix
    T = Eigen::MatrixXd::Identity(6, 6);

    R << 0,    -std::sin(alpha),    std::cos(beta)*std::cos(alpha), 
         0,     std::cos(alpha),    std::cos(beta)*std::sin(alpha), 
         1,     0,                 -std::sin(beta);

    T.bottomRightCorner(3, 3) << R;

    T = T.inverse();

    // Compute Jacobian of the current position
    auto jacobian_geo = jacobi_geo_matrix(q_);

    return T * jacobian_geo;
}

Eigen::MatrixXd moore_penrose_inverse(const Eigen::MatrixXd& M_) 
{
    Eigen::MatrixXd M_pinv;
    Eigen::JacobiSVD<Eigen::MatrixXd> svd(M_, Eigen::ComputeFullU | Eigen::ComputeFullV);
    Eigen::JacobiSVD<Eigen::MatrixXd>::SingularValuesType sing_vals = svd.singularValues();
    Eigen::MatrixXd S_ = M_;
    S_.setZero();
    for (auto i = 0; i < sing_vals.size(); i++)
    {
        if ( sing_vals(i) > 0.02 )
            S_(i, i) = 1. / sing_vals(i);
        else
            S_(i, i) = 0;
    }
    M_pinv = Eigen::MatrixXd(svd.matrixV() * S_.transpose() * svd.matrixU().transpose());
    return M_pinv;
}


int main(int argc, char* argv[])
{
    ros::init(argc, argv, "joint_velocity_controller");

    ros::NodeHandle node;

    std::fstream file;

    file.open("/home/daniel/Desktop/catkin_ws/src/joint_velocity_controller/data/velocities.csv", std::fstream::in | std::fstream::out);

    file << "x, y, z, alpha, beta, gamma, zero" << std::endl;

    // Load the Tree from with the URDF file from the parameter server.
	if (tree = new KDL::Tree(); !kdl_parser::treeFromParam("robot_description", *tree)) 
	{
		ROS_ERROR("Failed to construct kdl tree"); 
        return false;
	}

	// Get the chain from base to end of the manipulator.
	if(chain = new KDL::Chain(); !tree->getChain("panda_link0", "panda_link8", *chain))
	{
		ROS_ERROR("Failed to parse the chain from.");
		return false;
	}

    // Default q_state
    Eigen::VectorXd q_cur(JOINTS);
    q_cur << 0, 0, 0, -1.5707, 0, 1.5707, 0;

    Eigen::VectorXd q_end(JOINTS);
    q_end << 2.0, 1.0, 3, -2.5707, 2.5, -1.0707, -2.5;

    for(auto i = 0; i < 1000; i++)
    {
        
        auto jacobi = jacobi_analyt_matrix(q_cur);

        auto jacobi_pinv = moore_penrose_inverse(jacobi);
        
        Eigen::VectorXd qd = jacobi_pinv * x_error_euler(q_cur, q_end) + (Eigen::MatrixXd::Identity(7, 7) - jacobi_pinv * jacobi);

        q_cur = q_cur + 1./100. * qd;
        
        auto x_cur = cartesian_pose(q_cur);

        for(int i = 0; i < 6; i++)
        {
            file << x_cur(i) << ", ";
        }

        file << 0 << std::endl;

    }

    file.close();

    ROS_INFO_STREAM(x_error_euler(q_cur, q_end));

    return 0;
}