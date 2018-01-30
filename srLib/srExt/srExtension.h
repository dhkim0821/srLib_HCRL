#pragma once
/*!*****************************************************************************
[ SNU Robotics Extension Library ]

author		: Jisoo Hong
dependency	: Eigen 3
version		: 20161220
*******************************************************************************/
#include <ctime>
#include <vector>
#include <Eigen/Core>
#include <Eigen/SVD>
#include <Eigen/Eigenvalues>
#include <srDyn/srSpace.h>
#include <list>
#include <map>
#include <unordered_map>

#include "srExt_Eigen.h"
#include "srExt_LieGroup.h"

namespace srExt
{
	enum FRAME
	{
		SPACE,
		BODY
	};


	//	count total DOF from _joint to base or destination link (include joint it self)
	int count_dof(srJoint* _joint, srLink* const _destination = NULL);
	int count_dof(srLink* _link, srLink* const _destination = NULL);
	int	count_dof(srSystem* _system);
	int	count_joint(srSystem* _system, srJoint::JOINTTYPE _actType);

	Eigen::Matrix6Xd	Jacobian(FRAME _refFrame, srLink * const _EndEffector, srLink * const _baseLink = NULL, const SE3 * const _offset = NULL);

	/*
	Forward kinematice routine. User should specify position, velocity and acceleration of each joints and base link before run this routine.
	(gravity input generate virtual acceleration equivalent to gravity force.)
	*/
	void				fwdKin(srSystem* _system, const Vec3& _gravity = Vec3(0.0));
	
	/*
	Inverse kinematics routine.
	*/
	Eigen::VectorXd		invKin(srLink* const _endEffector, const SE3& _T, srLink* _baseLink = NULL, double _tol = 1e-7);

	/*
	Calculate torque of each joint.	Assume that pos, vel and acc of each joint are already set up (see "fwdKin()").
	Don't specify gravity if you apply it in "fwdKin".
	*/
	Eigen::VectorXd		invDyn(srSystem* _system, const Vec3& _gravity = Vec3(0.0));

	/*
	Calculate Jacobian of torque(result of inv dyn) w.r.t. state parameter p.
	caution : only valid for system which consists of revolute and weld joint.
	*/
	Eigen::MatrixXd		diffInvDyn(srSystem* _system, const Eigen::MatrixXd& _dqdp, const Eigen::MatrixXd& _dqdotdp, const Eigen::MatrixXd& _dqddotdp);
	Eigen::MatrixXd		diffInvDyn_fast(srSystem* _system, const Eigen::MatrixXd& _dqdp, const Eigen::MatrixXd& _dqdotdp, const Eigen::MatrixXd& _dqddotdp);
	Eigen::MatrixXd		diffInvDyn_fast2(srSystem* _system, const Eigen::MatrixXd& _dqdp, const Eigen::MatrixXd& _dqdotdp, const Eigen::MatrixXd& _dqddotdp);

	void				setJointValue(srLink* const _endEffector, const Eigen::VectorXd& _q);
	void				addJointValue(srLink* const _endEffector, const Eigen::VectorXd& _dq);
	Eigen::VectorXd		getJointValue(srLink* const _endEffector, srLink* _baseLink = NULL);

	void				adjust2Limit(srJoint* const _joint);
	void				avoidPosLimit(srJoint* const _joint);
	void				avoidPosLimit(srLink* const _endEffector, srLink* _baseLink = NULL);

	void				changeBaseOfSystem(srSystem* _system, srLink* _baseLink);

	struct linkState
	{
	public:
		Eigen::MatrixXd dVdp, dVdotdp, dFdp;
	};

	extern std::unordered_map<srLink*, linkState>	_linkStateMap;


	srSystem* makeBoxObject(Eigen::VectorXd pos, Eigen::VectorXd dim, bool invisible = false, bool expand = false);
}