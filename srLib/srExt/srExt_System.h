#pragma once

/*!*****************************************************************************
[ SNU Robotics Extension System ]

author		: Jisoo Hong
dependency	: STL, Eigen 3
version		: 20161229
*******************************************************************************/

#include <iostream>
#include <vector>
#include <list>
#include <string>
#include <map>
//#include <pair>
#include <unordered_map>
#include <ctime>

#include <srDyn/srSpace.h>
#include <srLib/common/srSDF.h>
#include "srExt_Eigen.h"
#include "srExt_LieGroup.h"

#define USE_FRICTION

namespace srExt
{
	class srExtSystem : public srSystem
    {
    public:
		enum REF_FRAME
		{
			SPACE,
			BODY
		};

		/*
		Constructors.
		*/
		srExtSystem();
		srExtSystem(srSystem* _system);
		srExtSystem(const std::string& _fileName, const std::string& _meshDir = std::string());
		
		/*
		Destructors
		*/
		~srExtSystem();

		/*
		Forward kinematice routine. User should specify position, velocity and acceleration of each joints and base link before run this routine.
		(gravity input generate virtual acceleration equivalent to gravity force.)
		*/
		void				fwdKin(const Vec3& _gravity = Vec3(0.0));

		/*
		Inverse kinematics routine.
		*/
		Eigen::VectorXd		invKin(srLink* const _endEffector, const SE3& _T, srLink* _baseLink = NULL, double _tol = 1e-7);

		/*
		Calculate torque of each joint.	Assume that pos, vel and acc of each joint are already set up (see "fwdKin()").
		Don't specify gravity if you apply it in "fwdKin".
		*/
		Eigen::VectorXd		invDyn(const Vec3& _gravity = Vec3(0.0)) const;

		/*
		End-effector Jacobian w.r.t. joint state
		*/
		Eigen::Matrix6Xd	Jacobian(REF_FRAME _refFrame, srLink * const _EndEffector, srLink * const _baseLink = NULL, const SE3 * const _offset = NULL) const;

		/*
		Calculate Jacobian of torque(result of inv dyn) w.r.t. state parameter p.
		caution : only valid for system which consists of revolute and weld joint.
		*/
		Eigen::MatrixXd		diffInvDyn(const Eigen::MatrixXd& _dqdp, const Eigen::MatrixXd& _dqdotdp, const Eigen::MatrixXd& _dqddotdp);


		/*
		Calculate Jacobian of linear acceleration (see  srLink::GetLinearAcc) w.r.t. state parameter p.
		caution: "fwdKin" must be called before calling this function.
		*/
		void	update_dAccdp(const Eigen::MatrixXd& _dqdp);
		
		
		/*
		Utility functions
		*/
		int	count_dof(bool count_weld = false) const;
		int count_dof(srLink* _link, srLink* const _destination = NULL) const;
		int count_dof(srJoint* _joint, srLink* const _destination = NULL) const;
		int	count_joint(srJoint::JOINTTYPE _actType) const;

		void				setJointValue(srLink* const _endEffector, const Eigen::VectorXd& _q);
		void				addJointValue(srLink* const _endEffector, const Eigen::VectorXd& _dq);
		Eigen::VectorXd		getJointValue(bool count_weld = false) const;
		Eigen::VectorXd		getJointValue(srLink* const _endEffector, srLink* _baseLink = NULL) const;

		void				adjust2Limit(srJoint* const _joint);
		void				avoidPosLimit(srJoint* const _joint);
		void				avoidPosLimit(srLink* const _endEffector, srLink* _baseLink = NULL);

		void				changeBaseOfSystem(srLink* _baseLink);
		
		void				setSupportPolygon(Eigen::VectorXd _supPolyBoundary);

		void				update_map();

		void				KIN_ValidateSystem();
		/*
		Map for link ptr and joint ptr.
		*/
		std::map<std::string, srLink*>	m_link_map;
		std::map<std::string, srJoint*>	m_joint_map;

		/*
		Map for finding index
		*/
		std::map<std::string, int>	m_joint_name_idx_map;


		/*
		Variables for intermediate values.
		*/
		struct linkState
		{
		public:
			Eigen::MatrixXd dVdp, dVdotdp, dFdp, dAccdp;
		
			//	Differentiation of rotation part of link frame w.r.t. state parameter p.
			//std::vector<Eigen::Matrix3d, Eigen::aligned_allocator<Eigen::Matrix3d>>	dRdp;

			//	Differentiation of link frame w.r.t. state parameter p.
			std::vector<Eigen::Matrix4d, Eigen::aligned_allocator<Eigen::Matrix4d>>	dTdp;
		};
		
		std::map<srLink*, linkState>	m_link_state_map;

		/*
		Varibles for ZMP constraints
		*/
		Eigen::VectorXd	m_supPolyBoundary;

	public:
		EIGEN_MAKE_ALIGNED_OPERATOR_NEW
	};
}
