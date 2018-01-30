#include "srExt_System.h"

using namespace std;
using namespace Eigen;
using namespace srExt;



srExt::srExtSystem::srExtSystem()
	:srSystem::srSystem()
{
	//	do nothing
}
srExt::srExtSystem::srExtSystem(srSystem * _system)
{
	srSystem::operator=(*_system);
}

srExt::srExtSystem::srExtSystem(const std::string & _fileName, const std::string & _meshDir)
	:srSystem::srSystem()
{
	size_t idx = _fileName.find_last_of(".");
	string file_ext;
	if (idx == string::npos)
		return;

	file_ext = _fileName.substr(idx + 1);
	if (file_ext == "urdf" || file_ext == "URDF")
	{

	}
	else if (file_ext == "sdf" || file_ext == "SDF")
	{
		srSDF::sdf2srSystem(this, _meshDir, _fileName);
	}
	else
	{
		cout << "Unknown type error: " << file_ext << '\t' << "in " << "srExt::srExtSystem::srExtSystem(const std::string & _fileName): " << endl;
	}
}
srExt::srExtSystem::~srExtSystem()
{
	//	dbg
	//cout << "~srExtSystem();" << endl;
	for (auto i : m_link_map)
		delete i.second;
	for (auto i : m_joint_map)
		delete i.second;

	return;
}
void srExt::srExtSystem::fwdKin(const Vec3 & _gravity)
{
	srLink	*link;
	srJoint *pJoint;

	//	position propagation

	KIN_UpdateFrame_All_The_Entity();

	//	velocity propagation

	__FS_LinkVelocityPropagation();


	//	acceleration propagation

	srRevoluteJoint	*rJoint;
	if (GetBaseLinkType() == srSystem::BASELINKTYPE::FIXED)
		m_BaseLink->m_Acc = InvAd(m_BaseLink->m_Frame, se3(Axis(0.0), -_gravity));		//	generate virtual acceleration equivalent to gravity force.

	for (int i = 1; i < m_KIN_Links.get_size(); i++)
	{
		link = m_KIN_Links[i];
		pJoint = link->m_ParentJoint;
		switch (pJoint->GetType())
		{
		case srJoint::JOINTTYPE::REVOLUTE:
			rJoint = static_cast<srRevoluteJoint*>(pJoint);
			link->m_Acc =
				rJoint->m_FS_Screw * rJoint->GetRevoluteJointState().m_rValue[2]
				+ InvAd(link->m_MexpSq, link->m_ParentLink->m_Acc)
				+ ad(link->m_Vel, rJoint->m_FS_Screw*rJoint->GetRevoluteJointState().m_rValue[1]);
			break;

		case srJoint::JOINTTYPE::WELD:
			link->m_Acc = InvAd(link->m_MexpSq, link->m_ParentLink->m_Acc);
			break;

			//	TODO: implement multi-dof joint.

		default:
			break;
		}
	}
}

Eigen::VectorXd srExt::srExtSystem::invKin(srLink * const _endEffector, const SE3 & _T, srLink * _baseLink, double _tol)
{
	//	0 : fixed frame
	//	E : end-effector frame
	//	R : reference frame
	//	G : goal-frame
	//	_T : T_RG
	SE3		goalSE3_EE;								//	T_EG
	se3		currError;								//	se3 error seen by Global frame
	double	errorNorm = 1e+10;						//	Norm of se3 error

	int _iter = 0;
	const unsigned int _max_iter = 100;
	VectorXd	dX(6), dQ, initialGuess;

	if (_baseLink == NULL)
		_baseLink = _endEffector->m_pSystem->GetBaseLink();

	//_endEffector->m_pSystem->BackupInitState();
	for (int trial = 0; trial < 10; trial++)
	{
		errorNorm = 1e+17;
		_iter = 0;

		if (trial == 2)
		{
			initialGuess = VectorXd::Zero(count_dof(_endEffector, _baseLink));
			setJointValue(_endEffector, initialGuess);
		}
		else if (trial > 2)
		{
			std::srand((unsigned int)std::time(0));
			initialGuess = VectorXd::Random(count_dof(_endEffector, _baseLink));
			setJointValue(_endEffector, initialGuess);
		}

		while (errorNorm > _tol)
		{
			if (_iter++ > _max_iter)
				break;
			_endEffector->m_pSystem->KIN_UpdateFrame_All_The_Entity();
			goalSE3_EE = _endEffector->GetFrame() % (_baseLink->GetFrame() * _T);	//	T_EG = T_E0 * T_0R * T_RG
			currError = Log(goalSE3_EE);					//	se3 error seen by EE frame
			errorNorm = sqrt(SquareSum(currError));
			for (int i = 0; i < 6; i++)
				dX[i] = currError[i];
			//dQ = pInv(srExt::Jacobian(srExt::FRAME::BODY, _endEffector, _baseLink)) * dX;
			dQ = Jacobian(REF_FRAME::BODY, _endEffector, _baseLink).jacobiSvd(ComputeThinU | ComputeThinV).solve(dX);
			addJointValue(_endEffector, dQ);
			avoidPosLimit(_endEffector, _baseLink);
		}
		if (errorNorm < _tol)
			return getJointValue(_endEffector, _baseLink);
	}

	cout << "InvKinematics failure (exceed maximum iterations)." << endl;
	return VectorXd::Zero(count_dof(_endEffector, _baseLink));
}

Eigen::VectorXd srExt::srExtSystem::invDyn(const Vec3 & _gravity) const
{
	//	solve forward kinematics first !!
	dse3	F(0.0);
	srLink*		cLink;
	srJoint*	joint;
	srJoint*	cJoint;

	VectorXd	tau(count_dof() + count_joint(srJoint::JOINTTYPE::WELD));
	int qIdx = 0;
	for (int jointIdx = m_KIN_Joints.get_size() - 1; jointIdx >= 0; jointIdx--)
	{
		joint = m_KIN_Joints[jointIdx];
		cLink = joint->m_ChildLink;

		// Reset external force and add Damping (for preventing divergence)
		cLink->m_ExtForce = -cLink->m_Damping*cLink->m_Vel;
		// Add gravity force to external force
		cLink->m_ExtForce += cLink->m_Inertia * InvAd(cLink->m_Frame, _gravity);
		// Add user external force to external force
		cLink->m_ExtForce += cLink->m_UserExtForce;

		F = cLink->m_Inertia * cLink->m_Acc - dad(cLink->m_Vel, cLink->m_Inertia*cLink->m_Vel) - cLink->m_ExtForce;


		//	Add forces from child links
		for (int c = 0; c < cLink->m_ChildJoints.get_size(); c++)
		{
			cJoint = cLink->m_ChildJoints[c];
			F += InvdAd(cJoint->m_ChildLink->FS_GetSE3(), cJoint->m_FS_Force);
		}
		joint->m_FS_Force = F;

		//	project force to screw axis
		double qdot;
		switch (joint->GetType())
		{
		case srJoint::JOINTTYPE::REVOLUTE:
			tau[tau.size() - qIdx - 1] = static_cast<srRevoluteJoint*>(joint)->m_FS_Screw * F;
			static_cast<srRevoluteJoint*>(joint)->m_State.m_rValue[3] = tau[tau.size() - qIdx - 1];
			
#ifdef USE_FRICTION

			//std::cout << "joint name: " << joint->GetName() << std::endl;
			//std::cout << "damper value: " << static_cast<srRevoluteJoint*>(joint)->GetDampingCoeff() << std::endl;
			//static_cast<srRevoluteJoint*>(joint)->SetDampingCoeff(20.0);

			///////////////////////////////// add friction term /////////////////////////////////
			qdot = static_cast<srRevoluteJoint*>(joint)->GetRevoluteJointState().m_rValue[1];
			tau[tau.size() - qIdx - 1] += static_cast<srRevoluteJoint*>(joint)->GetDampingCoeff() * qdot;
			if (qdot >= 0.0)
				tau[tau.size() - qIdx - 1] += static_cast<srRevoluteJoint*>(joint)->GetDampingCoeff();
			else
				tau[tau.size() - qIdx - 1] -= static_cast<srRevoluteJoint*>(joint)->GetDampingCoeff();
			////////////////////////////////////////////////////////////////////////////////////
#endif

			qIdx++;
			break;

		case srJoint::JOINTTYPE::PRISMATIC:
			tau[tau.size() - qIdx - 1] = static_cast<srPrismaticJoint*>(joint)->m_FS_Screw * F;
			static_cast<srPrismaticJoint*>(joint)->m_State.m_rValue[3] = tau[tau.size() - qIdx - 1];
			qIdx++;
			break;

		case srJoint::JOINTTYPE::UNIVERSAL:
			tau[tau.size() - qIdx - 2] = InvAd(static_cast<srUniversalJoint*>(joint)->m_FS_SE3_2, static_cast<srUniversalJoint*>(joint)->m_FS_Screw1) * F;
			tau[tau.size() - qIdx - 1] = static_cast<srUniversalJoint*>(joint)->m_FS_Screw2 * F;
			static_cast<srUniversalJoint*>(joint)->m_State.m_rValue[static_cast<srUniversalJoint*>(joint)->m_Idx_State[0]][3] = tau[tau.size() - qIdx - 2];
			static_cast<srUniversalJoint*>(joint)->m_State.m_rValue[static_cast<srUniversalJoint*>(joint)->m_Idx_State[1]][3] = tau[tau.size() - qIdx - 1];
			qIdx += 2;
			break;

		case srJoint::JOINTTYPE::BALL:
			for (int i = 0; i < 3; i++)
			{
				tau[tau.size() - qIdx - 3 + i] = static_cast<srBallJoint*>(joint)->m_FS_Screw[i] * F;
				static_cast<srBallJoint*>(joint)->m_State.m_Torque[i] = tau[tau.size() - qIdx - 3 + i];
			}
			break;
		case srJoint::JOINTTYPE::WELD:
			tau[tau.size() - qIdx - 1] = 0;
			qIdx++;
			break;
		default:
			break;
		}

	}

	return tau;
}

Eigen::Matrix6Xd srExt::srExtSystem::Jacobian(REF_FRAME _refFrame, srLink * const _EndEffector, srLink * const _baseLink, const SE3 * const _offset) const
{
	srLink* base_link;
	if (_baseLink == NULL)
		base_link = m_BaseLink;
	else
		base_link = _baseLink;

	Matrix6Xd J(6, count_dof(_EndEffector, base_link));	//	6 by n space Jacobian matrix
	srLink * childLink = _EndEffector;			//	
	srJoint * currentJoint = NULL;

	SE3	ref2childLink;		//	frame of child link in reference frame ( body : end effector , spatial : fixed frame )
	se3 temp_se3;
	se3& screw_local = temp_se3;		//	screw axis of joint in child link frame
	se3 screw_global;		//	screw axis of joint in space frame

	int colIdx = 0;
	while (childLink != base_link)
	{
		currentJoint = childLink->m_ParentJoint;
		if (_refFrame == REF_FRAME::BODY && _offset == NULL)
			ref2childLink = _EndEffector->GetFrame() % childLink->GetFrame();
		else if (_refFrame == REF_FRAME::BODY && _offset != NULL)
			ref2childLink = (_EndEffector->GetFrame() * *_offset) % childLink->GetFrame();
		else if (_refFrame == REF_FRAME::SPACE)
			ref2childLink = childLink->GetFrame();

		switch (currentJoint->GetType())
		{
		case srJoint::JOINTTYPE::REVOLUTE:
			screw_local = static_cast<srRevoluteJoint*>(currentJoint)->m_FS_Screw;
			screw_global = Ad(ref2childLink, screw_local);
			for (int i = 0; i < 6; i++)
				J(i, J.cols() - colIdx - 1) = screw_global[i];
			colIdx++;
			break;

		case srJoint::JOINTTYPE::PRISMATIC:
			screw_local = static_cast<srPrismaticJoint*>(currentJoint)->m_FS_Screw;
			screw_global = Ad(ref2childLink, screw_local);
			for (int i = 0; i < 6; i++)
				J(i, J.cols() - colIdx - 1) = screw_global[i];
			colIdx++;
			break;

		case srJoint::JOINTTYPE::UNIVERSAL:
			screw_local = static_cast<srUniversalJoint*>(currentJoint)->m_FS_Screw2;
			screw_global = Ad(ref2childLink, screw_local);
			for (int i = 0; i < 6; i++)
				J(i, J.cols() - colIdx - 1) = screw_global[i];
			colIdx++;

			screw_local = static_cast<srUniversalJoint*>(currentJoint)->m_FS_Screw1;
			screw_global = Ad(ref2childLink, screw_local);
			for (int i = 0; i < 6; i++)
				J(i, J.cols() - colIdx - 1) = screw_global[i];
			colIdx++;
			break;

		case srJoint::JOINTTYPE::BALL:
			for (int j = 2; j >= 0; j--)
			{
				screw_local = static_cast<srBallJoint*>(currentJoint)->m_FS_Screw[j];
				screw_global = Ad(ref2childLink, screw_local);
				for (int i = 0; i < 6; i++)
					J(i, J.cols() - colIdx - 1) = screw_global[i];
				colIdx++;
			}
			break;

		default:
			break;
		}
		childLink = childLink->m_ParentLink;
	}

	return J;
}

Eigen::MatrixXd srExt::srExtSystem::diffInvDyn(const Eigen::MatrixXd & _dqdp, const Eigen::MatrixXd & _dqdotdp, const Eigen::MatrixXd & _dqddotdp)
{
	//	caution : solve inverse dynamics first !!
	int n_joints = m_KIN_Joints.get_size();
	int n_links = m_KIN_Links.get_size();
	int n_p = _dqdp.cols();

	static vector<bool> is_zero_dqdp, is_zero_dqdotdp, is_zero_dqddotdp;
	if (is_zero_dqdp.empty())
	{
		is_zero_dqdp.resize(n_joints);
		is_zero_dqdotdp.resize(n_joints);
		is_zero_dqddotdp.resize(n_joints);
	}
	for (int i = 0; i < n_joints; i++)
	{
		is_zero_dqdp[i] = _dqdp.row(i).isZero();
		is_zero_dqdotdp[i] = _dqdotdp.row(i).isZero();
		is_zero_dqddotdp[i] = _dqddotdp.row(i).isZero();
	}

	//////////////////////////////////////////////////
	// Forward iteration
	//////////////////////////////////////////////////
	srLink	*link;				//	pointer to current link in iteration
	srLink	*p_link, *c_link;		//	pointer to parent and child link of "link".
	srJoint	*p_joint, *c_joint;	//	pointer to parent and child joint of "link"

	srRevoluteJoint*	rJoint;	//	pointer to parent joint, especially for revolute joint.
								//std::map<srLink*, MatrixXd> dVdp, dVdotdp;	//	TODO: performance comparison between map vs unordered_map (vs vector&_array using find)

								// initialization
	linkState* link_state = &m_link_state_map[m_BaseLink];
	linkState* plink_state;
	linkState* clink_state;

	link_state->dVdp = MatrixXd::Zero(6, n_p);
	link_state->dVdotdp = MatrixXd::Zero(6, n_p);

	for (int j = 0; j < m_KIN_Joints.get_size(); j++)
	{
		p_joint = m_KIN_Joints[j];

		link = p_joint->m_ChildLink;
		p_link = link->m_ParentLink;

		link_state = &m_link_state_map[link];
		plink_state = &m_link_state_map[p_link];

		if (p_joint->GetType() == srJoint::JOINTTYPE::REVOLUTE)
		{
			rJoint = static_cast<srRevoluteJoint*>(p_joint);
			const se3& screw = rJoint->m_FS_Screw;
			//	calculate Jacobian of link vel w.r.t. parameter (dVdp).
			link_state->dVdp =
				invAdMat(link->m_MexpSq) * plink_state->dVdp;
			if (!is_zero_dqdotdp[j])
				link_state->dVdp += toEigenVec(screw, 6) * _dqdotdp.row(j);
			if (!is_zero_dqdp[j])
				link_state->dVdp -= toEigenVec(ad(screw, link->m_Vel), 6) * _dqdp.row(j);

			//	calculate Jacobian of link acc w.r.t. parameter (dVdotdp).
			link_state->dVdotdp = invAdMat(link->m_MexpSq) * plink_state->dVdotdp;
			if (!is_zero_dqddotdp[j])
				link_state->dVdotdp += toEigenVec(screw, 6) * _dqddotdp.row(j);
			if (!is_zero_dqdotdp[j])
				link_state->dVdotdp -= toEigenVec(ad(screw, link->m_Vel), 6) * _dqdotdp.row(j);
			if (abs(rJoint->GetRevoluteJointState().m_rValue[1]) > std::numeric_limits<double>::epsilon())
				link_state->dVdotdp -= adMat(screw) * link_state->dVdp * rJoint->GetRevoluteJointState().m_rValue[1];
			if (!is_zero_dqdp[j])
				link_state->dVdotdp -= (adMat(screw) * toEigenVec(InvAd(link->m_MexpSq, p_link->m_Acc), 6) * _dqdp.row(j));
		}
		else if (p_joint->GetType() == srJoint::JOINTTYPE::WELD)
		{
			link_state->dVdp =
				invAdMat(link->m_MexpSq) * plink_state->dVdp;
			link_state->dVdotdp =
				invAdMat(link->m_MexpSq) * plink_state->dVdotdp;
		}
	}

	//////////////////////////////////////////////////
	// Backward iteration
	//////////////////////////////////////////////////

	// variables for backward iteration
	//std::map<srLink*, MatrixXd>	dFdp;
	MatrixXd					dtaudp(n_joints, n_p);			//	return value.
	int							cJointIdx;					//	child joint index in srSystem::m_KIN_joints[].

	static double						inertia_vec[36];
	static const Map<Matrix6d>			G(inertia_vec);
	static dse3							GV_temp;

	//	main loop
	for (int j = n_joints - 1; j >= 0; j--)
	{
		p_joint = m_KIN_Joints[j];
		link = p_joint->m_ChildLink;
		link_state = &m_link_state_map[link];

		link->m_Inertia.ToArray(inertia_vec);
		link_state->dFdp = G * link_state->dVdotdp - adMat(link->m_Vel).transpose() * G * link_state->dVdp;
		link_state->dFdp += link->m_Damping * link_state->dVdp;

		GV_temp = link->m_Inertia * link->m_Vel;

		for (int k = 0; k < n_p; k++)
			link_state->dFdp.col(k) -= dad(link_state->dVdp.col(k), GV_temp);

		for (int c = 0; c < link->m_ChildJoints.get_size(); c++)
		{
			c_joint = link->m_ChildJoints[c];
			c_link = c_joint->m_ChildLink;
			clink_state = &m_link_state_map[c_link];
			if (c_joint->GetType() == srJoint::JOINTTYPE::REVOLUTE)
			{
				cJointIdx = m_KIN_Joints.find(c_joint);
				rJoint = static_cast<srRevoluteJoint*>(c_joint);
				if (is_zero_dqdp[cJointIdx])
					link_state->dFdp +=
					invAdMat(c_link->m_MexpSq).transpose() * clink_state->dFdp;
				else
					link_state->dFdp +=
					invAdMat(c_link->m_MexpSq).transpose()
					* (srExt::dad(-rJoint->m_FS_Screw, rJoint->m_FS_Force) * _dqdp.row(cJointIdx) + clink_state->dFdp);
			}
			else if (c_joint->GetType() == srJoint::JOINTTYPE::WELD)
				link_state->dFdp += invAdMat(c_link->m_MexpSq).transpose() * clink_state->dFdp;
		}

		if (link->m_ParentJoint->GetType() == srJoint::JOINTTYPE::REVOLUTE)
		{
			rJoint = static_cast<srRevoluteJoint*>(link->m_ParentJoint);
			dtaudp.row(j) = toEigenVec(rJoint->m_FS_Screw, 6).transpose() * link_state->dFdp;

#ifdef USE_FRICTION
			///////////////////////////////// add friction term /////////////////////////////////
			dtaudp.row(j) += rJoint->GetDampingCoeff() * _dqdotdp.row(j);
			/////////////////////////////////////////////////////////////////////////////////////
#endif
		}
		else
			dtaudp.row(j).setZero();
	}

	return dtaudp;
}

void srExt::srExtSystem::update_dAccdp(const Eigen::MatrixXd & _dqdp)
{
	int n_joints = m_KIN_Joints.get_size();
	int n_links = m_KIN_Links.get_size();
	int n_p = _dqdp.cols();

	//////////////////////////////////////////////////
	// Forward iteration
	//////////////////////////////////////////////////
	srLink	*link;				//	pointer to current link in iteration
	srLink	*p_link;		//	pointer to parent link of "link".
	srJoint	*p_joint;	//	pointer to parent joint of "link"

	//srRevoluteJoint*	rJoint;	//	pointer to parent joint, especially for revolute joint.

	Matrix3d minusOffset;
	Matrix3d link_R;		//	rotation part of link's global SE3.
	Vector3d vel_lin, vel_ang, acc_lin, acc_ang;

	// initialization
	linkState* link_state = &m_link_state_map[m_BaseLink];
	linkState* plink_state;

	link_state->dAccdp = MatrixXd::Zero(3, n_p);
	link_state->dTdp.resize(n_p);

	for (int i = 0; i < n_p; i++)
		link_state->dTdp[i].setZero();

	for (int j = 0; j < m_KIN_Joints.get_size(); j++)
	{
		p_joint = m_KIN_Joints[j];

		link = p_joint->m_ChildLink;
		p_link = link->m_ParentLink;

		link_state = &m_link_state_map[link];
		plink_state = &m_link_state_map[p_link];

		link_state->dTdp.resize(n_p);

		//	calculate Jacobian of rotation matrix w.r.t. parameter (dRdp).

		for (int i = 0; i < n_p; i++)
			link_state->dTdp[i] = plink_state->dTdp[i] * link->m_MexpSq;
		if (p_joint->GetType() == srJoint::JOINTTYPE::REVOLUTE)
		{
			for (int i = 0; i < n_p; i++)
				if (abs(_dqdp(j, i)) > std::numeric_limits<double>::epsilon() || true)
				{
					link_state->dTdp[i] += link->m_Frame * se3_bracket(static_cast<srRevoluteJoint*>(p_joint)->m_FS_Screw) * _dqdp(j, i);
				}
		}

		////	calculate Jacobian of linear acceleration w.r.t. parameter(dAccdp)

		vel_ang = toEigenVec(link->m_Vel, 3);
		vel_lin = toEigenVec(link->m_Vel, 3, 3);
		acc_ang = toEigenVec(link->m_Acc, 3);
		acc_lin = toEigenVec(link->m_Acc, 3, 3);
		link_R = toEigenMat(link->m_Frame, 3, 3);
		minusOffset = skewSymm(-link->m_Inertia.GetOffset());

		link_state->dAccdp.resize(3, n_p);

		for (int i = 0; i < n_p; i++)
		{
			link_state->dAccdp.col(i) = link_state->dTdp[i].block<3, 3>(0, 0) * (MinusLinearAd(link->GetOffset(), link->m_Acc) - ad(MinusLinearAd(link->GetOffset(), link->m_Vel), link->m_Vel));
			if (!link_state->dVdp.col(i).isZero() || !link_state->dVdotdp.col(i).isZero())
				link_state->dAccdp.col(i) +=
				link_R * (
					minusOffset*link_state->dVdotdp.block<3,1>(0, i) + link_state->dVdotdp.block<3,1>(3, i)
					- skewSymm((minusOffset*link_state->dVdp.block<3, 1>(0, i) + link_state->dVdp.block<3, 1>(3, i))) * vel_ang
					- skewSymm(minusOffset * vel_ang + vel_lin) * link_state->dVdp.block<3,1>(0, i)
					);
		}
	}

	return;
}

int srExt::srExtSystem::count_dof(bool count_weld) const
{
	int dof = 0;

	for (int i = 0; i < m_KIN_Joints.get_size(); i++)
	{
		switch (m_KIN_Joints[i]->GetType())
		{
		case srJoint::JOINTTYPE::REVOLUTE:
		case srJoint::JOINTTYPE::PRISMATIC:
			dof += 1;
			break;
		case srJoint::JOINTTYPE::UNIVERSAL:
			dof += 2;
			break;
		case srJoint::JOINTTYPE::BALL:
			dof += 3;
			break;
		case srJoint::JOINTTYPE::WELD:
			if (count_weld)
				dof += 1;
			break;;	// 0 DOF
		default:
			//	invalid joint type!!
			break;
		}
	}

	return dof;
}

int srExt::srExtSystem::count_dof(srLink * _link, srLink * const _destination) const
{
	if (_link->m_IsBaseLink)
		return 0;
	else
		return count_dof(_link->m_ParentJoint, _destination);
}

int srExt::srExtSystem::count_dof(srJoint * _joint, srLink * const _destination) const
{
	int dof = 0;
	srJoint* currentJoint = _joint;
	while (true)
	{
		switch (currentJoint->GetType())
		{
		case srJoint::JOINTTYPE::REVOLUTE:
		case srJoint::JOINTTYPE::PRISMATIC:
			dof += 1;
			break;
		case srJoint::JOINTTYPE::UNIVERSAL:
			dof += 2;
			break;
		case srJoint::JOINTTYPE::BALL:
			dof += 3;
			break;
		case srJoint::JOINTTYPE::WELD:
			;	// 0 DOF
		default:
			//	invalid joint type!!
			break;
		}

		if (_destination == NULL && currentJoint->m_ParentLink->m_IsBaseLink)
			break;
		else if
			(_destination != NULL && currentJoint->m_ParentLink == _destination)
			break;
		else
			currentJoint = currentJoint->m_ParentLink->m_ParentJoint;
	}

	return dof;
}

int srExt::srExtSystem::count_joint(srJoint::JOINTTYPE _actType) const
{
	int n = 0;
	for (int i = 0; i < m_KIN_Joints.get_size(); i++)
		if (m_KIN_Joints[i]->GetType() == _actType)
			n++;

	return n;
}

void srExt::srExtSystem::setJointValue(srLink * const _endEffector, const Eigen::VectorXd & _q)
{
	int idx = _q.size() - 1;
	srLink* childLink = _endEffector;
	srJoint* currentJoint;

	while (!childLink->m_IsBaseLink)
	{
		currentJoint = childLink->m_ParentJoint;

		switch (currentJoint->GetType())
		{
		case srJoint::JOINTTYPE::REVOLUTE:
			static_cast<srRevoluteJoint*>(currentJoint)->GetRevoluteJointState().m_rValue[0] = _q[idx];
			idx--;
			break;

		case srJoint::JOINTTYPE::PRISMATIC:
			static_cast<srPrismaticJoint*>(currentJoint)->GetPrismaticJointState().m_rValue[0] = _q[idx];
			idx--;
			break;

		case srJoint::JOINTTYPE::UNIVERSAL:
			static_cast<srUniversalJoint*>(currentJoint)->GetUniversalJointState().m_rValue[1][0] = _q[idx];
			static_cast<srUniversalJoint*>(currentJoint)->GetUniversalJointState().m_rValue[0][0] = _q[idx - 1];
			idx -= 2;
			break;

		case srJoint::JOINTTYPE::BALL:
			static_cast<srBallJoint*>(currentJoint)->GetBallJointState().m_SO3Pos = EulerZYX(Vec3(_q[idx], _q[idx - 1], _q[idx - 2])).GetOrientation();
			idx -= 3;
			break;
		default:
			break;
		}
		//adjust2Limit(currentJoint);

		if (idx < 0)
			break;
		childLink = childLink->m_ParentLink;
	}
}

void srExt::srExtSystem::addJointValue(srLink * const _endEffector, const Eigen::VectorXd & _dq)
{
	int idx = _dq.size() - 1;
	srLink* childLink = _endEffector;
	srJoint* currentJoint;
	Vec3	ballJointState;
	while (!childLink->m_IsBaseLink)
	{
		currentJoint = childLink->m_ParentJoint;

		switch (currentJoint->GetType())
		{
		case srJoint::JOINTTYPE::REVOLUTE:
			static_cast<srRevoluteJoint*>(currentJoint)->GetRevoluteJointState().m_rValue[0] += _dq[idx];
			//if (static_cast<srRevoluteJoint*>(currentJoint)->GetRevoluteJointState().m_rValue[0] > SR_PI)
			//	static_cast<srRevoluteJoint*>(currentJoint)->GetRevoluteJointState().m_rValue[0] -= 2 * SR_PI;
			//else if(static_cast<srRevoluteJoint*>(currentJoint)->GetRevoluteJointState().m_rValue[0] < -SR_PI)
			//	static_cast<srRevoluteJoint*>(currentJoint)->GetRevoluteJointState().m_rValue[0] += 2 * SR_PI;

			idx--;
			break;

		case srJoint::JOINTTYPE::PRISMATIC:
			static_cast<srPrismaticJoint*>(currentJoint)->GetPrismaticJointState().m_rValue[0] += _dq[idx];
			idx--;
			break;

		case srJoint::JOINTTYPE::UNIVERSAL:
			static_cast<srUniversalJoint*>(currentJoint)->GetUniversalJointState().m_rValue[1][0] += _dq[idx];
			static_cast<srUniversalJoint*>(currentJoint)->GetUniversalJointState().m_rValue[0][0] += _dq[idx - 1];
			idx -= 2;
			break;

		case srJoint::JOINTTYPE::BALL:
			ballJointState = iEulerZYX(static_cast<srBallJoint*>(currentJoint)->GetBallJointState().m_SO3Pos);
			static_cast<srBallJoint*>(currentJoint)->GetBallJointState().m_SO3Pos = EulerZYX(Vec3(_dq[idx], _dq[idx - 1], _dq[idx - 2]) + ballJointState).GetOrientation();
			idx -= 3;
			break;
		default:
			break;
		}
		//adjust2Limit(currentJoint);
		if (idx < 0)
			break;
		childLink = childLink->m_ParentLink;
	}
}

Eigen::VectorXd srExt::srExtSystem::getJointValue(bool count_weld) const
{
	VectorXd q(count_dof(count_weld));
	srJoint* currentJoint;
	int q_idx = 0;
	Vec3	ballJointState;

	for (int j = 0; j < m_KIN_Joints.get_size(); j++)
	{
		currentJoint = m_KIN_Joints[j];
		switch (currentJoint->GetType())
		{
		case srJoint::JOINTTYPE::REVOLUTE:
			q[q_idx] = static_cast<srRevoluteJoint*>(currentJoint)->GetRevoluteJointState().m_rValue[0];
			q_idx++;
			break;

		case srJoint::JOINTTYPE::PRISMATIC:
			q[q_idx] = static_cast<srPrismaticJoint*>(currentJoint)->GetPrismaticJointState().m_rValue[0];
			q_idx++;
			break;

		case srJoint::JOINTTYPE::UNIVERSAL:
			q[q_idx] = static_cast<srUniversalJoint*>(currentJoint)->GetUniversalJointState().m_rValue[0][0];
			q[q_idx + 1] = static_cast<srUniversalJoint*>(currentJoint)->GetUniversalJointState().m_rValue[1][0];
			q_idx += 2;
			break;

		case srJoint::JOINTTYPE::BALL:
			ballJointState = iEulerZYX(static_cast<srBallJoint*>(currentJoint)->GetBallJointState().m_SO3Pos);
			q[q_idx] = ballJointState[2];
			q[q_idx+1] = ballJointState[1];
			q[q_idx+2] = ballJointState[0];
			q_idx += 3;
			break;
		case srJoint::JOINTTYPE::WELD:
			if (count_weld)
			{
				q[q_idx] = 0;
				q_idx++;
			}
		default:
			break;
		}
	}
	return q;
}

Eigen::VectorXd srExt::srExtSystem::getJointValue(srLink * const _endEffector, srLink * _baseLink) const
{
	if (_baseLink == NULL)
		_baseLink = _endEffector->m_pSystem->GetBaseLink();
	VectorXd q(count_dof(_endEffector, _baseLink));
	int idx = q.size() - 1;
	srLink* childLink = _endEffector;
	srJoint* currentJoint;
	Vec3	ballJointState;
	while (childLink != _baseLink)
	{
		currentJoint = childLink->m_ParentJoint;

		switch (currentJoint->GetType())
		{
		case srJoint::JOINTTYPE::REVOLUTE:
			q[idx] = static_cast<srRevoluteJoint*>(currentJoint)->GetRevoluteJointState().m_rValue[0];
			idx--;
			break;

		case srJoint::JOINTTYPE::PRISMATIC:
			q[idx] = static_cast<srPrismaticJoint*>(currentJoint)->GetPrismaticJointState().m_rValue[0];
			idx--;
			break;

		case srJoint::JOINTTYPE::UNIVERSAL:
			q[idx] = static_cast<srUniversalJoint*>(currentJoint)->GetUniversalJointState().m_rValue[1][0];
			q[idx - 1] = static_cast<srUniversalJoint*>(currentJoint)->GetUniversalJointState().m_rValue[0][0];
			idx -= 2;
			break;

		case srJoint::JOINTTYPE::BALL:
			ballJointState = iEulerZYX(static_cast<srBallJoint*>(currentJoint)->GetBallJointState().m_SO3Pos);
			q[idx] = ballJointState[0];
			q[idx - 1] = ballJointState[1];
			q[idx - 2] = ballJointState[2];
			idx -= 3;
			break;
		case srJoint::JOINTTYPE::WELD:
		default:
			break;
		}
		childLink = childLink->m_ParentLink;
	}
	return q;
}

void srExt::srExtSystem::adjust2Limit(srJoint * const _joint)
{
	switch (_joint->GetType())
	{
	case srJoint::JOINTTYPE::REVOLUTE:
		if (static_cast<srRevoluteJoint*>(_joint)->IsPostionLimited())
		{
			if (static_cast<srRevoluteJoint*>(_joint)->GetRevoluteJointState().m_rValue[0] > static_cast<srRevoluteJoint*>(_joint)->GetPositionUpperLimit()*SR_RADIAN)
				static_cast<srRevoluteJoint*>(_joint)->GetRevoluteJointState().m_rValue[0] = static_cast<srRevoluteJoint*>(_joint)->GetPositionUpperLimit()*SR_RADIAN;
			if (static_cast<srRevoluteJoint*>(_joint)->GetRevoluteJointState().m_rValue[0] < static_cast<srRevoluteJoint*>(_joint)->GetPositionLowerLimit()*SR_RADIAN)
				static_cast<srRevoluteJoint*>(_joint)->GetRevoluteJointState().m_rValue[0] = static_cast<srRevoluteJoint*>(_joint)->GetPositionLowerLimit()*SR_RADIAN;
		}
		break;
	case srJoint::JOINTTYPE::PRISMATIC:
		if (static_cast<srPrismaticJoint*>(_joint)->IsPostionLimited())
		{
			if (static_cast<srPrismaticJoint*>(_joint)->GetPrismaticJointState().m_rValue[0] > static_cast<srPrismaticJoint*>(_joint)->GetPositionUpperLimit())
				static_cast<srPrismaticJoint*>(_joint)->GetPrismaticJointState().m_rValue[0] = static_cast<srPrismaticJoint*>(_joint)->GetPositionUpperLimit();
			if (static_cast<srPrismaticJoint*>(_joint)->GetPrismaticJointState().m_rValue[0] < static_cast<srPrismaticJoint*>(_joint)->GetPositionLowerLimit())
				static_cast<srPrismaticJoint*>(_joint)->GetPrismaticJointState().m_rValue[0] = static_cast<srPrismaticJoint*>(_joint)->GetPositionLowerLimit();
		}
		break;
	case srJoint::JOINTTYPE::UNIVERSAL:
		for (int i = 0; i < 2; i++)
		{
			if (static_cast<srUniversalJoint*>(_joint)->IsPostionLimited(i))
			{
				if (static_cast<srUniversalJoint*>(_joint)->GetUniversalJointState().m_rValue[i][0] > static_cast<srUniversalJoint*>(_joint)->GetPositionUpperLimit(i)*SR_RADIAN)
					static_cast<srUniversalJoint*>(_joint)->GetUniversalJointState().m_rValue[i][0] = static_cast<srUniversalJoint*>(_joint)->GetPositionUpperLimit(i)*SR_RADIAN;
				if (static_cast<srUniversalJoint*>(_joint)->GetUniversalJointState().m_rValue[i][0] < static_cast<srUniversalJoint*>(_joint)->GetPositionLowerLimit(i)*SR_RADIAN)
					static_cast<srUniversalJoint*>(_joint)->GetUniversalJointState().m_rValue[i][0] = static_cast<srUniversalJoint*>(_joint)->GetPositionLowerLimit(i)*SR_RADIAN;
			}
		}
		break;
	case srJoint::JOINTTYPE::BALL:
		break;
	default:
		break;
	}
}

void srExt::srExtSystem::avoidPosLimit(srJoint * const _joint)
{
	switch (_joint->GetType())
	{
	case srJoint::JOINTTYPE::REVOLUTE:
		if (static_cast<srRevoluteJoint*>(_joint)->IsPostionLimited())
		{
			if (static_cast<srRevoluteJoint*>(_joint)->GetRevoluteJointState().m_rValue[0] > static_cast<srRevoluteJoint*>(_joint)->GetPositionUpperLimit()*SR_RADIAN ||
				static_cast<srRevoluteJoint*>(_joint)->GetRevoluteJointState().m_rValue[0] < static_cast<srRevoluteJoint*>(_joint)->GetPositionLowerLimit()*SR_RADIAN)
			{
				double m = (static_cast<srRevoluteJoint*>(_joint)->GetPositionUpperLimit() + static_cast<srRevoluteJoint*>(_joint)->GetPositionLowerLimit())*SR_RADIAN / 2;
				static_cast<srRevoluteJoint*>(_joint)->GetRevoluteJointState().m_rValue[0] -= m;
				static_cast<srRevoluteJoint*>(_joint)->GetRevoluteJointState().m_rValue[0] *= -0.5;
				static_cast<srRevoluteJoint*>(_joint)->GetRevoluteJointState().m_rValue[0] += m;
			}
		}
		break;
	case srJoint::JOINTTYPE::PRISMATIC:
		if (static_cast<srPrismaticJoint*>(_joint)->IsPostionLimited())
		{
			if (static_cast<srPrismaticJoint*>(_joint)->GetPrismaticJointState().m_rValue[0] > static_cast<srPrismaticJoint*>(_joint)->GetPositionUpperLimit() ||
				static_cast<srPrismaticJoint*>(_joint)->GetPrismaticJointState().m_rValue[0] < static_cast<srPrismaticJoint*>(_joint)->GetPositionLowerLimit())
			{
				double m = (static_cast<srPrismaticJoint*>(_joint)->GetPositionUpperLimit() + static_cast<srPrismaticJoint*>(_joint)->GetPositionLowerLimit()) / 2;
				static_cast<srPrismaticJoint*>(_joint)->GetPrismaticJointState().m_rValue[0] -= m;
				static_cast<srPrismaticJoint*>(_joint)->GetPrismaticJointState().m_rValue[0] *= -0.5;
				static_cast<srPrismaticJoint*>(_joint)->GetPrismaticJointState().m_rValue[0] += m;
			}
		}
		break;
	case srJoint::JOINTTYPE::UNIVERSAL:
		for (int i = 0; i < 2; i++)
		{
			if (static_cast<srUniversalJoint*>(_joint)->IsPostionLimited(i))
			{
				if (static_cast<srUniversalJoint*>(_joint)->GetUniversalJointState().m_rValue[i][0] > static_cast<srUniversalJoint*>(_joint)->GetPositionUpperLimit(i)*SR_RADIAN ||
					static_cast<srUniversalJoint*>(_joint)->GetUniversalJointState().m_rValue[i][0] < static_cast<srUniversalJoint*>(_joint)->GetPositionLowerLimit(i)*SR_RADIAN)
				{
					double m = (static_cast<srUniversalJoint*>(_joint)->GetPositionUpperLimit(i) + static_cast<srUniversalJoint*>(_joint)->GetPositionLowerLimit(i))*SR_RADIAN / 2;
					static_cast<srUniversalJoint*>(_joint)->GetUniversalJointState().m_rValue[i][0] -= m;
					static_cast<srUniversalJoint*>(_joint)->GetUniversalJointState().m_rValue[i][0] *= -0.5;
					static_cast<srUniversalJoint*>(_joint)->GetUniversalJointState().m_rValue[i][0] += m;
				}
			}
		}
		break;
	case srJoint::JOINTTYPE::BALL:
		break;
	default:
		break;
	}
}

void srExt::srExtSystem::avoidPosLimit(srLink * const _endEffector, srLink * _baseLink)
{
	if (_baseLink == NULL)
		_baseLink = _endEffector->m_pSystem->GetBaseLink();
	srLink* childLink = _endEffector;

	while (childLink != _baseLink)
	{
		avoidPosLimit(childLink->m_ParentJoint);
		childLink = childLink->m_ParentLink;
	}
}

void srExt::srExtSystem::changeBaseOfSystem(srLink * _baseLink)
{
	// update kinematics
	KIN_ValidateSystem();
	KIN_UpdateFrame_All_The_Entity();


	// make joint list to be reversed
	list<srJoint*> reverseJointList;
	srLink* iterLink = _baseLink;
	while (iterLink != GetBaseLink())
	{
		for (int i = 0; i < m_KIN_Joints.get_size(); i++)
		{
			if (m_KIN_Joints[i]->m_ChildLink == iterLink)
			{
				iterLink = m_KIN_Joints[i]->m_ParentLink;
				reverseJointList.push_back(m_KIN_Joints[i]);
				break;
			}
		}
	}

	// reverse joints needed
	srLink* tmpLink;
	SE3 tmpSE3;
	for (list<srJoint*>::iterator iter = reverseJointList.begin(); iter != reverseJointList.end(); iter++)
	{
		(*iter)->m_ParentLink->m_ChildJoints.find_pop((*iter));

		tmpLink = (*iter)->m_ParentLink;
		tmpSE3 = (*iter)->m_ParentLinkToJoint;

		(*iter)->m_ParentLink = (*iter)->m_ChildLink;
		(*iter)->m_ParentLinkToJoint = (*iter)->m_ChildLinkToJoint * RotX(SR_PI);

		(*iter)->m_ChildLink = tmpLink;
		(*iter)->m_ChildLinkToJoint = tmpSE3 * RotX(SR_PI);

		(*iter)->m_ParentLink->m_ChildJoints.add_tail((*iter));
	}


	// change base link
	_baseLink->SetFrame(SE3());
	SetBaseLink(_baseLink);
	SetBaseLinkType(srSystem::FIXED);
	KIN_ValidateSystem();
}

void srExt::srExtSystem::setSupportPolygon(Eigen::VectorXd _supPolyBoundary)
{
	m_supPolyBoundary = _supPolyBoundary;
}

void srExt::srExtSystem::update_map()
{
	m_joint_name_idx_map.clear();
	for (int i = 0; i < m_KIN_Joints.get_size(); i++)
	{
		m_joint_name_idx_map[m_KIN_Joints[i]->GetName()] = i;
		//m_joint_name_idx_map.insert(std::make_pair(m_KIN_Joints[i]->GetName(), i));
	}
}

void srExt::srExtSystem::KIN_ValidateSystem()
{
	srSystem::KIN_ValidateSystem();
	update_map();
}