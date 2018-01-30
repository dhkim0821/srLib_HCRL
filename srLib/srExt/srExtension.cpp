#include "srExtension.h"
using namespace srExt;
using namespace Eigen;

int srExt::count_dof(srJoint * _joint, srLink * const _destination)
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

int srExt::count_dof(srLink * _link, srLink * const _destination)
{
	if (_link->m_IsBaseLink)
		return 0;
	else
		return count_dof(_link->m_ParentJoint, _destination);
}

int srExt::count_dof(srSystem * _system)
{
	int dof = 0;

	for (int i = 0; i < _system->m_KIN_Joints.get_size(); i++)
	{
		switch (_system->m_KIN_Joints[i]->GetType())
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
	}

	return dof;
}

int srExt::count_joint(srSystem * _system, srJoint::JOINTTYPE _actType)
{
	int n = 0;
	for (int i = 0; i < _system->m_KIN_Joints.get_size(); i++)
		if (_system->m_KIN_Joints[i]->GetType() == _actType)
			n++;

	return n;
}


Matrix6Xd srExt::Jacobian(FRAME _refFrame, srLink * const _EndEffector, srLink * const _baseLink, const SE3 * const _offset)
{
	Matrix6Xd J(6, srExt::count_dof(_EndEffector, _baseLink));	//	6 by n space Jacobian matrix
	srLink * childLink = _EndEffector;			//	
	srJoint * currentJoint = NULL;

	SE3	ref2childLink;		//	frame of child link in reference frame ( body : end effector , spatial : fixed frame )
	se3 temp_se3;
	se3& screw_local = temp_se3;		//	screw axis of joint in child link frame
	se3 screw_global;		//	screw axis of joint in space frame

	int colIdx = 0;
	while (!childLink->m_IsBaseLink)
	{
		currentJoint = childLink->m_ParentJoint;
		if (_refFrame == srExt::FRAME::BODY && _offset == NULL)
			ref2childLink = _EndEffector->GetFrame() % childLink->GetFrame();
		else if (_refFrame == srExt::FRAME::BODY && _offset != NULL)
			ref2childLink = (_EndEffector->GetFrame() * *_offset) % childLink->GetFrame();
		else if (_refFrame == srExt::FRAME::SPACE)
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

void srExt::fwdKin(srSystem* _system, const Vec3& _gravity)
{
	srLink	*link;
	srJoint *pJoint;

	//	position propagation

	_system->KIN_UpdateFrame_All_The_Entity();

	//	velocity propagation

	_system->__FS_LinkVelocityPropagation();


	//	acceleration propagation

	srRevoluteJoint	*rJoint;
	if (_system->GetBaseLinkType() == srSystem::BASELINKTYPE::FIXED)
		_system->m_BaseLink->m_Acc = InvAd(_system->m_BaseLink->m_Frame, se3(Axis(0.0), -_gravity));		//	generate virtual acceleration equivalent to gravity force.

	for (int i = 1; i < _system->m_KIN_Links.get_size(); i++)
	{
		link = _system->m_KIN_Links[i];
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



VectorXd srExt::invKin(srLink * const _endEffector, const SE3& _T, srLink * _baseLink, double _tol)
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
			srExt::setJointValue(_endEffector, initialGuess);
		}
		else if (trial > 2)
		{
			std::srand((unsigned int)std::time(0));
			initialGuess = VectorXd::Random(count_dof(_endEffector, _baseLink));
			srExt::setJointValue(_endEffector, initialGuess);
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
			dQ = srExt::Jacobian(srExt::FRAME::BODY, _endEffector, _baseLink).jacobiSvd(ComputeThinU | ComputeThinV).solve(dX);
			addJointValue(_endEffector, dQ);
			avoidPosLimit(_endEffector, _baseLink);
		}
		if (errorNorm < _tol)
			return getJointValue(_endEffector, _baseLink);
	}

	cout << "Kibo::invKinematics failure (exceed maximum iterations)." << endl;
	return VectorXd::Zero(count_dof(_endEffector, _baseLink));
}


Eigen::VectorXd srExt::invDyn(srSystem * _system, const Vec3 & _gravity)
{
	//	solve forward kinematics first !!
	dse3	F(0.0);
	srLink*		cLink;
	srJoint*	joint;
	srJoint*	cJoint;

	VectorXd	tau(count_dof(_system) + count_joint(_system, srJoint::JOINTTYPE::WELD));
	int qIdx = 0;
	for (int jointIdx = _system->m_KIN_Joints.get_size() - 1; jointIdx >= 0; jointIdx--)
	{
		joint = _system->m_KIN_Joints[jointIdx];
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
		double v;
		switch (joint->GetType())
		{
		case srJoint::JOINTTYPE::REVOLUTE:
			tau[tau.size() - qIdx - 1] = static_cast<srRevoluteJoint*>(joint)->m_FS_Screw * F;
			static_cast<srRevoluteJoint*>(joint)->m_State.m_rValue[3] = tau[tau.size() - qIdx - 1];
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


MatrixXd srExt::diffInvDyn(srSystem* _system, const Eigen::MatrixXd& _dqdp, const Eigen::MatrixXd& _dqdotdp, const Eigen::MatrixXd& _dqddotdp)
{
	//	caution : solve inverse dynamics first !!
	int n_joints = _system->m_KIN_Joints.get_size();		//	TODO: how to deal with weld joint.
	//int n_joints = count_dof(_system);		//	TODO: how to deal with weld joint.
	int n_links = _system->m_KIN_Links.get_size();
	int n_p = _dqdp.cols();

	//////////////////////////////////////////////////
	// Forward iteration
	//////////////////////////////////////////////////
	srLink	*link;				//	pointer to current link in iteration
	srLink	*pLink, *cLink;		//	pointer to parent and child link of "link".
	srJoint	*pJoint, *cJoint;	//	pointer to parent and child joint of "link"

	srRevoluteJoint*	rJoint;	//	pointer to parent joint, especially for revolute joint.
								//	int		rJointIdx = 0;		//	index only for revolute joint (excluding WELD).
	std::map<srLink*, MatrixXd> dVdp, dVdotdp;	//	TODO: performance comparison between map vs unordered_map (vs vector&_array using find)

	// initialization
	dVdp[_system->m_BaseLink] = MatrixXd::Zero(6, n_p);
	dVdotdp[_system->m_BaseLink] = MatrixXd::Zero(6, n_p);

	for (int j = 0; j < _system->m_KIN_Joints.get_size(); j++)
	{
		pJoint = _system->m_KIN_Joints[j];
		link = pJoint->m_ChildLink;
		pLink = link->m_ParentLink;

		if (pJoint->GetType() == srJoint::JOINTTYPE::REVOLUTE)
		{
			rJoint = static_cast<srRevoluteJoint*>(pJoint);
			const se3& screw = rJoint->m_FS_Screw;
			//	calculate Jacobian of link vel w.r.t. parameter (dVdp).
			dVdp[link] =
				invAdMat(link->m_MexpSq) * dVdp[pLink]
				+ toEigenVec(screw, 6) * _dqdotdp.row(j)
				- toEigenVec(ad(screw, link->m_Vel), 6) * _dqdp.row(j);
			//	calculate Jacobian of link acc w.r.t. parameter (dVdotdp).
			dVdotdp[link] =
				invAdMat(link->m_MexpSq) * dVdotdp[pLink]
				+ toEigenVec(screw, 6) * _dqddotdp.row(j)
				- toEigenVec(ad(screw, link->m_Vel), 6) * _dqdotdp.row(j)
				- adMat(screw) * dVdp[link] * rJoint->GetRevoluteJointState().m_rValue[1]
				- (adMat(screw) * toEigenVec(InvAd(link->m_MexpSq, pLink->m_Acc), 6) * _dqdp.row(j));
		}
		else if (pJoint->GetType() == srJoint::JOINTTYPE::WELD)
		{
			dVdp[link] =
				invAdMat(link->m_MexpSq) * dVdp[pLink];
			dVdotdp[link] =
				invAdMat(link->m_MexpSq) * dVdotdp[pLink];
		}
	}

	//////////////////////////////////////////////////
	// Backward iteration
	//////////////////////////////////////////////////

	// variables for backward iteration
	std::map<srLink*, MatrixXd>	dFdp;
	MatrixXd					dtaudp(n_joints, n_p);			//	return value.
	int							cJointIdx;					//	child joint index in srSystem::m_KIN_joints[].

	double						inertia_vec[36];
	const Map<Matrix6d>			G(inertia_vec);
	dse3						GV_temp;

	//	main loop
	for (int j = n_joints - 1; j >= 0; j--)
	{
		pJoint = _system->m_KIN_Joints[j];
		link = pJoint->m_ChildLink;
		link->m_Inertia.ToArray(inertia_vec);
		dFdp[link] = G * dVdotdp[link] - adMat(link->m_Vel).transpose() * G * dVdp[link];
		dFdp[link] += link->m_Damping * dVdp[link];
		GV_temp = link->m_Inertia * link->m_Vel;

		for (int k = 0; k < n_p; k++)
			dFdp[link].col(k) -= dad(dVdp[link].col(k), GV_temp);

		for (int c = 0; c < link->m_ChildJoints.get_size(); c++)
		{
			cJoint = link->m_ChildJoints[c];
			cLink = cJoint->m_ChildLink;
			if (cJoint->GetType() == srJoint::JOINTTYPE::REVOLUTE)
			{
				cJointIdx = _system->m_KIN_Joints.find(cJoint);

				//int dbg1 = dFdp[link].rows();
				//int dbg2 = dFdp[link].cols();
				//auto dbg3 = invAdMat(cLink->m_MexpSq).transpose()
				//	* (srExt::dad(rJoint->m_FS_Screw, rJoint->m_FS_Force) * _dqdp.row(cJointIdx));
				//dbg1 = dbg3.rows();
				//dbg2 = dbg3.cols();

				rJoint = static_cast<srRevoluteJoint*>(cJoint);
				dFdp[link] +=
					invAdMat(cLink->m_MexpSq).transpose()
					* (srExt::dad(-rJoint->m_FS_Screw, rJoint->m_FS_Force) * _dqdp.row(cJointIdx) + dFdp[cLink]);
			}
			else if (cJoint->GetType() == srJoint::JOINTTYPE::WELD)
				dFdp[link] += invAdMat(cLink->m_MexpSq).transpose() * dFdp[cLink];
		}

		if (link->m_ParentJoint->GetType() == srJoint::JOINTTYPE::REVOLUTE)
		{
			rJoint = static_cast<srRevoluteJoint*>(link->m_ParentJoint);
			dtaudp.row(j) = toEigenVec(rJoint->m_FS_Screw, 6).transpose() * dFdp[link];
			//cout << link->GetName() << endl;
			//if(dtaudp.row(j).isZero())
			//	cout << link->GetName() << '\t';
		}
		else
			dtaudp.row(j).setZero();
	}

	return dtaudp;
}

std::unordered_map<srLink*, linkState>	srExt::_linkStateMap;
Eigen::MatrixXd srExt::diffInvDyn_fast(srSystem * _system, const Eigen::MatrixXd & _dqdp, const Eigen::MatrixXd & _dqdotdp, const Eigen::MatrixXd & _dqddotdp)
{
	//static int cnt = 0;
	//cout << "diff inv dyn: " << cnt++ << endl;


	//	caution : solve inverse dynamics first !!
	int n_joints = _system->m_KIN_Joints.get_size();
	int n_links = _system->m_KIN_Links.get_size();
	int n_p = _dqdp.cols();

	//////////////////////////////////////////////////
	// Forward iteration
	//////////////////////////////////////////////////
	srLink	*link;				//	pointer to current link in iteration
	srLink	*p_link, *c_link;		//	pointer to parent and child link of "link".
	srJoint	*p_joint, *c_joint;	//	pointer to parent and child joint of "link"

	srRevoluteJoint*	rJoint;	//	pointer to parent joint, especially for revolute joint.
	//std::map<srLink*, MatrixXd> dVdp, dVdotdp;	//	TODO: performance comparison between map vs unordered_map (vs vector&_array using find)

	// initialization
	linkState* link_state = &_linkStateMap[_system->m_BaseLink];
	linkState* plink_state;
	linkState* clink_state;

	link_state->dVdp = MatrixXd::Zero(6, n_p);
	link_state->dVdotdp = MatrixXd::Zero(6, n_p);

	for (int j = 0; j < _system->m_KIN_Joints.get_size(); j++)
	{
		p_joint = _system->m_KIN_Joints[j];

		link = p_joint->m_ChildLink;
		p_link = link->m_ParentLink;

		link_state = &_linkStateMap[link];
		plink_state = &_linkStateMap[p_link];

		if (p_joint->GetType() == srJoint::JOINTTYPE::REVOLUTE)
		{
			rJoint = static_cast<srRevoluteJoint*>(p_joint);
			const se3& screw = rJoint->m_FS_Screw;
			//	calculate Jacobian of link vel w.r.t. parameter (dVdp).
			link_state->dVdp =
				invAdMat(link->m_MexpSq) * plink_state->dVdp
				+ toEigenVec(screw, 6) * _dqdotdp.row(j)
				- toEigenVec(ad(screw, link->m_Vel), 6) * _dqdp.row(j);

			//	calculate Jacobian of link acc w.r.t. parameter (dVdotdp).
			link_state->dVdotdp =
				invAdMat(link->m_MexpSq) * plink_state->dVdotdp
				+ toEigenVec(screw, 6) * _dqddotdp.row(j)
				- toEigenVec(ad(screw, link->m_Vel), 6) * _dqdotdp.row(j)
				- adMat(screw) * link_state->dVdp * rJoint->GetRevoluteJointState().m_rValue[1]
				- (adMat(screw) * toEigenVec(InvAd(link->m_MexpSq, p_link->m_Acc), 6) * _dqdp.row(j));
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
		p_joint = _system->m_KIN_Joints[j];
		link = p_joint->m_ChildLink;
		link_state = &_linkStateMap[link];

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
			clink_state = &_linkStateMap[c_link];
			if (c_joint->GetType() == srJoint::JOINTTYPE::REVOLUTE)
			{
				cJointIdx = _system->m_KIN_Joints.find(c_joint);
				rJoint = static_cast<srRevoluteJoint*>(c_joint);
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
		}
		else
			dtaudp.row(j).setZero();
	}

	return dtaudp;
}

Eigen::MatrixXd srExt::diffInvDyn_fast2(srSystem * _system, const Eigen::MatrixXd & _dqdp, const Eigen::MatrixXd & _dqdotdp, const Eigen::MatrixXd & _dqddotdp)
{
	//	caution : solve inverse dynamics first !!
	int n_joints = _system->m_KIN_Joints.get_size();
	int n_links = _system->m_KIN_Links.get_size();
	int n_p = _dqdp.cols();

	static vector<bool> is_zero_dqdp, is_zero_dqdotdp, is_zero_dqddotdp;
	if (is_zero_dqdp.empty())
	{
		is_zero_dqdp.resize(n_joints);
		is_zero_dqdotdp.resize(n_joints);
		is_zero_dqddotdp.resize(n_joints);
		for (int i = 0; i < n_joints; i++)
		{
			is_zero_dqdp[i] = _dqdp.row(i).isZero();
			is_zero_dqdotdp[i] = _dqdotdp.row(i).isZero();
			is_zero_dqddotdp[i] = _dqddotdp.row(i).isZero();
		}
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
	linkState* link_state = &_linkStateMap[_system->m_BaseLink];
	linkState* plink_state;
	linkState* clink_state;

	link_state->dVdp = MatrixXd::Zero(6, n_p);
	link_state->dVdotdp = MatrixXd::Zero(6, n_p);

	for (int j = 0; j < _system->m_KIN_Joints.get_size(); j++)
	{
		p_joint = _system->m_KIN_Joints[j];

		link = p_joint->m_ChildLink;
		p_link = link->m_ParentLink;

		link_state = &_linkStateMap[link];
		plink_state = &_linkStateMap[p_link];

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
		p_joint = _system->m_KIN_Joints[j];
		link = p_joint->m_ChildLink;
		link_state = &_linkStateMap[link];

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
			clink_state = &_linkStateMap[c_link];
			if (c_joint->GetType() == srJoint::JOINTTYPE::REVOLUTE)
			{
				cJointIdx = _system->m_KIN_Joints.find(c_joint);
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
		}
		else
			dtaudp.row(j).setZero();
	}

	return dtaudp;
}



void srExt::setJointValue(srLink * const _endEffector, const VectorXd & _q)
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

void srExt::addJointValue(srLink * const _endEffector, const VectorXd & _dq)
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

VectorXd srExt::getJointValue(srLink * const _endEffector, srLink * _baseLink)
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

void srExt::adjust2Limit(srJoint * const _joint)
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

void srExt::avoidPosLimit(srJoint * const _joint)
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

void srExt::avoidPosLimit(srLink * const _endEffector, srLink * _baseLink)
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

void srExt::changeBaseOfSystem(srSystem * _system, srLink * _baseLink)
{
	// update kinematics
	_system->KIN_ValidateSystem();
	_system->KIN_UpdateFrame_All_The_Entity();


	// make joint list to be reversed
	list<srJoint*> reverseJointList;
	srLink* iterLink = _baseLink;
	while (iterLink != _system->GetBaseLink())
	{
		for (int i = 0; i < _system->m_KIN_Joints.get_size(); i++)
		{
			if (_system->m_KIN_Joints[i]->m_ChildLink == iterLink)
			{
				iterLink = _system->m_KIN_Joints[i]->m_ParentLink;
				reverseJointList.push_back(_system->m_KIN_Joints[i]);
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
		(*iter)->m_ParentLinkToJoint = (*iter)->m_ChildLinkToJoint;

		(*iter)->m_ChildLink = tmpLink;
		(*iter)->m_ChildLinkToJoint = tmpSE3;

		(*iter)->m_ParentLink->m_ChildJoints.add_tail((*iter));
	}


	// change base link
	_baseLink->SetFrame(SE3());
	_system->SetBaseLink(_baseLink);
	_system->SetBaseLinkType(srSystem::FIXED);
}

srSystem * srExt::makeBoxObject(Eigen::VectorXd pos, Eigen::VectorXd dim, bool invisible, bool expand)
{
	srSystem * system = new srSystem();

	srLink * link = new srLink();
	srCollision * coll = new srCollision();


	link->GetGeomInfo().SetShape(srGeometryInfo::BOX);
	if (invisible)
		link->GetGeomInfo().SetDimension(Vec3(0.001));
	else
		link->GetGeomInfo().SetDimension(dim(0), dim(1), dim(2));
	link->GetGeomInfo().SetColor(0, 0, 0);
	link->AddCollision(coll);

	coll->GetGeomInfo().SetLocalFrame(SE3());
	coll->GetGeomInfo().SetShape(srGeometryInfo::BOX);
	if(expand)
		coll->GetGeomInfo().SetDimension(dim(0)*1.5, dim(1)*1.5, dim(2)*1.5);
	else
		coll->GetGeomInfo().SetDimension(dim(0), dim(1), dim(2));

	link->SetFrame(EulerZYX(Vec3(0.0), Vec3(pos(0), pos(1), pos(2))));

	system->SetBaseLink(link);
	system->SetBaseLinkType(srSystem::FIXED);
	system->SetSelfCollision(false);

	return system;
}
