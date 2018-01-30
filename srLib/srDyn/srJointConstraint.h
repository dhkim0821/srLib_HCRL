#pragma once

#include "srDyn/srConstraint.h"
#include "srDyn/srState.h"
#include "srDyn/srSystem.h"
#include "srDyn/srJoint.h"

//**********************************************************************//
// JointConstraint 
class JointConstraint: public Constraint
{
public:
	JointConstraint()
	{
		nd = 1;
		type1 = false;
	};

	// static memebers
	static void SetErp(SR_REAL _erp);
	static SR_REAL erp_jointpositionlimit;

	static void SetAllowedPenetration(SR_REAL _allowedpenetration);
	static SR_REAL allowedjointerror;

	static void SetBouncingThreshold(SR_REAL _bouncingthreshold);
	static SR_REAL bouncing_threshold;

	static void SetMaximumErpVelocity(SR_REAL _maximum_erp_velocity);
	static SR_REAL maximum_erp_velocity;

	static void	SetMaximumBouncingVelocity(SR_REAL _maximum_bouncing_velocity);
	static SR_REAL maximum_bouncing_velocity;

	//variables

	//=== PRESTEP ===//
	srJoint *	pJoint;
	srSystem *	pSystem;
	srJoint::ACTTYPE		actuationtype;
	srJoint::JOINTTYPE	jointtype;

	//-- Target JointState
	srRevoluteState  *	m_pRstate;
	srPrismaticState *	m_pPstate;
	srUniversalState *	m_pUstate;
	//srBallState		*	m_pBstate;


	//-- Limit
	SR_REAL	Limit[2];		// Position Limit  [0]:lower, [1]:upper
	SR_REAL	ForceLimit[2];	// Force Limit [0]:lower, [1]:upper


	//=== RUNTIME ===//
	SR_REAL	LimitError;
	SR_REAL	Negative_Velocity;

	bool	bActive;
	int		lifetime;
	SR_REAL	lambda;


	//-- Detection
	bool	Inspect_JointState();
	bool	(JointConstraint::*m_pfn_inspect_jointstate)();
	bool	_inspect_R_PositionLimit();
	bool	_inspect_R_TorqueLimit();

	bool	_inspect_P_PositionLimit();
	bool	_inspect_P_TorqueLimit();

	bool	_inspect_U1_PositionLimit();
	bool	_inspect_U1_TorqueLimit();
	bool	_inspect_U2_PositionLimit();
	bool	_inspect_U2_TorqueLimit();

	bool	_inspect_B_Yaw();
	bool	_inspect_B_RollPitch();

	// virtual function
	void	GetInformation(ConstraintInfo * info);
	void	(JointConstraint::*m_pfn_getInformation)(ConstraintInfo * info);
	void	_getInformation_PositionLimit(ConstraintInfo * info);
	void	_getInformation_TorqueLimit(ConstraintInfo * info);


	void	ApplyImpulse(int _idx);
	void	(JointConstraint::*m_pfn_applyimpulse)(int _idx);
	void	_applyimpulse_R(int _idx);
	void	_applyimpulse_P(int _idx);
	void	_applyimpulse_U_1(int _idx);
	void	_applyimpulse_U_2(int _idx);
	void	_applyimpulse_B_Y(int _idx);
	void	_applyimpulse_B_RP(int _idx);


	void	GetDelVelocity(SR_REAL * sjari);
	void	(JointConstraint::*m_pfn_getdelvelocity)(SR_REAL * sjari);
	void	_getdelvelocity_R(SR_REAL * sjari);
	void	_getdelvelocity_P(SR_REAL * sjari);
	void	_getdelvelocity_U_1(SR_REAL * sjari);
	void	_getdelvelocity_U_2(SR_REAL * sjari);
	void	_getdelvelocity_B_Y(SR_REAL * sjari);
	void	_getdelvelocity_B_RP(SR_REAL * sjari);


	void	Excite();

	void	UnExcite();
	void	(JointConstraint::*m_pfn_unexcite)();
	void	_unexcite_R();
	void	_unexcite_P();
	void	_unexcite_U_1();
	void	_unexcite_U_2();
	void	_unexcite_B_Y();
	void	_unexcite_B_RP();

	void	SetImpulse(SR_REAL * _lambda);
	void	(JointConstraint::*m_pfn_setimpulse)(SR_REAL * _lambda);
	void	_setimpulse_R(SR_REAL * _lambda);
	void	_setimpulse_P(SR_REAL * _lambda);
	void	_setimpulse_U_1(SR_REAL * _lambda);
	void	_setimpulse_U_2(SR_REAL * _lambda);
	void	_setimpulse_B_Y(SR_REAL * _lambda);
	void	_setimpulse_B_RP(SR_REAL * _lambda);

	srSystem*	UF_Find_Constraint();
};
