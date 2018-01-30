#pragma once

#include "srDyn/srConstraint.h"


//**********************************************************************//
// Closed Loop
class ClosedLoop : public Constraint
{
public:
	ClosedLoop()
	{
		nd = 6;
		type1 = true;

		lambda[0] = 0.0;
		lambda[1] = 0.0;
		lambda[2] = 0.0;
		lambda[3] = 0.0;
		lambda[4] = 0.0;
		lambda[5] = 0.0;
	};

	// static members
	static void SetErp(SR_REAL _erp);
	static SR_REAL erp_closedloop;

	static void SetAllowedPenetration(SR_REAL _allowedpenetration);
	static SR_REAL allowederror;

	static void SetMaximumErpVelocity(SR_REAL _maximum_erp_velocity);
	static SR_REAL maximum_erp_velocity;

	// member variables
	static dse3 UnitImp[6]; // constraint jacobian : this is same for all closed loop, hence i chose to use static variable.
	SR_REAL	closedloopError[6];
	SR_REAL	lambda[6];

	srSystem	* pSystem;
	srLink	* pLeftMass;
	srLink	* pRightMass;

	SE3		HomeRelativeFrame;
	SE3		RelativeFrame;

	// member functions
	void		GetError(SR_REAL _recip_timestep);

	// virtual functions
	void		GetInformation(ConstraintInfo * info);
	void		ApplyImpulse(int _idx);
	void		GetDelVelocity(SR_REAL * sjari);
	void		Excite();
	void		UnExcite();
	void		SetImpulse(SR_REAL * _lambda);
	srSystem*	UF_Find_Constraint();

};
//**********************************************************************//

