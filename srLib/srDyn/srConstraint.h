#pragma once

#include "srDyn/srSystem.h"
#include "LieGroup/_array.h"

//#define	INFINITY_BK									1.7976931348623158e+308 // SR_REAL max value
#define	PINFINITY_BK									1e+300 // SR_REAL max value
#define	NINFINITY_BK									-1e+300 // SR_REAL max value

struct ConstraintInfo
{
	SR_REAL recip_timestep;

	SR_REAL * _rhs;
	SR_REAL * _lambda;

	SR_REAL * _lo;
	SR_REAL * _hi;
	int	   * _findex;
};


class Constraint
{
public:
	// variables
	int		nd;		// number of constraint dimension
	bool	type1;	// true : solid , false : blink


	// methods
	virtual void		GetInformation(ConstraintInfo * info) = 0;
	virtual	void		ApplyImpulse(int _idx) = 0;
	virtual void		GetDelVelocity(SR_REAL * sjari) = 0;
	virtual void		Excite() = 0;
	virtual void		UnExcite() = 0;
	virtual void		SetImpulse(SR_REAL * _lambda) = 0;
	virtual srSystem*	UF_Find_Constraint() = 0;


	// union-find
	static srSystem* UF_Find_System_PathCompression(srSystem* pSystem);
	static srSystem* UF_Find_System(srSystem* pSystem);
};
typedef _array<Constraint*> ConstraintPtrArray;
