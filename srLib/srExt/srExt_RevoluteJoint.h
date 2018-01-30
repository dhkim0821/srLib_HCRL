#pragma once

#include <srDyn/srRevoluteJoint.h>
#include <srMath/Diagnostic.h>

class srExtRevoluteJoint : public srRevoluteJoint
{
public:

	srExtRevoluteJoint()
		:srRevoluteJoint()
	{
		m_IsVelLimited = false;
		m_VelLimit[0] = -100.0;
		m_VelLimit[0] = +100.0;

		m_IsAccLimited = false;
		m_AccLimit[0] = -100.0;
		m_AccLimit[0] = +100.0;

		m_IsMotorParameters = false;
		m_TorqueConstant = 1.0;
		m_Resistance = 1.0;
		m_GearRatio = 1.0;
		m_IsGearRatioSet = false;
		m_RotorInertia = 0.0;
		m_ViscousCoeff = 0.0;
		m_CoulombCoeff = 0.0;
		m_MaxRPM = 10000.0;
		m_IsMaxRPMSet = false;
	}

	/*
	Velocity limit related functions.
	*/
	bool	IsVelocityLimited()
	{
		return m_IsVelLimited;
	}

	void	MakeVelocityLimit(bool v = true)
	{
		m_IsVelLimited = v;
	}

	SR_REAL	GetVelocityLowerLimit()
	{
		return m_VelLimit[0];
	}

	SR_REAL	GetVelocityUpperLimit()
	{
		return m_VelLimit[1];
	}

	void	SetVelocityLimit(SR_REAL lowerlimit, SR_REAL upperlimit)
	{
		m_VelLimit[0] = lowerlimit;
		m_VelLimit[1] = upperlimit;
	}

	void	SetVelocityLimit(SR_REAL abslimit)
	{
		m_VelLimit[0] = -abslimit;
		m_VelLimit[1] = +abslimit;
	}

	void	SetVelocityLimit()
	{
		// set velocity limit with gear ratio and maximum rpm
		LOGIF(m_IsGearRatioSet && m_IsMaxRPMSet, "gear ratio or maximum RPM are not set");
		m_VelLimit[0] = -m_MaxRPM * 360.0 / 60.0 / m_GearRatio;
		m_VelLimit[1] = -m_VelLimit[0];
	}

	/*
	Acceleration limit related functions.
	*/
	bool	IsAccelerationLimited()
	{
		return m_IsAccLimited;
	}

	void	MakeAccelerationLimit(bool v = true)
	{
		m_IsAccLimited = v;
	}

	SR_REAL	GetAccelerationLowerLimit()
	{
		return m_AccLimit[0];
	}

	SR_REAL	GetAccelerationUpperLimit()
	{
		return m_AccLimit[1];
	}

	void	SetAccelerationLimit(SR_REAL lowerlimit, SR_REAL upperlimit)
	{
		m_AccLimit[0] = lowerlimit;
		m_AccLimit[1] = upperlimit;
	}

	void	SetAccelerationLimit(SR_REAL abslimit)
	{
		m_AccLimit[0] = -abslimit;
		m_AccLimit[1] = +abslimit;
	}

	/*
	Motor parameters boolean related functions.
	*/
	bool	IsMotorParameters()
	{
		return m_IsMotorParameters;
	}

	void	MakeMotorParameters(bool v = true)
	{
		m_IsMotorParameters = v;
	}

	/*
	Torque constant related functions.
	*/
	const SR_REAL	GetTorqueConstant() const
	{
		return m_TorqueConstant;
	}

	void	SetTorqueConstant(const SR_REAL TorqueCnt)
	{
		m_TorqueConstant = TorqueCnt;
	}

	/*
	Resistance related functions.
	*/
	const SR_REAL	GetResistance() const
	{
		return m_Resistance;
	}

	void	SetResistance(const SR_REAL R)
	{
		m_Resistance = R;
	}

	/*
	Gear ratio related functions.
	*/
	const SR_REAL	GetGearRatio() const
	{
		return m_GearRatio;
	}

	void	SetGearRatio(const SR_REAL G)
	{
		m_GearRatio = G;
		m_IsGearRatioSet = true;
	}

	/*
	Rotor inertia related functions.
	*/
	const SR_REAL	GetRotorInertia() const
	{
		return m_RotorInertia;
	}

	void	SetRotorInertia(const SR_REAL R)
	{
		m_RotorInertia = R;
	}

	/*
	Friction coefficient related functions.
	*/
	const SR_REAL	GetViscousCoeff() const
	{
		return m_ViscousCoeff;
	}

	void	SetViscousCoeff(const SR_REAL fv)
	{
		m_ViscousCoeff = fv;
	}

	const SR_REAL	GetCoulombCoeff() const
	{
		return m_ViscousCoeff;
	}

	void	SetCoulombCoeff(const SR_REAL fc)
	{
		m_ViscousCoeff = fc;
	}

	void	SetFrictionCoeff(const SR_REAL fv, const SR_REAL fc)
	{
		m_ViscousCoeff = fv;
		m_CoulombCoeff = fc;
	}

	/*
	Maximum RPM related functions.
	*/
	const SR_REAL	GetMaxRPM() const
	{
		return m_MaxRPM;
	}

	void	SetMaxRPM(const SR_REAL maxRPM)
	{
		m_MaxRPM = maxRPM;
		m_IsMaxRPMSet = true;
	}

	/*
	Motor parameters copy function
	*/
	void CopyMotorParameters(srExtRevoluteJoint* J)
	{
		m_IsMotorParameters = true;
		m_TorqueConstant = J->GetTorqueConstant();
		m_Resistance = J->GetResistance();
		SetGearRatio(J->GetGearRatio());
		m_RotorInertia = J->GetRotorInertia();
		m_ViscousCoeff = J->GetViscousCoeff();
		m_CoulombCoeff = J->GetCoulombCoeff();
		SetMaxRPM(J->GetMaxRPM());
		SetVelocityLimit();
		MakeVelocityLimit();
	}	

	////////////////////
	//	member vars.
	////////////////////
	
	/*!
	Limits of revolute joint angluar velocity. m_VelLimit[0] is lower limit and m_VelLimit[1] is upper limit.
	Upper limit must be greater than lower limit. This should be specified by user. Unit is deg/s.
	*/
	SR_REAL	m_VelLimit[2];
	bool m_IsVelLimited;

	/*!
	Limits of revolute joint angluar acceleration. m_AccLimit[0] is lower limit and m_AccLimit[1] is upper limit.
	Upper limit must be greater than lower limit. This should be specified by user. Unit is deg/s^2.
	*/
	SR_REAL	m_AccLimit[2];
	bool m_IsAccLimited;

	/*!
	boolean indicating whether or not motor parameters are set.
	Motor parameters include torque constant, resistance, gear ration, and rotor inertia.
	*/
	bool m_IsMotorParameters;

	/*!
	Torque constant of motor. This should be specified by user. Unit is Nm / A
	*/
	SR_REAL	m_TorqueConstant;

	/*!
	Resistance of motor. This should be specified by user. Unit is Ohm
	*/
	SR_REAL m_Resistance;

	/*!
	Gear ratio of motor. This should be specified by user. nondimension
	*/
	SR_REAL m_GearRatio;
	bool m_IsGearRatioSet;

	/*!
	Rotor inetia of motor. This should be specified by user. Unit is kg m^2
	*/
	SR_REAL m_RotorInertia;

	/*!
	Friction coefficient of motor. This should be specified by user.
	*/
	SR_REAL m_ViscousCoeff;
	SR_REAL m_CoulombCoeff;

	/*!
	Maximum RPM of motor. This should be specified by user.
	*/
	SR_REAL m_MaxRPM;
	bool m_IsMaxRPMSet;
};
