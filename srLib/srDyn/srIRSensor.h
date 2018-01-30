#ifndef SRLIB_IR_SENSOR
#define SRLIB_IR_SENSOR

#include "srDyn/srSensor.h"
#include "LieGroup/_array.h"


/*!
	\class srIRSensor
	\brief Class represents IR spot range sensor.
*/
class srIRSensor : public srSensor
{
public:
	/*!
		Maximum detection range. This should be specified by user.
		This must be greater than or equal to m_MinRange.
		Default is 1.0.
	*/
	SR_REAL	m_MaxRange;
	/*!
		Minimum detection range. This should be specified by user.
		This value must be between zero and m_MaxRange.
		Default is 0.0.
	*/
	SR_REAL	m_MinRange;
	/*!
		Get maximum detection range.
	*/
	SR_REAL	GetMaxRange();
	/*!
		Get minimum detection range.
	*/
	SR_REAL	GetMinRange();
	/*!
		Set maximum detection range.
	*/
	void	SetMaxRange(SR_REAL );
	/*!
		Set minimum detection range.
	*/
	void	SetMinRange(SR_REAL );
	/*!
		Set maximum and minimum detection range.
	*/
	void	SetRange(SR_REAL max, SR_REAL min = 0.0);
public:
	/*!
		Constructor.
	*/
	srIRSensor();

	/*!
		List of collision entities that sensor can detect.
	*/
	_array<srCollision*>	m_Objects;
	/*!
		Detected value of sensor.
	*/
	SR_REAL	m_DetectedValue;

	/*!
		Reset detected value of sensor.
	*/
	void	ResetSensor();
	/*!
		Get Detected value of sensor.
	*/
	SR_REAL&	GetDetectedValue();

	/*!
		Add collision entity to detection list.
	*/
	void	AddObject(srCollision* );
	/*!
		Remove collision entity to detection list.
	*/
	void	RemoveObject(srCollision* );
	/*!
		Reset detection list.
	*/
	void	ClearObject();

	/*!
		Initialize sensor. This function is called once before simulation.
	*/
	void	Initialize();
	/*!
		Sensor operate.
	*/
	void	Detect();

	void	Draw();
};

#endif

