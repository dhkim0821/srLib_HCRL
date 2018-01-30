#pragma once

#include "Constant.h"

namespace srMath
{
	class LinearIntegrator
	{
	public:
		LinearIntegrator(int num_of_points = 0, Real initialTime = 0.0, Real finalTime = 1.0)
			: _N(num_of_points), _t0(initialTime), _tf(finalTime)
		{
			//	user should initialize _t and _w in constructor of derived class.
		}
		virtual			~LinearIntegrator() {}

		virtual void	UpdateNumOfPoints(int num_of_points) = 0;
		virtual void	UpdateTimeInterval(Real initialTime, Real finalTime) = 0;

		int				GetNumberOfPoints() const { return _N; }
		double			GetInitialTime() const { return _t0; }
		double			GetFinalTime() const { return _tf; }
		const VectorX&	GetPoints() const { return _t; }
		const VectorX&	GetWeights() const { return _w; }

		const Real		EvalIntegration(const VectorX& functionVal) const { return _w.dot(functionVal); }

	protected:

		// sampling points from t0 to tf.
		VectorX		_t;
		// weight for integration.
		VectorX		_w;
		//	number of points.
		int			_N;
		// initial time & final time
		Real		_t0, _tf;
	};
}