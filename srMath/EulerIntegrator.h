#pragma once

#include "LinearIntegrator.h"

namespace srMath
{
	class EulerIntegrator : public LinearIntegrator
	{
	public:
		EulerIntegrator(int num_of_points = 0, Real initialTime = 0, Real finalTime = 1)
			:LinearIntegrator(num_of_points, initialTime, finalTime)
		{
			UpdateNumOfPoints(num_of_points);
		}

		virtual void	UpdateNumOfPoints(int num_of_points)
		{
			_N = num_of_points;
			_t = VectorX::LinSpaced(_N, _t0, _tf);

			_w = VectorX::Constant(_N, (_tf - _t0) / _N);
		}
		virtual void	UpdateTimeInterval(Real initialTime, Real finalTime)
		{
			assert(finalTime > initialTime && "finalTime should be larger than initialTime.");
			_t0 = initialTime;
			_tf = finalTime;
			_t.setLinSpaced(_N, _t0, _tf);
			//_t[_t.size() - 1] -= 1e-7;
			_w.setConstant((_tf - _t0) / _N);
		}

	};
}