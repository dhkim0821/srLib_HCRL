/*!
*	\file	GaussianQuadrature.h
*	\date	2016.01.23
*	\author	Keunjun Choi(ckj.monikaru@gmail.com)
*	\brief	GaussianQuadrature class header file
*/

#pragma once

#include "LinearIntegrator.h"

namespace srMath
{
	class GaussianQuadrature : public LinearIntegrator
	{
	public:
		//const enum Schem { LG, LGR, LGL };
		
		GaussianQuadrature(int num_of_points = 0, Real initialTime = 0, Real finalTime = 1);

		virtual void	UpdateNumOfPoints(int num_of_points);
		virtual void	UpdateTimeInterval(Real initialTime, Real finalTime);

	protected:
		void	UpdateWeights();
		// sampling points in [-1, 1].
		VectorX		_x;

	public:


	};
}