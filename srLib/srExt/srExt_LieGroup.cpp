#include "srExt_LieGroup.h"

using namespace Eigen;
Eigen::Matrix6d srExt::AdMat(const SE3 & _T)
{
	Eigen::Matrix6d AdMatrix = Eigen::Matrix6d::Zero();

	//	fill upper left and lower right block
	for (int row = 0; row < 3; row++)
		for (int col = 0; col < 3; col++)
		{
			AdMatrix(row, col) = _T(row, col);
			AdMatrix(row + 3, col + 3) = _T(row, col);
		}

	//	fill lower left block
	double p0 = _T(0, 3);
	double p1 = _T(1, 3);
	double p2 = _T(2, 3);
	Eigen::Matrix3d	p_skew;
	p_skew <<
		0, -p2, p1,
		p2, 0, -p0,
		-p1, p0, 0;

	AdMatrix.block(3, 0, 3, 3) = p_skew * AdMatrix.block(0, 0, 3, 3);

	return AdMatrix;
}



Eigen::Matrix6d srExt::invAdMat(const SE3 & _T)
{
	Eigen::Matrix6d result;

	for (int r = 0; r < 3; r++)
		for (int c = 0; c < 3; c++)
		{
			result(r, c) = _T(c, r);
			result(r, c + 3) = 0.0;
			result(r + 3, c + 3) = _T(c, r);
		}

	result(3, 0) = -_T(1, 0)*_T(2, 3) + _T(2, 0)*_T(1, 3);
	result(3, 1) = _T(0, 0)*_T(2, 3) - _T(2, 0)*_T(0, 3);
	result(3, 2) = -_T(0, 0)*_T(1, 3) + _T(1, 0)*_T(0, 3);

	result(4, 0) = -_T(1, 1)*_T(2, 3) + _T(2, 1)*_T(1, 3);
	result(4, 1) = _T(0, 1)*_T(2, 3) - _T(2, 1)*_T(0, 3);
	result(4, 2) = -_T(0, 1)*_T(1, 3) + _T(1, 1)*_T(0, 3);

	result(5, 0) = -_T(1, 2)*_T(2, 3) + _T(2, 2)*_T(1, 3);
	result(5, 1) = _T(0, 2)*_T(2, 3) - _T(2, 2)*_T(0, 3);
	result(5, 2) = -_T(0, 2)*_T(1, 3) + _T(1, 2)*_T(0, 3);

	return result;
}

Eigen::Matrix6d srExt::adMat(const se3 & _s)
{
	Eigen::Matrix6d result;

	result(0, 0) = 0;
	result(0, 1) = -_s[2];
	result(0, 2) = _s[1];

	result(0, 3) = 0;
	result(0, 4) = 0;
	result(0, 5) = 0;

	result(1, 0) = _s[2];
	result(1, 1) = 0;
	result(1, 2) = -_s[0];

	result(1, 3) = 0;
	result(1, 4) = 0;
	result(1, 5) = 0;

	result(2, 0) = -_s[1];
	result(2, 1) = _s[0];
	result(2, 2) = 0;

	result(2, 3) = 0;
	result(2, 4) = 0;
	result(2, 5) = 0;

	result(3, 0) = 0;
	result(3, 1) = -_s[5];
	result(3, 2) = _s[4];

	result(3, 3) = 0;
	result(3, 4) = -_s[2];
	result(3, 5) = _s[1];

	result(4, 0) = _s[5];
	result(4, 1) = 0;
	result(4, 2) = -_s[3];

	result(4, 3) = _s[2];
	result(4, 4) = 0;
	result(4, 5) = -_s[0];

	result(5, 0) = -_s[4];
	result(5, 1) = _s[3];
	result(5, 2) = 0;

	result(5, 3) = -_s[1];
	result(5, 4) = _s[0];
	result(5, 5) = 0;

	return result;
}

Eigen::Matrix4d srExt::se3_bracket(const se3 & _s)
{
	Eigen::Matrix4d ret;

	//	rotation part
	ret(0, 0) = 0;
	ret(1, 1) = 0;
	ret(2, 2) = 0;

	ret(1, 0) = _s[2];
	ret(0, 1) = -_s[2];

	ret(2, 0) = -_s[1];
	ret(0, 2) = _s[1];

	ret(2, 1) = _s[0];
	ret(1, 2) = -_s[0];


	//	linear part
	ret(0, 3) = _s[3];
	ret(1, 3) = _s[4];
	ret(2, 3) = _s[5];

	ret(3, 0) = 0;
	ret(3, 1) = 0;
	ret(3, 2) = 0;
	ret(3, 3) = 0;

	return ret;
}
