#pragma once

#include <Eigen/Core>

#include <srDyn/srSpace.h>
#include "srExt_Eigen.h"


namespace srExt
{
	/*
	_T	=	|            |
			|    R     p |
			|			 |
			| 0  0  0  1 |

	_s	=	|   |
			| w |
			|   |
			|   |
			| v |
			|   |
	*/




	/*
	|                  |
	|    R       [0]   |
	|		           |
	|                  |
	|  [p]R       R    |
	|				   |
	*/
	Eigen::Matrix6d		AdMat(const SE3& _T);

	Eigen::Matrix6d		invAdMat(const SE3& _T);


	Eigen::Matrix6d		adMat(const se3& _s);

	/*
		|            |
		|   [w]    v |
		|			 |
		| 0  0  0  0 |
	*/
	Eigen::Matrix4d		se3_bracket(const se3& _s);



	///		template functions		///
	template<class _se3, class _dse3>
	Eigen::Vector6d dad(const _se3& s, const _dse3& t)
	{
		Eigen::Vector6d ret_dse3;
		ret_dse3[0] = t[1] * s[2] - t[2] * s[1] + t[4] * s[5] - t[5] * s[4];
		ret_dse3[1] = t[2] * s[0] - t[0] * s[2] + t[5] * s[3] - t[3] * s[5];
		ret_dse3[2] = t[0] * s[1] - t[1] * s[0] + t[3] * s[4] - t[4] * s[3];
		ret_dse3[3] = t[4] * s[2] - t[5] * s[1];
		ret_dse3[4] = t[5] * s[0] - t[3] * s[2];
		ret_dse3[5] = t[3] * s[1] - t[4] * s[0];

		return ret_dse3;
	}

	template<class SRtype>
	Eigen::VectorXd toEigenVec(const SRtype& _vec, int _numel, int _offset = 0)
	{
		Eigen::VectorXd ret(_numel);
		for (int i = 0; i < _numel; i++)
			ret(i) = _vec[i + _offset];
		return ret;
	}

	template<class SRtype, class MatrixType>
	SRtype fromEigenVec(const MatrixType& _vec)
	{
		SRtype	ret;
		for (int i = 0; i < _vec.size(); i++)
			ret[i] = _vec(i);
		return ret;
	}

	template<class SRtype>
	Eigen::MatrixXd toEigenMat(const SRtype& _mat, int _rows, int _cols)
	{
		Eigen::MatrixXd ret(_rows, _cols);
		for (int r = 0; r < _rows; r++)
			for (int c = 0; c < _cols; c++)
				ret(r, c) = _mat(r, c);
		return ret;
	}

	template<class MatrixType>
	Eigen::MatrixXd		operator * (const MatrixType& _M, const SO3& _R)
	{
		assert(_M.cols() == 3 && "Dimension mismatch.");
		Eigen::MatrixXd ret(_M.rows(), 3);
		ret.setZero();

		for (int r = 0; r < _M.rows(); r++)
			for (int c = 0; c < 3; c++)
				for (int i = 0; i < 3; i++)
					ret(r, c) += _M(r, i) * _R(i, c);

		return ret;
	}

	template<class MatrixType>
	Eigen::MatrixXd		operator * (const SO3& _R, const MatrixType& _M)
	{
		assert(_M.rows() == 3 && "Dimension mismatch.");
		Eigen::MatrixXd ret(3, _M.cols());
		ret.setZero();

		for (int r = 0; r < 3; r++)
			for (int c = 0; c < _M.cols(); c++)
				for (int i = 0; i < 3; i++)
					ret(r, c) += _R(r, i) * _M(i, c);

		return ret;
	}


	template<class MatrixType>
	Eigen::MatrixXd		operator * (const MatrixType& _M, const SE3& _T)
	{
		assert(_M.cols() == 4 && "Dimension mismatch.");
		Eigen::MatrixXd ret(_M.rows(), 4);
		ret.setZero();

		for (int r = 0; r < _M.rows(); r++)
			for (int c = 0; c < 4; c++)
				for (int i = 0; i < 4; i++)
					ret(r, c) += _M(r, i) * _T(i, c);

		return ret;
	}

	template<class MatrixType>
	Eigen::MatrixXd		operator * (const SE3& _T, const MatrixType& _M)
	{
		assert(_M.rows() == 4 && "Dimension mismatch.");
		Eigen::MatrixXd ret(4, _M.cols());
		ret.setZero();

		for (int r = 0; r < 4; r++)
			for (int c = 0; c < _M.cols(); c++)
				for (int i = 0; i < 4; i++)
					ret(r, c) += _T(r, i) * _M(i, c);

		return ret;
	}

	template<class MatrixType>
	Eigen::MatrixXd		operator * (const MatrixType& _M, const Vec3& _v)
	{
		assert(_M.cols() == 3 && "Dimension mismatch.");
		Eigen::MatrixXd ret(_M.rows(), 1);
		ret.setZero();

		for (int r = 0; r < _M.rows(); r++)
				for (int i = 0; i < 3; i++)
					ret(r, 0) += _M(r, i) * _v[i];

		return ret;
	}
}