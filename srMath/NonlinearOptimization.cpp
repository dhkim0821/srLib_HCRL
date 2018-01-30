//#include "NonlinearOptimization.h"

//#include "Common.h"

//#include <iostream>
//#include <time.h>

//using namespace std;
//using namespace nlopt;

//namespace srMath
//{
	//double objective(unsigned n, const double* x, double* grad, void *f_data)
	//{
		//NonlinearOptimization* ptr = reinterpret_cast<NonlinearOptimization*>(f_data);
		//VectorX xValue(n);
		//for (unsigned int i = 0; i < n; i++)
		//{
			//xValue(i) = x[i];
		//}
		//if (grad)
		//{
			//MatrixX jacobian = (*ptr->getObjectiveFunction()).Jacobian(xValue);
			//for (unsigned int j = 0; j < n; j++)
			//{
				//grad[j] = jacobian(0, j);
			//}
		//}
		//return (*ptr->getObjectiveFunction())(xValue)(0);
	//}

	//void constraint(unsigned m, double* result, unsigned n, const double* x, double* grad, void* f_data)
	//{
		//pair<bool, NonlinearOptimization*>* data = reinterpret_cast<pair<bool, NonlinearOptimization*>*>(f_data);
		//FunctionPtr constraintFunction;
		//if (!data->first)
		//{
			//constraintFunction = data->second->getEqualityConstraint();
		//}
		//else
		//{
			//constraintFunction = data->second->getInequalityConstraint();
		//}
		//VectorX xValue(n);
		//for (unsigned int i = 0; i < n; i++)
		//{
			//xValue(i) = x[i];
		//}
		//if (grad)
		//{
			//MatrixX jacobian = (*constraintFunction).Jacobian(xValue);
			//for (unsigned int i = 0; i < m; i++)
			//{
				//for (unsigned int j = 0; j < n; j++)
				//{
					//grad[i*n + j] = jacobian(i, j);
				//}
			//}
		//}
		//VectorX fval = (*constraintFunction).func(xValue);
		//for (unsigned int i = 0; i < m; i++)
		//{
			//result[i] = fval(i);
		//}
	//}

	//void NonlinearOptimization::solve(const VectorX& initialX, const VectorX& lower, const VectorX& upper)
	//{
		//if (_xN == -1)		_xN = initialX.size();
		//if (_eqN == -1)		_eqN = _eqConstraint->func(initialX).size();
		//if (_ineqN == -1)	_ineqN = _ineqConstraint->func(initialX).size();

		//if (_eqN != 0) optimizer = opt(nlopt::LD_SLSQP, _xN);
		//else optimizer = opt(nlopt::LD_MMA, _xN);

		//optimizer.set_min_objective(objective, this);
		//if (_eqN != 0)
		//{
			//optimizer.add_equality_mconstraint(constraint, new pair<bool, NonlinearOptimization*>(false, this), vector<Real>(_eqN, 1e-5));
		//}
		//if (_ineqN != 0)
		//{
			//optimizer.add_inequality_mconstraint(constraint, new pair<bool, NonlinearOptimization*>(true, this), vector<Real>(_ineqN, 1e-5));
		//}

		//Real startTime = clock();

        //optimizer.set_maxtime(1.5); // 원래는 없었음
        //optimizer.set_maxtime(3.0);
		//optimizer.set_maxtime(10);
        //optimizer.set_maxtime(30);
		//optimizer.set_maxeval(500);
        //optimizer.set_xtol_rel(1e-3);
        //optimizer.set_ftol_rel(1e-3);
        //optimizer.set_xtol_rel(1e-4);
        //optimizer.set_ftol_rel(1e-4);
		//optimizer.set_xtol_rel(1e-6);
		//optimizer.set_ftol_rel(1e-6);


		//if (lower.size() != 0)
		//{
			//vector<Real> lower_bounds;
			//for (int i = 0; i < _xN; i++)
			//{
				//lower_bounds.push_back(lower(i));
			//}
			//optimizer.set_lower_bounds(lower_bounds);
		//}
		//if (upper.size() != 0)
		//{
			//vector<Real> upper_bounds;
			//for (int i = 0; i < _xN; i++)
			//{
				//upper_bounds.push_back(upper(i));
			//}
			//optimizer.set_upper_bounds(upper_bounds);
		//}

		//std::vector<Real> x(_xN);
		//for (int i = 0; i < _xN; i++)
		//{
			//x[i] = initialX(i);
		//}

        ///////////////////////////
        //optimizer.optimize(x, resultFunc);

        //resultX = VectorX::Zero(_xN);
        //for (int i = 0; i < _xN; i++)
        //{
            //resultX(i) = x[i];
        //}
        /////////////////////////

		//VectorX minX = initialX;
		//Real minObjVal = RealMax;
		//nlopt::result current_code, final_code;
		//for (unsigned int i = 0; i < 1[>10<]; i++) // 3 -> 10
		//{
			//Real past = resultFunc;
			//current_code = optimizer.optimize(x, resultFunc);

			//VectorX x_vec(_xN);
            //std::copy(x.begin(), x.end(), x_vec.data());
			//for (unsigned int i = 0; i < x.size(); i++)
				//x_vec[i] = x[i];
			
			//resultFunc = _objectFunc->func(x_vec)(0);

			//if (minObjVal > resultFunc && RealLessEqual(_ineqConstraint->func(x_vec), 1e-3))
			//{
				//minX = x_vec;
				//minObjVal = resultFunc;
				//final_code = current_code;
			//}
		//}

		//if (!RealLessEqual(_ineqConstraint->func(minX), 1e-3))
		//{
			//minObjVal = RealMax;
		//}

		//cout << "[OPTIMIZATION] Computation Time: " << clock() - startTime << "ms" << endl;
		//cout << "[OPTIMIZATION] Result: ";
		//switch (final_code)
		//{
		//case NLOPT_SUCCESS:
			//cout << "Optimized successfully." << endl;
			//break;
		//case NLOPT_STOPVAL_REACHED:
			//cout << "Optimization stopped because stopval was reached." << endl;
			//break;
		//case NLOPT_FTOL_REACHED:
			//cout << "Optimization stopped because ftol_rel or ftol_abs was reached." << endl;
			//break;
		//case NLOPT_XTOL_REACHED:
			//cout << "Optimization stopped because xtol_rel or xtol_abs was reached." << endl;
			//break;
		//case NLOPT_MAXEVAL_REACHED:
			//cout << "Optimization stopped because maxeval was reached." << endl;
			//break;
		//case NLOPT_MAXTIME_REACHED:
			//cout << "Optimization stopped because maxtime was reached." << endl;
			//break;

		//case NLOPT_FAILURE:
			//cout << "Generic failure code." << endl;
			//break;
		//case NLOPT_INVALID_ARGS:
			//cout << "Invalid arguments (e.g. lower bounds are bigger than upper bounds, an unknown algorithm was specified, etcetera)." << endl;
			//break;
		//case NLOPT_OUT_OF_MEMORY:
			//cout << "Ran out of memory." << endl;
			//break;
		//case NLOPT_ROUNDOFF_LIMITED:
			//cout << "Halted because roundoff errors limited progress. (In this case, the optimization still typically returns a useful result.)" << endl;
			//break;
		//case NLOPT_FORCED_STOP:
			//cout << "Halted because of a forced termination: the user called nlopt_force_stop(opt) on the optimization’s nlopt_opt object opt from the user’s objective function or constraints." << endl;
			//break;
		//}
		
        ///////////////////////////// 만약에 정답이 없는 경우 처리 ///////////////////////////////
		//resultX = minX;
		//resultFunc = minObjVal;
	//}
//}
