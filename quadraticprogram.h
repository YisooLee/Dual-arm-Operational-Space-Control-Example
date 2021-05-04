#ifndef QUADRATICPROGRAM_H
#define QUADRATICPROGRAM_H

#include <iostream>
#include <Eigen/Dense>
#include "qpOASES.hpp"

#define useDynamicMemoryAllocation

#ifdef useDynamicMemoryAllocation
	#define CustomQPArraySize Dynamic
#else
	#define CustomQPArraySize 80
#endif

typedef double  rScalar;
namespace Eigen
{
		typedef Matrix<rScalar, -1, -1, 0, CustomQPArraySize, CustomQPArraySize> MatrixQPd;
		typedef Matrix<rScalar, -1, -1, 0, CustomQPArraySize, 1> VectorQPd;	
}

using namespace Eigen;
using namespace std;
using namespace qpOASES;

class CQuadraticProgram
{
public:
	CQuadraticProgram();
	virtual ~CQuadraticProgram();

public:
	void InitializeProblemSize(const int& num_var, const int& num_cons);
	void UpdateMinProblem(const MatrixQPd& H, const VectorQPd& g);
	void UpdateSubjectToAx(const MatrixQPd& A, const VectorQPd& lbA, const VectorQPd& ubA);
	void UpdateSubjectToX(const VectorQPd& lb, const VectorQPd& ub);
	void DeleteSubjectToAx();
	void DeleteSubjectToX();
	void PrintMinProb();
	void PrintSubjectToAx();
	void PrintSubjectTox();
	void EnablePrintOptionDebug();
	void DisablePrintOptionDebug();
	void EnableEqualityCondition(const double Tolerance);
	void DisableEqualityCondition();
	void SetHotstartComputationTimeLimit(const real_t& compute_time); //it maybe unstable
	void SolveQPoases(const int& num_max_iter);
	int _num_state;
	VectorQPd _Xopt;
	bool _bInitialized;
	int _num_var;
	int _num_cons;

private:
	SQProblem _SQPprob;
	Options _options;	
	MatrixQPd _H;
	VectorQPd _g;
	bool _bool_constraint_Ax;
	MatrixQPd _A;
	VectorQPd _lbA;
	VectorQPd _ubA;
	bool _bool_constraint_x;
	VectorQPd _lb;
	VectorQPd _ub;
	real_t _comptime;

	real_t _H_realt[10000] = { 0 };
	real_t _g_realt[100] = { 0 };
	real_t _A_realt[10000] = { 0 };
	real_t _lbA_realt[100] = { 0 };
	real_t _ubA_realt[100] = { 0 };
	real_t _lb_realt[100] = { 0 };
	real_t _ub_realt[100] = { 0 };
	real_t _Xopt_realt[100] = { 0 };


	void Initialize();
};

#endif // QUADRATICPROGRAM_H