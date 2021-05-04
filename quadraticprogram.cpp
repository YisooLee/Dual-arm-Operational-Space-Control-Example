#include "quadraticprogram.h"

CQuadraticProgram::CQuadraticProgram()
{
    Initialize();
}
CQuadraticProgram::~CQuadraticProgram()
{
}

void CQuadraticProgram::Initialize()
{
    _num_var = 1;
    _num_cons = 1;
    InitializeProblemSize(_num_var, _num_cons);

}

void CQuadraticProgram::InitializeProblemSize(const int& num_var, const int& num_cons)
{
    _SQPprob = SQProblem(num_var, num_cons);
    _bool_constraint_Ax = false;
    _bool_constraint_x = false;
    _num_var = num_var;
    _num_cons = num_cons;
    _H.resize(_num_var, _num_var);
    _H.setZero();
    _g.resize(_num_var);
    _g.setZero();
    _A.resize(_num_cons, _num_var);
    _A.setZero();
    _lbA.resize(_num_cons);
    _lbA.setZero();
    _ubA.resize(_num_cons);
    _ubA.setZero();
    _lb.resize(_num_var);
    _lb.setZero();
    _ub.resize(_num_var);
    _ub.setZero();
    _bInitialized = false;
    _num_state = 100;
    _Xopt.resize(_num_var);
    _Xopt.setZero();
    _comptime = 100000.0;
    _options.printLevel = PL_NONE;
}


void CQuadraticProgram::UpdateMinProblem(const MatrixQPd& H, const VectorQPd& g)
{
    for (int i = 0; i < _num_var; i++)
    {
        for (int j = 0; j < _num_var; j++)
        {
            _H(i, j) = H(i, j);
        }
        _g(i) = g(i);
    }
}

void CQuadraticProgram::UpdateSubjectToAx(const MatrixQPd& A, const VectorQPd& lbA, const VectorQPd& ubA)
{
    for (int i = 0; i < _num_cons; i++)
    {
        for (int j = 0; j < _num_var; j++)
        {
            _A(i, j) = A(i, j);
        }
        _lbA(i) = lbA(i);
        _ubA(i) = ubA(i);
    }

    _bool_constraint_Ax = true;

    for (int i = 0; i < _num_cons; i++)
    {
        if (_lbA(i) > _ubA(i))
        {
            cout << "-- Error in Constraint Value in lbA <= Ax <= ubA --" << endl;
            cout << "lbA[" << i << "] is bigger than ubA." << endl;

            _bool_constraint_Ax = false;
        }
    }

    if (_num_cons == 0)
    {
        _bool_constraint_Ax = false;
        cout << "-- Number of Constraint is zero. --" << endl;
    }
}

void CQuadraticProgram::UpdateSubjectToX(const VectorQPd& lb, const VectorQPd& ub)
{
    for (int i = 0; i < _num_var; i++)
    {
        _lb(i) = lb(i);
        _ub(i) = ub(i);
    }
    _bool_constraint_x = true;

    for (int i = 0; i < _num_var; i++)
    {
        if (_lb(i) > _ub(i))
        {
            cout << "-- Error in Constraint Value in lb <= x <= ub --" << endl;
            cout << "lb[" << i << "] is bigger than ub." << endl;

            _bool_constraint_x = false;
        }
    }
    if (_num_var == 0)
    {
        _bool_constraint_x = false;
        cout << "-- Number of Variable is zero. --" << endl;
    }
}

void CQuadraticProgram::DeleteSubjectToAx()
{
    _bool_constraint_Ax = false;
}

void CQuadraticProgram::DeleteSubjectToX()
{
    _bool_constraint_x = false;
}

void CQuadraticProgram::PrintMinProb()
{
    cout << "------------------------------------------------------------------------------" << endl;
    cout << "----------------------------------    H    -----------------------------------" << endl;
    cout << "------------------------------------------------------------------------------" << endl;
    cout << _H << endl;
    cout << "------------------------------------------------------------------------------" << endl;
    cout << "----------------------------------    g    -----------------------------------" << endl;
    cout << "------------------------------------------------------------------------------" << endl;
    cout << _g.transpose() << endl;
    cout << "------------------------------------------------------------------------------" << endl;
}

void CQuadraticProgram::PrintSubjectToAx()
{
    if (_bool_constraint_Ax == true)
    {
        cout << "------------------------------------------------------------------------------" << endl;
        cout << "----------------------------------    A    -----------------------------------" << endl;
        cout << "------------------------------------------------------------------------------" << endl;
        cout << _A << endl;
        cout << "------------------------------------------------------------------------------" << endl;
        cout << "---------------------------------    lbA    ----------------------------------" << endl;
        cout << "------------------------------------------------------------------------------" << endl;
        cout << _lbA.transpose() << endl;
        cout << "------------------------------------------------------------------------------" << endl;
        cout << "---------------------------------    ubA    ----------------------------------" << endl;
        cout << "------------------------------------------------------------------------------" << endl;
        cout << _ubA.transpose() << endl;
        cout << "------------------------------------------------------------------------------" << endl;
    }
    else
    {
        cout << "------------------------------------------------------------------------------" << endl;
        cout << "                   s.t. lbA <= Ax <= ubA is not inserted.                     " << endl;
        cout << "------------------------------------------------------------------------------" << endl;
    }

    if (_num_cons == 0)
    {
        cout << "------------------------------------------------------------------------------" << endl;
        cout << "                             wrong problem type                               " << endl;
        cout << "------------------------------------------------------------------------------" << endl;
    }
}

void CQuadraticProgram::PrintSubjectTox()
{
    if (_bool_constraint_x == true)
    {
        cout << "------------------------------------------------------------------------------" << endl;
        cout << "---------------------------------    lb    -----------------------------------" << endl;
        cout << "------------------------------------------------------------------------------" << endl;
        cout << _lb.transpose() << endl;
        cout << "------------------------------------------------------------------------------" << endl;
        cout << "---------------------------------    ub    -----------------------------------" << endl;
        cout << "------------------------------------------------------------------------------" << endl;
        cout << _ub.transpose() << endl;
        cout << "------------------------------------------------------------------------------" << endl;
    }
    else
    {
        cout << "------------------------------------------------------------------------------" << endl;
        cout << "                     s.t. lb <= x <= ub is not inserted.                      " << endl;
        cout << "------------------------------------------------------------------------------" << endl;
    }
}

void CQuadraticProgram::EnableEqualityCondition(const double Tolerance)
{
    _options.enableEqualities = BT_TRUE;
    real_t Tolerance_equal = Tolerance;
    _options.boundRelaxation = Tolerance_equal;
}

void CQuadraticProgram::DisableEqualityCondition()
{
    _options.enableEqualities = BT_FALSE;
}

void CQuadraticProgram::SetHotstartComputationTimeLimit(const real_t& compute_time) //sec, do not use it yet
{
    _comptime = compute_time;
}

void CQuadraticProgram::EnablePrintOptionDebug()
{
    _options.printLevel = PL_DEBUG_ITER;
}

void CQuadraticProgram::DisablePrintOptionDebug()
{
    _options.printLevel = PL_NONE;
}

void CQuadraticProgram::SolveQPoases(const int& num_max_iter)
{
    //translate eigen to real_t formulation
    // H in min eq 1/2x'Hx + x'g
    for (int i = 0; i < _num_var; i++)
    {
        for (int j = 0; j < _num_var; j++)
        {
            _H_realt[_num_var * j + i] = _H(j, i);
        }
    }

    // g in min eq 1/2x'Hx + x'g
    for (int i = 0; i < _num_var; i++)
    {
        _g_realt[i] = _g(i);
    }

    // A in s.t. eq lbA<= Ax <=ubA
    if (_bool_constraint_Ax == true)
    {
        for (int i = 0; i < _num_var; i++)
        {
            for (int j = 0; j < _num_cons; j++)
            {
                _A_realt[_num_var * j + i] = _A(j, i);
            }
        }
    }

    // lbA in s.t. eq lbA<= Ax <=ubA
    if (_bool_constraint_Ax == true)
    {
        for (int i = 0; i < _num_cons; i++)
        {
            _lbA_realt[i] = _lbA(i);
        }
    }

    // ubA in s.t. eq lbA<= Ax <=ubA
    if (_bool_constraint_Ax == true)
    {
        for (int i = 0; i < _num_cons; i++)
        {
            _ubA_realt[i] = _ubA(i);
        }
    }

    //lb in s.t. eq lb <= x <= ub
    if (_bool_constraint_x == true)
    {
        for (int i = 0; i < _num_var; i++)
        {
            _lb_realt[i] = _lb(i);
        }
    }

    //ub in s.t. eq lb <= x <= ub
    if (_bool_constraint_x == true)
    {
        for (int i = 0; i < _num_var; i++)
        {
            _ub_realt[i] = _ub(i);
        }
    }

    _SQPprob.setOptions(_options);

    int_t nWSR = num_max_iter;
    returnValue m_status;

    if (_bInitialized == false)//init
    {
        if (_bool_constraint_Ax == true && _bool_constraint_x == true)
        {
            m_status = _SQPprob.init(_H_realt, _g_realt, _A_realt, _lb_realt, _ub_realt, _lbA_realt, _ubA_realt, nWSR);            
        }
        else if (_bool_constraint_Ax == true && _bool_constraint_x == false)
        {
            m_status = _SQPprob.init(_H_realt, _g_realt, _A_realt, nullptr, nullptr, _lbA_realt, _ubA_realt, nWSR);            
        }
        else if (_bool_constraint_Ax == false && _bool_constraint_x == true)
        {
            m_status = _SQPprob.init(_H_realt, _g_realt, nullptr, _lb_realt, _ub_realt, nullptr, nullptr, nWSR);            
        }
        else
        {
            m_status = _SQPprob.init(_H_realt, _g_realt, nullptr, nullptr, nullptr, nullptr, nullptr, nWSR);            
        }
        _bInitialized = true;
    }
    else//hotstart
    {
        if (_bool_constraint_Ax == true && _bool_constraint_x == true)
        {
            m_status = _SQPprob.hotstart(_H_realt, _g_realt, _A_realt, _lb_realt, _ub_realt, _lbA_realt, _ubA_realt, nWSR);//,&_comptime);
        }
        else if (_bool_constraint_Ax == true && _bool_constraint_x == false)
        {
            m_status = _SQPprob.hotstart(_H_realt, _g_realt, _A_realt, nullptr, nullptr, _lbA_realt, _ubA_realt, nWSR);//,&_comptime);
        }
        else if (_bool_constraint_Ax == false && _bool_constraint_x == true)
        {
            m_status = _SQPprob.hotstart(_H_realt, _g_realt, nullptr, _lb_realt, _ub_realt, nullptr, nullptr, nWSR);//,&_comptime);
        }
        else
        {
            m_status = _SQPprob.hotstart(_H_realt, _g_realt, nullptr, nullptr, nullptr, nullptr, nullptr, nWSR);//,&_comptime);            
        }
    }

    _SQPprob.getPrimalSolution(_Xopt_realt);    

    _num_state = m_status;
    for (int i = 0; i < _num_var; i++)
    {
        _Xopt(i) = _Xopt_realt[i];
    }
}
