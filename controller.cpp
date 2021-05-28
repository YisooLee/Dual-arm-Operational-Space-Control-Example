#include "controller.h"

CController::CController(int JDOF)
{
	_dofj = JDOF;
	Initialize();
}

CController::~CController()
{
}

void CController::read(double t, double* q, double* qdot)
{	
	
	_dt = t - _pre_t;
	_pre_t = t;

	for (int i = 0; i < _dofj; i++)
	{
		_q(i) = q[i];
		//_qdot(i) = qdot[i]; //from simulator
		_qdot(i) = CustomMath::VelLowpassFilter(_dt, 2.0*PI* 10.0, _pre_q(i), _q(i), _pre_qdot(i)); //low-pass filter
		
		_pre_q(i) = _q(i);
		_pre_qdot(i) = _qdot(i);
	}
}

void CController::write(double* torque)
{
	for (int i = 0; i < _dofj; i++)
	{
		torque[i] = _torque(i);
	}
}

void CController::control_mujoco(double time)
{
	_t = time;
	ModelUpdate();	

	//Motion planning
	if (_t < 1.0 && _bool_joint_motion == false)
	{
		_control_mode = 1;

		_start_time = 0.0;
		_end_time = 1.0;		
		_q_goal.setZero();
		_qdot_goal.setZero();
		JointTrajectory.reset_initial(_start_time, _q, _qdot);
		JointTrajectory.update_goal(_q_goal, _qdot_goal, _end_time);
		_bool_joint_motion = true;		
		cout << endl << "Maintain Initial Position, t = " << _t << " s" << endl;

		cout << Model._R_left_hand << endl<<endl;
	}
	else if (_t >= 1.0 && _t < 4.0 && _bool_joint_motion == true)
	{
		_control_mode = 1;

		_start_time = 1.0;
		_end_time = 4.0;
		_q_goal.setZero();
		_qdot_goal.setZero();
		_q_goal = _q_home;//home position
		JointTrajectory.reset_initial(_start_time, _q, _qdot);
		JointTrajectory.update_goal(_q_goal, _qdot_goal, _end_time);
		_bool_joint_motion = false;	

		cout << "Start <Home Position>, t = " << _t << " s" << endl;
	}
	else if (_t >= 4.0 && _t < 5.0 && _bool_joint_motion == false && _bool_hands_motion == false)
	{
		_control_mode = 4;

		_start_time = 4.0;
		_end_time = 5.0;

		_R_goal_right_hand = Model._R_right_hand;
		_R_goal_left_hand = Model._R_left_hand;
		/*
		_R_goal_right_hand.setZero();
		_R_goal_right_hand(0, 2) = 1.0;
		_R_goal_right_hand(1, 1) = -1.0;
		_R_goal_right_hand(2, 0) = 1.0;
		_R_goal_left_hand.setZero();
		_R_goal_left_hand(0, 2) = 1.0;
		_R_goal_left_hand(1, 1) = 1.0;
		_R_goal_left_hand(2, 0) = -1.0;
		*/

		_x_goal_left_hand = Model._x_left_hand; //set as current state
		LeftHandPosTrajectory.reset_initial(_start_time, Model._x_left_hand, Model._xdot_left_hand.head(3));
		LeftHandPosTrajectory.update_goal(_x_goal_left_hand, _xdot_goal_left_hand, _end_time);
		_x_goal_right_hand = Model._x_right_hand; //set as current state
		RightHandPosTrajectory.reset_initial(_start_time, Model._x_right_hand, Model._xdot_right_hand.head(3));
		RightHandPosTrajectory.update_goal(_x_goal_right_hand, _xdot_goal_right_hand, _end_time);

		_bool_hands_motion = true;

		cout << "Start Operational Sapce Control - Regulate Initial Hand Positions, t = " << _t << " s" << endl;
	}
	else if (_t >= 5.0 && _t < 6.0 && _bool_joint_motion == false && _bool_hands_motion == true)
	{
		_control_mode = 4;

		_start_time = 5.0;
		_end_time = 6.0;

		_R_goal_right_hand = Model._R_right_hand;
		_R_goal_left_hand = Model._R_left_hand;
		/*
		_R_goal_right_hand.setZero();
		_R_goal_right_hand(0, 2) = 1.0;
		_R_goal_right_hand(1, 1) = -1.0;
		_R_goal_right_hand(2, 0) = 1.0;
		_R_goal_left_hand.setZero();
		_R_goal_left_hand(0, 2) = 1.0;
		_R_goal_left_hand(1, 1) = 1.0;
		_R_goal_left_hand(2, 0) = -1.0;
		*/
		_x_goal_left_hand = Model._x_left_hand;
		//_x_goal_left_hand(1) = Model._x_left_hand(1) - 0.2;
		_x_goal_left_hand(2) = Model._x_left_hand(2) - 0.1;
		LeftHandPosTrajectory.reset_initial(_start_time, Model._x_left_hand, Model._xdot_left_hand.head(3));
		LeftHandPosTrajectory.update_goal(_x_goal_left_hand, _xdot_goal_left_hand, _end_time);
		_x_goal_right_hand = Model._x_right_hand;
		//_x_goal_right_hand(1) = Model._x_right_hand(1) + 0.2;
		_x_goal_right_hand(2) = Model._x_right_hand(2) - 0.1;
		RightHandPosTrajectory.reset_initial(_start_time, Model._x_right_hand, Model._xdot_right_hand.head(3));
		RightHandPosTrajectory.update_goal(_x_goal_right_hand, _xdot_goal_right_hand, _end_time);

		_bool_hands_motion = false;

		cout << "Operational Sapce Control - Hand Motions, t = " << _t << " s" << endl;
	}
	else if (_t >= 6.0 && _bool_joint_motion == false && _bool_hands_motion == false)
	{
		_control_mode = 4;

		_start_time = 6.0;
		_end_time = 6.1;

		_R_goal_right_hand = Model._R_right_hand;
		_R_goal_left_hand = Model._R_left_hand;
		/*
		_R_goal_right_hand.setZero();
		_R_goal_right_hand(0, 2) = 1.0;
		_R_goal_right_hand(1, 1) = -1.0;
		_R_goal_right_hand(2, 0) = 1.0;
		_R_goal_left_hand.setZero();
		_R_goal_left_hand(0, 2) = 1.0;
		_R_goal_left_hand(1, 1) = 1.0;
		_R_goal_left_hand(2, 0) = -1.0;
		*/

		_x_goal_left_hand = Model._x_left_hand;
		LeftHandPosTrajectory.reset_initial(_start_time, Model._x_left_hand, Model._xdot_left_hand.head(3));
		LeftHandPosTrajectory.update_goal(_x_goal_left_hand, _xdot_goal_left_hand, _end_time);
		_x_goal_right_hand = Model._x_right_hand;
		RightHandPosTrajectory.reset_initial(_start_time, Model._x_right_hand, Model._xdot_right_hand.head(3));
		RightHandPosTrajectory.update_goal(_x_goal_right_hand, _xdot_goal_right_hand, _end_time);

		_bool_hands_motion = true;

		cout << "Operational Sapce Control - Regulate Hand Positions, t = " << _t << " s" << endl;
	}
	
	//Control
	if (_control_mode == 1) //joint space control
	{
		JointTrajectory.update_time(_t);
		_q_des = JointTrajectory.position_cubicSpline();
		_qdot_des = JointTrajectory.velocity_cubicSpline();

		JointControl();
	}
	else if (_control_mode == 2 || _control_mode == 3 || _control_mode == 4) //task space hand control
	{
		LeftHandPosTrajectory.update_time(_t);
		_x_des_left_hand = LeftHandPosTrajectory.position_cubicSpline();
		_xdot_des_left_hand = LeftHandPosTrajectory.velocity_cubicSpline();

		RightHandPosTrajectory.update_time(_t);
		_x_des_right_hand = RightHandPosTrajectory.position_cubicSpline();
		_xdot_des_right_hand = RightHandPosTrajectory.velocity_cubicSpline();

		if (_control_mode == 2)
		{
			OperationalSpaceControl();
		}
		else if (_control_mode == 3)
		{
			HQPTaskSpaceControl();
		}
		else if (_control_mode == 4)
		{
			ReducedHQPTaskSpaceControl();
		}
		
	}
	
}


void CController::ModelUpdate()
{
	Model.update_kinematics(_q, _qdot);
	Model.update_dynamics();
	Model.calculate_EE_Jacobians();
	Model.calculate_EE_positions_orientations();
	Model.calculate_EE_velocity();

	//set Jacobian
	_J_hands.block<6, 15>(0, 0) = Model._J_left_hand;
	_J_hands.block<6, 15>(6, 0) = Model._J_right_hand;
	_J_T_hands = _J_hands.transpose();

	_J_ori_hands.block<3, 15>(0, 0) = Model._J_left_hand_ori;
	_J_ori_hands.block<3, 15>(3, 0) = Model._J_right_hand_ori;
	_J_ori_T_hands = _J_ori_hands.transpose();
	_J_pos_hands.block<3, 15>(0, 0) = Model._J_left_hand_pos;
	_J_pos_hands.block<3, 15>(3, 0) = Model._J_right_hand_pos;
	_J_pos_T_hands = _J_pos_hands.transpose();

	//calc Jacobian dot (with lowpass filter)	
	for (int i = 0; i < 12; i++)
	{
		for (int j = 0; j < 15; j++)
		{
			_Jdot_hands(i,j) = CustomMath::VelLowpassFilter(_dt, 2.0 * PI * 20.0, _pre_J_hands(i,j), _J_hands(i,j), _pre_Jdot_hands(i,j)); //low-pass filter

			_pre_J_hands(i, j) = _J_hands(i, j);
			_pre_Jdot_hands(i, j) = _Jdot_hands(i, j);
		}		
	}
	_Jdot_qdot = _Jdot_hands * _qdot;
}


void CController::JointControl()
{
	_torque.setZero();
	_torque = Model._A*(_kpj*(_q_des - _q) + _kdj*(_qdot_des - _qdot) ) + Model._bg;	
}

void CController::OperationalSpaceControl()
{
	_torque.setZero();	
	_kp = 100.0;
	_kd = 20.0;

	_x_err_left_hand = _x_des_left_hand - Model._x_left_hand;
	_R_err_left_hand.setZero();
	_R_err_left_hand = -CustomMath::getPhi(Model._R_left_hand, _R_goal_left_hand);
	_x_err_right_hand = _x_des_right_hand - Model._x_right_hand;
	_R_err_right_hand.setZero();
	_R_err_right_hand = -CustomMath::getPhi(Model._R_right_hand, _R_goal_right_hand);

	_xdot_err_left_hand = _xdot_des_left_hand - Model._xdot_left_hand.segment(0, 3);
	_Rdot_err_left_hand = -Model._xdot_left_hand.segment(3, 3); //only daming for orientation
	_xdot_err_right_hand = _xdot_des_right_hand - Model._xdot_right_hand.segment(0, 3);
	_Rdot_err_right_hand = -Model._xdot_right_hand.segment(3, 3); //only daming for orientation	

	cout << _x_err_left_hand.transpose() << endl;
	
	// set 1
	// 1st: hands pos and ori, 2nd: joint dampings
	_Lambda_hands.setZero();
	_Lambda_hands = CustomMath::pseudoInverseQR(_J_T_hands) * Model._A * CustomMath::pseudoInverseQR(_J_hands);	
	_Null_hands = _Id_15 - _J_T_hands * _Lambda_hands * _J_hands * Model._A.inverse();	

	//cout << _Null_hands << endl << endl;

	_xddot_star.segment(0, 3) = _kp * _x_err_left_hand + _kd * _xdot_err_left_hand;//left hand position control
	_xddot_star.segment(3, 3) = _kp * _R_err_left_hand + _kd * _Rdot_err_left_hand;//left hand orientation control
	_xddot_star.segment(6, 3) = _kp * _x_err_right_hand + _kd * _xdot_err_right_hand;//right hand position control
	_xddot_star.segment(9, 3) = _kp * _R_err_right_hand + _kd * _Rdot_err_right_hand;//right hand orientation control	
	
	_torque = _J_T_hands * _Lambda_hands * _xddot_star + _Null_hands * (Model._A * (-_kdj * _qdot)) + Model._bg;	

	// set 2
	// 1st: hands ori, 2nd: hands pos, 3rd: joint dampings
	//_Lambda_ori_hands = CustomMath::pseudoInverseQR(_J_ori_T_hands) * Model._A * CustomMath::pseudoInverseQR(_J_ori_hands);
	//_Lambda_pos_hands = CustomMath::pseudoInverseQR(_J_pos_T_hands) * Model._A * CustomMath::pseudoInverseQR(_J_pos_hands);

	//_Null_hands_ori = _Id_15 - _J_ori_T_hands * _Lambda_ori_hands * _J_ori_hands * Model._A.inverse();
	//_Null_hands_pos = _Id_15 - _J_pos_T_hands * _Lambda_pos_hands * _J_pos_hands * Model._A.inverse();

	//VectorXd xddot_star_hand_ori(6);
	//VectorXd xddot_star_hand_pos(6);
	//xddot_star_hand_ori.segment(0, 3) = _kp * _R_err_left_hand + _kd * _Rdot_err_left_hand;//left hand orientation control
	//xddot_star_hand_ori.segment(3, 3) = _kp * _R_err_right_hand + _kd * _Rdot_err_right_hand;//right hand orientation control
	//xddot_star_hand_pos.segment(0, 3) = _kp * _x_err_left_hand + _kd * _xdot_err_left_hand;//left hand position control
	//xddot_star_hand_pos.segment(3, 3) = _kp * _x_err_right_hand + _kd * _xdot_err_right_hand;//right hand position control	

	//cout << xddot_star_hand_ori.transpose() << endl << endl;;
	//xddot_star_hand_ori.setZero();
	//cout << _Lambda_ori_hands << endl << endl;
	//cout <<  _qdot.transpose() << endl;
	//cout << Model._A << endl;
	//cout << (Model._A * (-_kdj * _qdot)).transpose() << endl;
	//cout << (_Null_hands_ori * Model._A * (-_kdj * _qdot)).transpose() << endl << endl;
	//xddot_star_hand_ori.setZero();
	//_Null_hands_ori.setIdentity();
		
	//_torque = Model._bg +_J_ori_T_hands * _Lambda_ori_hands * xddot_star_hand_ori +_Null_hands_ori * (_J_pos_T_hands * _Lambda_pos_hands * xddot_star_hand_pos + _Null_hands_pos * Model._A * (-_kdj * _qdot));// *(_J_pos_T_hands * _Lambda_pos_hands * xddot_star_hand_pos + _Null_hands_pos * (Model._A * (-_kdj * _qdot)));
	//cout << _torque.transpose() << endl<<endl;
	
	
}

void CController::HQPTaskSpaceControl()
{
	_torque.setZero();	

	_kp = 100.0;
	_kd = 20.0;

	_x_err_left_hand = _x_des_left_hand - Model._x_left_hand;
	_R_err_left_hand.setZero();
	_R_err_left_hand = -CustomMath::getPhi(Model._R_left_hand, _R_goal_left_hand);
	_x_err_right_hand = _x_des_right_hand - Model._x_right_hand;
	_R_err_right_hand.setZero();
	_R_err_right_hand = -CustomMath::getPhi(Model._R_right_hand, _R_goal_right_hand);


	_xdot_err_left_hand = _xdot_des_left_hand - Model._xdot_left_hand.segment(0, 3);
	_Rdot_err_left_hand = -Model._xdot_left_hand.segment(3, 3); //only daming for orientation
	_xdot_err_right_hand = _xdot_des_right_hand - Model._xdot_right_hand.segment(0, 3);
	_Rdot_err_right_hand = -Model._xdot_right_hand.segment(3, 3); //only daming for orientation	

	_xddot_star.segment(0, 3) = _kp * _x_err_left_hand + _kd * _xdot_err_left_hand;//left hand position control
	_xddot_star.segment(3, 3) = _kp * _R_err_left_hand + _kd * _Rdot_err_left_hand;//left hand orientation control
	_xddot_star.segment(6, 3) = _kp * _x_err_right_hand + _kd * _xdot_err_right_hand;//right hand position control
	_xddot_star.segment(9, 3) = _kp * _R_err_right_hand + _kd * _Rdot_err_right_hand;//right hand orientation control

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// Solve HQP   /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	double threshold = 0.0001;
	int max_iter = 1000;			
	//first priority task QP ////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	//set cost function x^T*H*x + g	
	_H1.setZero();
	for (int i = 0; i < 30; i++)
	{
		_H1(i, i) = 0.00001;
	}
	for (int i = 30; i < 42; i++)
	{
		_H1(i, i) = 1.0;
	}
	//_H1.block<12, 12>(30, 30) = _Id_12;
	_g1.setZero();	
	HQP_P1.UpdateMinProblem(_H1,_g1);
	
	//set A*x <= b	
	_A1.setZero();
	_lbA1.setZero();
	_ubA1.setZero();	
	_A1.block<15, 15>(0, 0) = Model._A;
	_A1.block<15, 15>(0, 15) = -_Id_15;
	_A1.block<12, 15>(15, 0) = _J_hands;
	_A1.block<12, 12>(15, 30) = -_Id_12;
	for (int i = 0; i < 15; i++)
	{
		_lbA1(i) = -Model._bg(i) - threshold;
		_ubA1(i) = -Model._bg(i) + threshold;
	}
	for (int i = 0; i < 12; i++)
	{
		_lbA1(i + 15) = -_Jdot_qdot(i) + _xddot_star(i) - threshold;
		_ubA1(i + 15) = -_Jdot_qdot(i) + _xddot_star(i) + threshold;
	}	
	HQP_P1.UpdateSubjectToAx(_A1, _lbA1, _ubA1);
	
	//set lb <= x <= ub	
	_lb1.setZero();
	_ub1.setZero();
	//joint acceleration limit (for joint position and velocity)
	for (int i = 0; i < 15; i++)
	{
		_lb1(i) = -500.0;
		_ub1(i) = 500.0;
	}
	//joint torque limit
	for (int i = 0; i < 15; i++)
	{
		_lb1(i + 15) = -2000.0;
		_ub1(i + 15) = 2000.0;
	}
	//task limit	
	for (int i = 0; i < 12; i++)
	{
		_lb1(i + 30) = -500.0;
		_ub1(i + 30) = 500.0;
	}
	HQP_P1.UpdateSubjectToX(_lb1, _ub1);

	//Solve
	HQP_P1.EnableEqualityCondition(0.0001);	
	HQP_P1.SolveQPoases(max_iter);
	_torque = HQP_P1._Xopt.segment(15, 15);
	//cout << "slack: " << HQP_P1._Xopt.segment(30, 12).transpose() << endl;

	//second priority task QP ////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	
	//set cost function x^T*H*x + g
	_H2.setZero();
	for (int i = 0; i < 30; i++) //torque
	{
		_H2(i, i) = 0.00001;
	}
	for (int i = 30; i < 45; i++) //slack
	{
		_H2(i, i) = 100000.0;
	}
	_H2.block<15,15>(30,30) = Model._A;  //acceleration
	_g2.setZero();
	HQP_P2.UpdateMinProblem(_H2, _g2);

	//set A*x <= b	
	_A2.setZero();
	_lbA2.setZero();
	_ubA2.setZero();
	_A2.block<15, 15>(0, 0) = Model._A;
	_A2.block<15, 15>(0, 15) = -_Id_15;
	_A2.block<15, 15>(15, 0) = _Id_15;
	_A2.block<15, 15>(15, 30) = -_Id_15;
	_A2.block<12, 15>(30, 0) = _J_hands;
	
	VectorXd joint_acc_des(15);
	joint_acc_des.setZero();
	_kdj = 40.0;
	joint_acc_des = -_kdj * _qdot;
	//joint_acc_des(0) = 400.0 * (_q_home(0)- _q(0)) - _kdj * _qdot(0);
	for (int i = 0; i < 15; i++)
	{
		_lbA2(i) = -Model._bg(i) - threshold;
		_ubA2(i) = -Model._bg(i) + threshold;
		_lbA2(i + 15) = joint_acc_des(i) - threshold;
		_ubA2(i + 15) = joint_acc_des(i) + threshold;
	}
	for (int i = 0; i < 12; i++)
	{
		_lbA2(i + 30) = -_Jdot_qdot(i) + _xddot_star(i) + HQP_P1._Xopt(i+30) - threshold;
		_ubA2(i + 30) = -_Jdot_qdot(i) + _xddot_star(i) + HQP_P1._Xopt(i+30) + threshold;
	}
	HQP_P2.UpdateSubjectToAx(_A2, _lbA2, _ubA2);

	//set lb <= x <= ub
	_lb2.setZero();
	_ub2.setZero();
	//joint acceleration limit (for joint position and velocity)
	for (int i = 0; i < 15; i++)
	{
		_lb2(i) = -50.0;
		_ub2(i) = 50.0;
	}
	//joint torque limit
	for (int i = 0; i < 15; i++)
	{
		_lb2(i + 15) = -500.0;
		_ub2(i + 15) = 500.0;
	}
	//task limit	
	for (int i = 0; i < 15; i++)
	{
		_lb2(i + 30) = -500.0;
		_ub2(i + 30) = 500.0;
	}
	HQP_P2.UpdateSubjectToX(_lb2, _ub2);	
	//Solve
	HQP_P2.EnableEqualityCondition(0.0001);	
	HQP_P2.SolveQPoases(max_iter);
	_torque = HQP_P2._Xopt.segment(15, 15);
}

void CController::ReducedHQPTaskSpaceControl()
{
	_torque.setZero();

	_kp = 100.0;
	_kd = 20.0;

	_x_err_left_hand = _x_des_left_hand - Model._x_left_hand;
	_R_err_left_hand = -CustomMath::getPhi(Model._R_left_hand, _R_goal_left_hand);	
	_x_err_right_hand = _x_des_right_hand - Model._x_right_hand;
	_R_err_right_hand = -CustomMath::getPhi(Model._R_right_hand, _R_goal_right_hand);	

	_xdot_err_left_hand = _xdot_des_left_hand - Model._xdot_left_hand.segment(0, 3);
	_Rdot_err_left_hand = -Model._xdot_left_hand.segment(3, 3); //only daming for orientation
	_xdot_err_right_hand = _xdot_des_right_hand - Model._xdot_right_hand.segment(0, 3);
	_Rdot_err_right_hand = -Model._xdot_right_hand.segment(3, 3); //only daming for orientation	

	_xddot_star.segment(0, 3) = _kp * _x_err_left_hand + _kd * _xdot_err_left_hand; //left hand position control
	_xddot_star.segment(3, 3) = _kp * _R_err_left_hand + _kd * _Rdot_err_left_hand; //left hand orientation control
	_xddot_star.segment(6, 3) = _kp * _x_err_right_hand + _kd * _xdot_err_right_hand; //right hand position control
	_xddot_star.segment(9, 3) = _kp * _R_err_right_hand + _kd * _Rdot_err_right_hand; //right hand orientation control

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// Solve rHQP   /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	double threshold = 0.0001;
	int max_iter = 1000;
	//first priority task QP ////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	//set cost function x^T*H*x + g	
	_rH1.setZero();
	for (int i = 0; i < 15; i++)
	{
		_rH1(i, i) = 0.00001;
	}
	for (int i = 15; i < 27; i++)
	{
		_rH1(i, i) = 1.0;
	}	
	_rg1.setZero();
	rHQP_P1.UpdateMinProblem(_rH1, _rg1);

	MatrixXd J_Ainv(12, 15);
	J_Ainv = _J_hands * Model._A.inverse();

	//set A*x <= b	
	_rA1.setZero();
	_rlbA1.setZero();
	_rubA1.setZero();
	_rA1.block<12, 15>(0, 0) = J_Ainv;
	_rA1.block<12, 12>(0, 15) = -_Id_12;

	for (int i = 0; i < 12; i++)
	{
		_rlbA1(i) = - _Jdot_qdot(i) + _xddot_star(i) - threshold;
		_rubA1(i) = - _Jdot_qdot(i) + _xddot_star(i) + threshold;
	}
	rHQP_P1.UpdateSubjectToAx(_rA1, _rlbA1, _rubA1);

	//set lb <= x <= ub	
	_rlb1.setZero();
	_rub1.setZero();
	//joint torque limit
	for (int i = 0; i < 15; i++)
	{
		_rlb1(i) = Model._min_joint_torque(i) - Model._bg(i);
		_rub1(i) = Model._max_joint_torque(i) - Model._bg(i);
		
		if (_qdot(i) >= Model._max_joint_velocity(i) * 0.8) //0.8 is margin coefficient
		{
			_rlb1(i) = Model._A(i, i) * 20.0 * (Model._max_joint_velocity(i) * 0.8 - _qdot(i)) - threshold;
			_rub1(i) = Model._A(i, i) * 20.0 * (Model._max_joint_velocity(i) * 0.8 - _qdot(i)) + threshold;
		}
		else if (_qdot(i) <= Model._min_joint_velocity(i) * 0.8)
		{
			_rlb1(i) = Model._A(i, i) * 20.0 * (Model._min_joint_velocity(i) * 0.8 - _qdot(i)) - threshold;
			_rub1(i) = Model._A(i, i) * 20.0 * (Model._min_joint_velocity(i) * 0.8 - _qdot(i)) + threshold;
		}

		double k_tanh = 5.0;
		if (abs(Model._min_joint_position(i) - _q(i)) <= abs(Model._max_joint_position(i) - _q(i)))
		{
			if (_q(i) > Model._min_joint_position(i))
			{
				_rlb1(i) = -(Model._max_joint_torque(i) - Model._min_joint_torque(i)) * tanh(k_tanh * (_q(i) - Model._min_joint_position(i))) + Model._max_joint_torque(i);
			}
			else
			{
				_rlb1(i) = Model._max_joint_torque(i) - 0.05;
			}
			_rub1(i) = Model._max_joint_torque(i);
		}
		else
		{
			if (_q(i) < Model._max_joint_position(i))
			{
				_rub1(i) = -(Model._max_joint_torque(i) - Model._min_joint_torque(i)) * tanh(k_tanh * (_q(i) - Model._max_joint_position(i))) + Model._min_joint_torque(i);
			}
			else
			{
				_rub1(i) = Model._min_joint_torque(i) + 0.05;
			}
			_rlb1(i) = Model._min_joint_torque(i);
		}
	}
	//task limit	
	for (int i = 0; i < 12; i++)
	{
		_rlb1(i + 15) = -1000.0;
		_rub1(i + 15) = 1000.0;
	}
	rHQP_P1.UpdateSubjectToX(_rlb1, _rub1);

	//Solve
	rHQP_P1.EnableEqualityCondition(0.0001);
	rHQP_P1.SolveQPoases(max_iter);
	//_torque = rHQP_P1._Xopt.segment(0, 15);

	//second priority task QP ////////////////////////////////////////////////////////////////////////////////////////////////////////////////	
	//set cost function x^T*H*x + g	
	_rH2.setZero();
	for (int i = 0; i < 15; i++)
	{
		_rH2(i, i) = 0.00001;
	}
	for (int i = 15; i < 30; i++)
	{
		_rH2(i, i) = 1.0;
	}
	_rH2.block<15, 15>(15, 15) = Model._A;
	_rg2.setZero();
	rHQP_P2.UpdateMinProblem(_rH2, _rg2);

	//set A*x <= b	
	_rA2.setZero();
	_rlbA2.setZero();
	_rubA2.setZero();
	_rA2.block<15, 15>(0, 0) = Model._A.inverse();
	_rA2.block<15, 15>(0, 15) = -_Id_15;
	_rA2.block<12, 15>(15, 0) = J_Ainv;	

	VectorXd joint_acc_des(15);
	joint_acc_des.setZero();
	_kdj = 40.0;
	joint_acc_des = -_kdj * _qdot;
	//joint_acc_des(0) = 400.0 * (_q_home(0)- _q(0)) - _kdj * _qdot(0);

	for (int i = 0; i < 15; i++)
	{
		_rlbA2(i) = joint_acc_des(i) - threshold;
		_rubA2(i) = joint_acc_des(i) + threshold;
	}
	for (int i = 0; i < 12; i++)
	{
		_rlbA2(i + 15) = _Jdot_qdot(i) + _xddot_star(i) + rHQP_P1._Xopt(i + 15) - threshold;
		_rubA2(i + 15) = _Jdot_qdot(i) + _xddot_star(i) + rHQP_P1._Xopt(i + 15) + threshold;
	}
	rHQP_P2.UpdateSubjectToAx(_rA2, _rlbA2, _rubA2);

	//set lb <= x <= ub
	_rlb2.setZero();
	_rub2.setZero();
	
	//joint torque limit
	for (int i = 0; i < 15; i++)
	{
		_rlb2(i) = Model._min_joint_torque(i) - Model._bg(i);
		_rub2(i) = Model._max_joint_torque(i) - Model._bg(i);
		
		if (_qdot(i) >= Model._max_joint_velocity(i) * 0.8) //0.8 is margin coefficient
		{
			_rlb2(i) = Model._A(i, i) * 20.0 * (Model._max_joint_velocity(i) * 0.8 - _qdot(i)) - threshold;
			_rub2(i) = Model._A(i, i) * 20.0 * (Model._max_joint_velocity(i) * 0.8 - _qdot(i)) + threshold;
			cout << "Max joint " << i <<", qdot" << _qdot(i) << "vel d" << Model._max_joint_velocity(i) * 0.8 << endl;
		}
		else if (_qdot(i) <= Model._min_joint_velocity(i) * 0.8)
		{
			_rlb2(i) = Model._A(i, i) * 20.0 * (Model._min_joint_velocity(i) * 0.8 - _qdot(i)) - threshold;
			_rub2(i) = Model._A(i, i) * 20.0 * (Model._min_joint_velocity(i) * 0.8 - _qdot(i)) + threshold;
			cout << "Min joint " << i << ", qdot" << _qdot(i) << "vel d" << Model._min_joint_velocity(i) * 0.8 << endl;
		}

		double k_tanh = 5.0;
		if(abs(Model._min_joint_position(i) - _q(i)) <= abs(Model._max_joint_position(i) - _q(i)))
		{
		  if(_q(i) > Model._min_joint_position(i))
		  {
			  _rlb2(i) = -(Model._max_joint_torque(i)- Model._min_joint_torque(i)) * tanh(k_tanh*(_q(i)- Model._min_joint_position(i))) + Model._max_joint_torque(i);
		  }
		  else
		  {
			  _rlb2(i) = Model._max_joint_torque(i) - 0.05;
		  }
		  _rub2(i) = Model._max_joint_torque(i);
		}
		else
		{
		  if(_q(i) < Model._max_joint_position(i))
		  {
			  _rub2(i) = -(Model._max_joint_torque(i)- Model._min_joint_torque(i)) *tanh(k_tanh*(_q(i)- Model._max_joint_position(i))) + Model._min_joint_torque(i);
		  }
		  else
		  {
			  _rub2(i) = Model._min_joint_torque(i) + 0.05;
		  }
		  _rlb2(i) = Model._min_joint_torque(i);
		}
	}
	//task limit
	for (int i = 0; i < 15; i++)
	{
		_rlb2(i + 15) = -1000.0;
		_rub2(i + 15) = 1000.0;
	}
	rHQP_P2.UpdateSubjectToX(_rlb2, _rub2);

	//Solve
	rHQP_P2.EnableEqualityCondition(0.0001);
	rHQP_P2.SolveQPoases(max_iter);
	_torque = rHQP_P2._Xopt.segment(0, 15) + Model._bg;
	//cout << _torque.transpose() << endl;
}

void CController::Initialize()
{
	_control_mode = 1; //1: joint space, 2: operational space

	_t = 0.0;

	_pre_t = 0.0;
	_dt = 0.0;

	_q.setZero(_dofj);
	_qdot.setZero(_dofj);
	_torque.setZero(_dofj);

	_pre_q.setZero(_dofj);
	_pre_qdot.setZero(_dofj);

	_q_home.setZero(_dofj);
	_q_home(0) = -0.2;
	_q_home(1) = -30.0 * DEG2RAD; //LShP
	_q_home(8) = 30.0 * DEG2RAD; //RShP
	_q_home(2) = 20.0 * DEG2RAD; //LShR
	_q_home(9) = -20.0 * DEG2RAD; //RShR
	_q_home(4) = 80.0 * DEG2RAD; //LElP
	_q_home(11) = -80.0 * DEG2RAD; //RElP
	_q_home(6) = -60.0 * DEG2RAD; //LWrP
	_q_home(13) = 60.0 * DEG2RAD; //RWrP

	_start_time = 0.0;
	_end_time = 0.0;

	_q_des.setZero(_dofj);
	_qdot_des.setZero(_dofj);
	_q_goal.setZero(_dofj);
	_qdot_goal.setZero(_dofj);

	_x_goal_left_hand.setZero();
	_xdot_goal_left_hand.setZero();
	_x_des_left_hand.setZero();
	_xdot_des_left_hand.setZero();
	_x_goal_right_hand.setZero();
	_xdot_goal_right_hand.setZero();
	_x_des_right_hand.setZero();
	_xdot_des_right_hand.setZero();

	_R_goal_left_hand.setIdentity();
	_R_goal_right_hand.setIdentity();

	_xddot_star.setZero(12);
	_x_err_left_hand.setZero();
	_x_err_right_hand.setZero();
	_xdot_err_left_hand.setZero();
	_xdot_err_right_hand.setZero();
	_R_err_left_hand.setZero();
	_R_err_right_hand.setZero();
	_Rdot_err_left_hand.setZero();
	_Rdot_err_right_hand.setZero();

	_kpj = 400.0;
	_kdj = 40.0;
	_kp = 400.0;
	_kd = 40.0;

	_J_hands.setZero(12, 15);
	_Jdot_hands.setZero(12, 15);
	_Jdot_qdot.setZero(12);
	_pre_J_hands.setZero(12, 15);
	_pre_Jdot_hands.setZero(12, 15);
	_J_T_hands.setZero(15, 12);
	_Lambda_hands.setZero(12, 12);
	_Null_hands.setZero(15, 15);

	_J_pos_hands.setZero(6, 15);
	_J_pos_T_hands.setZero(15, 6);
	_J_ori_hands.setZero(6, 15);
	_J_ori_T_hands.setZero(15, 6);
	_Lambda_pos_hands.setZero(6, 6);
	_Lambda_ori_hands.setZero(6, 6);
	_Null_hands_pos.setZero(15, 15);
	_Null_hands_ori.setZero(15, 15);


	_Id_15.setIdentity(15, 15);
	_Id_12.setIdentity(12, 12);

	_bool_joint_motion = false;
	_bool_hands_motion = false;

	_zero_vec.setZero(_dofj);

	JointTrajectory.set_size(15);
	LeftHandPosTrajectory.set_size(3);
	RightHandPosTrajectory.set_size(3);

	//HQP
	HQP_P1.InitializeProblemSize(42,27); //variable size = (joint dof)*2+(task dof), constraint size = (joint dof) + (task dof) 
	_H1.setZero(HQP_P1._num_var, HQP_P1._num_var);
	_g1.setZero(HQP_P1._num_var);
	_A1.setZero(HQP_P1._num_cons, HQP_P1._num_var);
	_lbA1.setZero(HQP_P1._num_cons);
	_ubA1.setZero(HQP_P1._num_cons);
	_lb1.setZero(HQP_P1._num_var);
	_ub1.setZero(HQP_P1._num_var);
	HQP_P2.InitializeProblemSize(45,42); //variable size = (joint dof)*2+(task dof), constraint size = (joint dof) + (1st prioirty task dof)  + (2nd prioirty task dof) 
	_H2.setZero(HQP_P2._num_var, HQP_P2._num_var);
	_g2.setZero(HQP_P2._num_var);
	_A2.setZero(HQP_P2._num_cons, HQP_P2._num_var);
	_lbA2.setZero(HQP_P2._num_cons);
	_ubA2.setZero(HQP_P2._num_cons);
	_lb2.setZero(HQP_P2._num_var);
	_ub2.setZero(HQP_P2._num_var);


	//rHQP
	rHQP_P1.InitializeProblemSize(27, 12); //variable size = (joint dof)+(task dof), constraint size =(task dof) 
	_rH1.setZero(rHQP_P1._num_var, HQP_P1._num_var);
	_rg1.setZero(rHQP_P1._num_var);
	_rA1.setZero(rHQP_P1._num_cons, HQP_P1._num_var);
	_rlbA1.setZero(rHQP_P1._num_cons);
	_rubA1.setZero(rHQP_P1._num_cons);
	_rlb1.setZero(rHQP_P1._num_var);
	_rub1.setZero(rHQP_P1._num_var);
	rHQP_P2.InitializeProblemSize(30, 27); //variable size = (joint dof)+(task dof), constraint size =(1st prioirty task dof)  + (2nd prioirty task dof) 
	_rH2.setZero(rHQP_P2._num_var, rHQP_P2._num_var);
	_rg2.setZero(rHQP_P2._num_var);
	_rA2.setZero(rHQP_P2._num_cons, rHQP_P2._num_var);
	_rlbA2.setZero(rHQP_P2._num_cons);
	_rubA2.setZero(rHQP_P2._num_cons);
	_rlb2.setZero(rHQP_P2._num_var);
	_rub2.setZero(rHQP_P2._num_var);

}