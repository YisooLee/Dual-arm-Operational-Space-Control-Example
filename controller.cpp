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
		_control_mode = 2;

		_start_time = 4.0;
		_end_time = 5.0;

		_x_goal_left_hand = Model._x_left_hand; //set as current state
		LeftHandPosTrajectory.reset_initial(_start_time, Model._x_left_hand, Model._xdot_left_hand.head(3));
		LeftHandPosTrajectory.update_goal(_x_goal_left_hand, _xdot_goal_left_hand, _end_time);
		_x_goal_right_hand = Model._x_right_hand; //set as current state
		RightHandPosTrajectory.reset_initial(_start_time, Model._x_right_hand, Model._xdot_right_hand.head(3));
		RightHandPosTrajectory.update_goal(_x_goal_right_hand, _xdot_goal_right_hand, _end_time);

		_bool_hands_motion = true;

		cout << "Start Operational Sapce Control - Regulate Initial Hand Positions, t = " << _t << " s" << endl;
	}
	else if (_t >= 5.0 && _t < 10.0 && _bool_joint_motion == false && _bool_hands_motion == true)
	{
		_control_mode = 2;

		_start_time = 5.0;
		_end_time = 10.0;

		_x_goal_left_hand = Model._x_left_hand;
		_x_goal_left_hand(1) = Model._x_left_hand(1) + 0.1;
		LeftHandPosTrajectory.reset_initial(_start_time, Model._x_left_hand, Model._xdot_left_hand.head(3));
		LeftHandPosTrajectory.update_goal(_x_goal_left_hand, _xdot_goal_left_hand, _end_time);
		_x_goal_right_hand = Model._x_right_hand;
		_x_goal_right_hand(1) = Model._x_right_hand(1) - 0.1;
		RightHandPosTrajectory.reset_initial(_start_time, Model._x_right_hand, Model._xdot_right_hand.head(3));
		RightHandPosTrajectory.update_goal(_x_goal_right_hand, _xdot_goal_right_hand, _end_time);

		_bool_hands_motion = false;

		cout << "Operational Sapce Control - Hand Motions, t = " << _t << " s" << endl;
	}
	else if (_t >= 10.0 && _bool_joint_motion == false && _bool_hands_motion == false)
	{
		_control_mode = 2;

		_start_time = 10.0;
		_end_time = 100.0;

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
	if (_control_mode == 1)
	{
		JointTrajectory.update_time(_t);
		_q_des = JointTrajectory.position_cubicSpline();
		_qdot_des = JointTrajectory.velocity_cubicSpline();

		JointControl();
	}
	else if (_control_mode == 2)
	{
		LeftHandPosTrajectory.update_time(_t);
		_x_des_left_hand = LeftHandPosTrajectory.position_cubicSpline();
		_xdot_des_left_hand = LeftHandPosTrajectory.velocity_cubicSpline();

		RightHandPosTrajectory.update_time(_t);
		_x_des_right_hand = RightHandPosTrajectory.position_cubicSpline();
		_xdot_des_right_hand = RightHandPosTrajectory.velocity_cubicSpline();

		OperationalSpaceControl();
	}
	
}


void CController::ModelUpdate()
{
	Model.update_kinematics(_q, _qdot);
	Model.update_dynamics();
	Model.calculate_EE_Jacobians();
	Model.calculate_EE_positions_orientations();
}


void CController::JointControl()
{
	_torque.setZero();
	_torque = Model._A*(_kpj*(_q_des - _q) + _kdj*(_qdot_des - _qdot) ) + Model._bg;	
}

void CController::OperationalSpaceControl()
{
	_torque.setZero();	
	_J_hands.block<6,15>(0,0) = Model._J_left_hand;
	_J_hands.block<6,15>(6,0) = Model._J_right_hand;
	_J_T_hands = _J_hands.transpose();	
	//_Lambda_hands = CustomMath::pseudoInverseQR(_J_T_hands)*Model._A* CustomMath::pseudoInverseQR(_J_hands);
	_Lambda_hands = CustomMath::pseudoInverseQR(_J_hands* Model._A.inverse()* _J_T_hands);// CustomMath::pseudoInverseQR(_J_T_hands)* Model._A* CustomMath::pseudoInverseQR(_J_hands);
	_Null_hands = _Id_15 - _J_T_hands * _Lambda_hands * _J_hands * Model._A.inverse();		
	_x_err_left_hand = _x_des_left_hand - Model._x_left_hand;
	_R_err_left_hand.setZero();
	_x_err_right_hand = _x_des_right_hand - Model._x_right_hand;
	_R_err_right_hand.setZero();	
	_xdot_err_left_hand = _xdot_des_left_hand  - Model._xdot_left_hand.segment(0, 3);
	_Rdot_err_left_hand = -Model._xdot_left_hand.segment(3, 3); //only daming for orientation
	_xdot_err_right_hand = _xdot_des_right_hand - Model._xdot_right_hand.segment(0, 3);
	_Rdot_err_right_hand = -Model._xdot_right_hand.segment(3, 3); //only daming for orientation	
	_xddot_star.segment(0, 3) = _kp * _x_err_left_hand + _kd * _xdot_err_left_hand;//left hand position control
	_xddot_star.segment(3, 3) = _kp * _R_err_left_hand + _kd * _Rdot_err_left_hand;//left hand orientation control
	_xddot_star.segment(6, 3) = _kp * _x_err_right_hand + _kd * _xdot_err_right_hand;//right hand position control
	_xddot_star.segment(9, 3) = _kp * _R_err_right_hand + _kd * _Rdot_err_right_hand;//right hand orientation control
		
	_torque = _J_T_hands * _Lambda_hands * _xddot_star + _Null_hands * (Model._A * (-_kdj * _qdot)) +  Model._bg;// _J_T_hands* _Lambda_hands* _xddot_star + _Null_hands * (Model._A * (-_kdj * _qdot)) + Model._bg;

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

	_x_goal_left_hand.setZero();
	_xdot_goal_left_hand.setZero();
	_x_des_right_hand.setZero();
	_xdot_des_right_hand.setZero();

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
	_J_T_hands.setZero(15, 12);
	_Lambda_hands.setZero(12, 12);
	_Null_hands.setZero(15, 15);
	_Id_15.setIdentity(15, 15);

	_bool_joint_motion = false;
	_bool_hands_motion = false;

	_zero_vec.setZero(_dofj);

	JointTrajectory.set_size(15);
	LeftHandPosTrajectory.set_size(3);
	RightHandPosTrajectory.set_size(3);

}