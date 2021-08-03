#pragma once
#ifndef __CONTROLLER_H
#define __CONTROLLER_H

#include <iostream>
#include <Eigen/Dense>
#include <rbdl/rbdl.h>
#include <rbdl/addons/urdfreader/urdfreader.h>

#include "trajectory.h"
#include "robotmodel.h"
#include "custommath.h"
#include "quadraticprogram.h"

//#define DEG2RAD (0.01745329251994329576923690768489)
//#define RAD2DEG 1.0/DEG2RAD

using namespace std;
using namespace Eigen;


class CController
{

public:
	CController(int JDOF);
	virtual ~CController();	
	void read(double time, double* q, double* qdot);
	//void write(double* qdes, double* qdotdes);
	void write(double* torque);
	void control_mujoco();

public:
	VectorXd _torque;
	VectorXd _q; //joint angle vector
	VectorXd _qdot; //joint velocity vector

private:
	double _t;
	bool _bool_init;
	double _init_t;
	double _dt;
	double _pre_t;
	int _dofj; //number of joint
	int _control_mode; //1: joint space, 2: operational space 
	VectorXd _zero_vec;	
	VectorXd _q_home;

	VectorXd _pre_q;
	VectorXd _pre_qdot;

	void Initialize();

	//plan
	void reset_target(double motion_time, VectorXd target_joint_position);
	void reset_target(double motion_time, Vector3d target_pos_lh, Vector3d target_ori_lh, Vector3d target_pos_rh, Vector3d target_ori_rh);
	void motionPlan();
	int _cnt_plan;
	VectorXd _time_plan;
	VectorXi _bool_plan;

	//controller
	double _kpj; //joint control P gain
	double _kdj; //joint control D gain	
	double _kp; //Operational space control P gain
	double _kd; //Operational space control D gain	
	void JointControl();
	void OperationalSpaceControl();
	void HQPTaskSpaceControl();
	void ReducedHQPTaskSpaceControl();

	//robot model
	CModel Model;
	void ModelUpdate();
	MatrixXd _J_hands; // 12x15
	MatrixXd _J_T_hands; // 15x12
	MatrixXd _Jdot_hands;
	MatrixXd _pre_J_hands;
	MatrixXd _pre_Jdot_hands;
	VectorXd _Jdot_qdot;

	MatrixXd _J_pos_hands; // 6x15
	MatrixXd _J_pos_T_hands; // 15x6
	MatrixXd _J_ori_hands; // 6x15
	MatrixXd _J_ori_T_hands; // 15x6	

	//operational space variables (two hand)
	VectorXd _x_left_hand; //state
	VectorXd _x_right_hand; //state
	VectorXd _xdot_left_hand; //state
	VectorXd _xdot_right_hand; //state

	MatrixXd _Lambda_hands; //inertia matri 12x12
	MatrixXd _Null_hands; //null space projection matrix 15x15	
	MatrixXd _Id_15, _Id_12;
	VectorXd _xddot_star; //12

	MatrixXd _Lambda_pos_hands; //inertia matri 6x6
	MatrixXd _Lambda_ori_hands; //inertia matri 6x6
	MatrixXd _Null_hands_ori; //null space projection matrix 15x15
	MatrixXd _Null_hands_pos; //null space projection matrix 15x15

	Vector3d _x_err_left_hand;
	Vector3d _x_err_right_hand;
	Vector3d _xdot_err_left_hand;
	Vector3d _xdot_err_right_hand;
	Vector3d _R_err_left_hand;
	Vector3d _R_err_right_hand;
	Vector3d _Rdot_err_left_hand;
	Vector3d _Rdot_err_right_hand;

	//HQP
	CQuadraticProgram HQP_P1; //first priority task - dual hand control
	CQuadraticProgram HQP_P2; //second priority task - joint damping
	MatrixXd _H1, _H2, _A1, _A2;
	VectorXd _g1, _g2, _lbA1, _lbA2, _ubA1, _ubA2, _lb1, _lb2, _ub1, _ub2;

	//Reduced HQP
	CQuadraticProgram rHQP_P1; //first priority task - dual hand control
	CQuadraticProgram rHQP_P2; //second priority task - joint damping
	MatrixXd _rH1, _rH2, _rA1, _rA2;
	VectorXd _rg1, _rg2, _rlbA1, _rlbA2, _rubA1, _rubA2, _rlb1, _rlb2, _rub1, _rub2;
	
	//motion trajectory
	double _start_time, _end_time, _motion_time;

	//joint space
	bool _bool_joint_motion;
	CTrajectory JointTrajectory; //size = joint dof
	VectorXd _q_goal;
	VectorXd _qdot_goal;
	VectorXd _q_des;//desired joint angle vector
	VectorXd _qdot_des;//desired joint velocity vector

	//operational space (two hand)
	bool _bool_ee_motion;
	CTrajectory RightHandTrajectory; //size = 6
	CTrajectory LeftHandTrajectory; //size = 6

	VectorXd _x_goal_left_hand;
	VectorXd _xdot_goal_left_hand;
	VectorXd _x_des_left_hand;
	VectorXd _xdot_des_left_hand;
	VectorXd _x_goal_right_hand;
	VectorXd _xdot_goal_right_hand;
	VectorXd _x_des_right_hand;
	VectorXd _xdot_des_right_hand;
	Matrix3d _R_des_left_hand;
	Matrix3d _R_des_right_hand;

	Vector3d _pos_goal_left_hand;
	Vector3d _rpy_goal_left_hand;
	Vector3d _pos_goal_right_hand;
	Vector3d _rpy_goal_right_hand;

	// addons  ////////////////////////////////////////////////////////////////////////////////////////	
	bool _bool_safemode;
	void safeModeReplaceTorque(bool bool_safemode);
	void safeWorkSpaceLimit();
	double _dist_shoulder_hand_left;
	double _dist_shoulder_hand_right;
	double _workspace_avoid_gain;
	Vector3d _dir_hand_to_shoulder_left;
	Vector3d _dir_hand_to_shoulder_right;
	Vector3d _acc_workspace_avoid_left;
	Vector3d _acc_workspace_avoid_right;


	////////////////////////////////////////////////////////////////////////////////////////////////////
	
};

#endif
