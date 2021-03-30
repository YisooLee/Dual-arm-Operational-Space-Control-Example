#pragma once
#ifndef __CONTROLLER_H
#define __CONTROLLER_H

#include <iostream>
#include <eigen-3.3.8/Eigen/Dense>
#include <rbdl/rbdl.h>
#include <rbdl/addons/urdfreader/urdfreader.h>

#include "trajectory.h"
#include "robotmodel.h"
#include "custommath.h"

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
	void control_mujoco(double time);

public:
	VectorXd _torque;
	VectorXd _q; //joint angle vector
	VectorXd _qdot; //joint velocity vector

private:
	double _t;
	double _dt;
	double _pre_t;
	int _dofj; //number of joint
	int _control_mode; //1: joint space, 2: operational space 
	VectorXd _zero_vec;	
	VectorXd _q_home;

	VectorXd _pre_q;
	VectorXd _pre_qdot;

	void Initialize();

	//controller
	double _kpj; //joint control P gain
	double _kdj; //joint control D gain	
	double _kp; //Operational space control P gain
	double _kd; //Operational space control D gain	
	void JointControl();
	void OperationalSpaceControl();

	//robot model
	CModel Model;
	void ModelUpdate();

	//operational space variables (two hand)
	MatrixXd _Lambda_hands; //inertia matri 12x12
	MatrixXd _Null_hands; //null space projection matrix 15x15
	MatrixXd _J_hands; // 12x15
	MatrixXd _J_T_hands; // 15x12
	MatrixXd _Id_15;
	VectorXd _xddot_star; //12
	Vector3d _x_err_left_hand;
	Vector3d _x_err_right_hand;
	Vector3d _xdot_err_left_hand;
	Vector3d _xdot_err_right_hand;
	Vector3d _R_err_left_hand;
	Vector3d _R_err_right_hand;
	Vector3d _Rdot_err_left_hand;
	Vector3d _Rdot_err_right_hand;
	
	//motion trajectory
	double _start_time, _end_time;
	//joint space
	bool _bool_joint_motion;
	CTrajectory JointTrajectory; //size = joint dof
	VectorXd _q_goal;
	VectorXd _qdot_goal;
	VectorXd _q_des;//desired joint angle vector
	VectorXd _qdot_des;//desired joint velocity vector
	//operational space (two hand)
	bool _bool_hands_motion;
	CTrajectory RightHandPosTrajectory; //size = 3
	CTrajectory LeftHandPosTrajectory; //size = 3
	Vector3d _x_goal_left_hand;
	Vector3d _xdot_goal_left_hand;
	Vector3d _x_des_left_hand;
	Vector3d _xdot_des_left_hand;
	Vector3d _x_goal_right_hand;
	Vector3d _xdot_goal_right_hand;
	Vector3d _x_des_right_hand;
	Vector3d _xdot_des_right_hand;


};

#endif
