#pragma once
#ifndef __MODEL_H
#define __MODEL_H

#include <iostream>
#include <eigen-3.3.8/Eigen/Dense>
#include <rbdl/rbdl.h>
#include <rbdl/addons/urdfreader/urdfreader.h>
#include "custommath.h"

using namespace std;
using namespace Eigen;

class CModel
{
public:
	CModel();
	virtual ~CModel();

	void update_kinematics(VectorXd& q, VectorXd& qdot);
	void update_dynamics();
	void calculate_EE_positions_orientations();
	void calculate_EE_Jacobians();	
	void calculate_EE_velocity();

	RigidBodyDynamics::Model _model;
	MatrixXd _A; //inertia matrix
	VectorXd _g; //gravity force vector
	VectorXd _b; //Coriolis/centrifugal force vector
	VectorXd _bg; //Coriolis/centrifugal force vector + gravity force vector

	MatrixXd _J_left_hand, _J_right_hand;
	MatrixXd _J_left_hand_pos, _J_left_hand_ori, _J_right_hand_pos, _J_right_hand_ori;
	Vector3d _x_left_hand, _x_right_hand;
	Matrix3d _R_left_hand, _R_right_hand;
	VectorXd _xdot_left_hand, _xdot_right_hand;
	Vector3d _x_left_shoulder, _x_right_shoulder;

	VectorXd _max_joint_torque, _min_joint_torque, _max_joint_velocity, _min_joint_velocity, _max_joint_position, _min_joint_position, _max_ctrl_joint_torque, _min_ctrl_joint_torque;

	int _k;//joint number

private:
	void Initialize();
	void load_model();
	void set_robot_config();

	bool _bool_model_update, _bool_kinematics_update, _bool_dynamics_update, _bool_Jacobian_update;

	int _id_left_hand, _id_right_hand, _id_left_shoulder, _id_right_shoulder;

	VectorXd _q, _qdot;
	VectorXd _zero_vec_joint;

	Vector3d _position_local_task_left_hand;
	Vector3d _position_local_task_right_hand;
	Vector3d _position_local_zerovec;

	MatrixXd _J_tmp;	
	Matrix3d _global_rotate;	
};

#endif