#include "robotmodel.h"
#define JDOF 15


CModel::CModel()
{
	Initialize();
}

CModel::~CModel()
{
}

void CModel::Initialize()
{
	_bool_model_update = false;
	_bool_kinematics_update = false;
	_bool_dynamics_update = false;
	_bool_Jacobian_update = false;

	_k = JDOF;
	_q.setZero(_k);
	_qdot.setZero(_k);
	_zero_vec_joint.setZero(_k);

	_id_left_hand = 8;
	_id_right_hand = 15;

	_A.setZero(_k,_k);
	_g.setZero(_k);
	_b.setZero(_k);
	_bg.setZero(_k);

	_J_tmp.setZero(6, _k);
	_J_left_hand.setZero(6,_k);
	_J_right_hand.setZero(6,_k);
	_position_local_task_left_hand.setZero();
	_position_local_task_right_hand.setZero();
	_x_left_hand.setZero();
	_x_right_hand.setZero();
	_R_left_hand.setZero();
	_R_right_hand.setZero();
	_xdot_left_hand.setZero(6);
	_xdot_right_hand.setZero(6);

	_global_rotate.setZero(6, 6);

	load_model();
}

void CModel::load_model()
{
	//read urdf model
	RigidBodyDynamics::Addons::URDFReadFromFile("../mujoco200_win64/model/dualarm.urdf", &_model, false, true);	
	//in this model
	//body id 1: body_link (trunk)
	//body id 8: LWrR_Link (left hand)
	//body id 15: RWrR_Link (right hand)

	cout << endl << endl << "Model Loaded for RBDL." << endl << "Total DoFs: " << _model.dof_count << endl << endl;
	if (_model.dof_count != _k)
	{
		cout << "Simulation model and RBDL model mismatch!!!" << endl << endl;
	}

	_global_rotate.setZero(); //x방향이 앞뒤 방향이 되도록 변경하기 위하여...	
	_global_rotate = CustomMath::GetBodyRotationMatrix(0.0, 0.0, -90.0 * DEG2RAD);

	_bool_model_update = true; //check model is updated

	cout << "Model Loading Complete." << endl << endl;
}

void CModel::update_kinematics(VectorXd & q, VectorXd & qdot)
{
	_q = q;
	_qdot = qdot;

	if (_bool_model_update == true)
	{
		RigidBodyDynamics::UpdateKinematicsCustom(_model, &_q, &_qdot, NULL); //update kinematics
	}
	else
	{
		cout << "Robot model is not ready. Please load model first." << endl << endl;
	}
	_bool_kinematics_update = true; //check kinematics is updated
}

void CModel::update_dynamics()
{
	if (_bool_kinematics_update == true)
	{
		RigidBodyDynamics::CompositeRigidBodyAlgorithm(_model, _q, _A, false); //update dynamics
		RigidBodyDynamics::InverseDynamics(_model, _q, _zero_vec_joint, _zero_vec_joint, _g, NULL); //get _g
		RigidBodyDynamics::InverseDynamics(_model, _q, _qdot, _zero_vec_joint, _bg, NULL); //get _g+_b
		_b = _bg - _g; //get _b
	}
	else
	{
		cout << "Robot kinematics is not ready. Please update kinematics first." << endl << endl;
	}
	_bool_dynamics_update = true; //check kinematics is updated
}

void CModel::calculate_EE_Jacobians()
{
	if (_bool_kinematics_update == true)
	{	
		_J_left_hand.setZero();
		_J_tmp.setZero();
		RigidBodyDynamics::CalcPointJacobian6D(_model, _q, _id_left_hand, _position_local_task_left_hand, _J_tmp, false); //left hand
		_J_left_hand.block<3, 15>(0, 0) = _global_rotate *_J_tmp.block<3, 15>(3, 0);
		_J_left_hand.block<3, 15>(3, 0) = _global_rotate *_J_tmp.block<3, 15>(0, 0);

		_J_right_hand.setZero();
		_J_tmp.setZero();
		RigidBodyDynamics::CalcPointJacobian6D(_model, _q, _id_right_hand, _position_local_task_right_hand, _J_tmp, false); //right hand
		_J_right_hand.block<3, 15>(0, 0) = _global_rotate * _J_tmp.block<3, 15>(3, 0);
		_J_right_hand.block<3, 15>(3, 0) = _global_rotate * _J_tmp.block<3, 15>(0, 0);

		_bool_Jacobian_update = true;
	}
	else
	{
		cout << "Robot kinematics is not ready. Please update kinematics first." << endl << endl;
	}
}

void CModel::calculate_EE_positions_orientations()
{
	if (_bool_kinematics_update == true)
	{
		_x_left_hand.setZero();
		_x_left_hand = _global_rotate*RigidBodyDynamics::CalcBodyToBaseCoordinates(_model, _q, _id_left_hand, _position_local_task_left_hand, false);
		_x_right_hand.setZero();
		_x_right_hand = _global_rotate*RigidBodyDynamics::CalcBodyToBaseCoordinates(_model, _q, _id_right_hand, _position_local_task_right_hand, false);

		_R_left_hand = _global_rotate*(RigidBodyDynamics::CalcBodyWorldOrientation(_model, _q, _id_left_hand, false)).transpose();
		_R_right_hand = _global_rotate*(RigidBodyDynamics::CalcBodyWorldOrientation(_model, _q, _id_right_hand, false)).transpose();
	}
	else
	{
		cout << "Robot kinematics is not ready. Please update kinematics first." << endl << endl;
	}
}

void CModel::calculate_EE_velocity()
{
	if (_bool_Jacobian_update == true)
	{
		_xdot_left_hand = _J_left_hand * _qdot;
		_xdot_right_hand = _J_right_hand * _qdot;
	}
	else
	{
		cout << "Jacobian matrices are not ready. Please calculate Jacobians first." << endl << endl;
	}
}