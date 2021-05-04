#ifndef CUSTOMMATH_H
#define CUSTOMMATH_H

#define DEG2RAD (0.01745329251994329576923690768489)
#define RAD2DEG 1/DEG2RAD
#define GRAVITY 9.80665
#define PI 3.1415926535897932384626433

#include "math.h"
#include <Eigen/Dense>
using namespace Eigen;


namespace CustomMath
{

// pseudo inverse
static MatrixXd pseudoInverseSVD(const MatrixXd& a, double epsilon = std::numeric_limits<double>::epsilon())
{
    JacobiSVD<MatrixXd> svd(a, Eigen::ComputeThinU | Eigen::ComputeThinV);
    double tolerance = epsilon * std::max(a.cols(), a.rows()) * svd.singularValues().array().abs()(0);

    MatrixXd ainv(a.cols(), a.rows());
    ainv.noalias() = svd.matrixV() * (svd.singularValues().array().abs() > tolerance).select(svd.singularValues().array().inverse(), 0).matrix().asDiagonal() * svd.matrixU().adjoint();

    return ainv;
}

static MatrixXd pseudoInverseQR(const MatrixXd& A)
{
    CompleteOrthogonalDecomposition<MatrixXd> cod(A);
    return cod.pseudoInverse();
}

static MatrixXd OneSidedInverse(const MatrixXd& A) //no use
{
    MatrixXd Ainv(A.cols(), A.rows());

    if (A.cols() > A.rows()) //left inverse
    {
        MatrixXd AAT(A.rows(), A.rows());
        AAT = A * A.transpose();
        MatrixXd AATinv(A.rows(), A.rows());
        AATinv = AAT.inverse();

        Ainv = A.transpose() * AATinv;
    }
    else //right inverse
    {
        MatrixXd ATA(A.cols(), A.cols());
        ATA = A.transpose() * A;
        MatrixXd ATAinv(A.cols(), A.cols());
        ATAinv = ATA.inverse();
        Ainv = ATAinv * A.transpose();
    }

    return Ainv;
}

static MatrixXd WeightedPseudoInverse(const MatrixXd& Q, const MatrixXd& W, const bool Is_W_fullrank)
{
    MatrixXd invQ(Q.cols(), Q.rows());
    MatrixXd transposeQ(Q.cols(), Q.rows());
    MatrixXd Winv(W.cols(), W.rows());
    if (Is_W_fullrank == true)
    {
        Winv = W.inverse();
    }
    else
    {
        Winv = pseudoInverseQR(W);
        //Winv = pseudoInverseSVD(W, 0.0001);
    }
    transposeQ = Q.transpose();
    MatrixXd tmp1(W.cols(), Q.rows());
    tmp1.noalias() = Winv * transposeQ;
    MatrixXd tmp2(Q.rows(), W.rows());
    tmp2.noalias() = Q * Winv;
    MatrixXd tmp3(Q.rows(), Q.rows());
    tmp3.noalias() = tmp2 * transposeQ;
    MatrixXd tmp3inv(Q.rows(), Q.rows());
    tmp3inv = tmp3.inverse();
    invQ.noalias() = tmp1 * tmp3inv;
    //invQ = Winv*transposeQ*pseudoInverseQR(Q*Winv*transposeQ);
    //invQ = Winv*transposeQ*pseudoInverseSVD(Q*Winv*transposeQ);

    return invQ;
}

static MatrixXd DampedWeightedPseudoInverse(const MatrixXd& Q, const MatrixXd& W, const bool Is_W_fullrank)
{
    MatrixXd invQ(Q.cols(), Q.rows());
    MatrixXd transposeQ(Q.cols(), Q.rows());
    MatrixXd Winv(W.cols(), W.rows());
    if (Is_W_fullrank == true)
    {
        Winv = W.inverse();
    }
    else
    {
        Winv = pseudoInverseQR(W);
        //Winv = pseudoInverseSVD(W, 0.0001);
    }
    transposeQ = Q.transpose();
    double sigma = 0.01;
    MatrixXd Id(Q.rows(), Q.rows());
    Id.setIdentity();
    //invQ = Winv*transposeQ*pseudoInverseQR(Q*Winv*transposeQ + sigma*Id);
    MatrixXd tmp1(W.cols(), Q.rows());
    tmp1.noalias() = Winv * transposeQ;
    MatrixXd tmp2(Q.rows(), W.rows());
    tmp2.noalias() = Q * Winv;
    MatrixXd tmp3(Q.rows(), Q.rows());
    tmp3.noalias() = tmp2 * transposeQ;
    MatrixXd tmp4(Q.rows(), Q.rows());
    tmp4.noalias() = tmp3 + sigma * Id;
    MatrixXd tmp4inv(Q.rows(), Q.rows());
    //tmp4inv = pseudoInverseSVD(tmp4, 0.0001);
    tmp4inv = pseudoInverseQR(tmp4);

    invQ.noalias() = tmp1 * tmp4inv;

    return invQ;
}

// lowpass filter
static double VelLowpassFilter(double rT, //delT
    double rWn, //cutoff freq (rad/sec)
    double rQ_pre, //previous position
    double rQ, //current position
    double rVel_pre //previous filtered velocity
)
{
    double rA;
    double rB;
    double rC;
    double rD;

    double rVel;

    rA = 2.0 * rWn;
    rB = (-2.0) * rWn;
    rC = 2.0 + rT * rWn;
    rD = rT * rWn - 2.0;

    rVel = ((-(rD)) * rVel_pre + (rA)*rQ + (rB)*rQ_pre) / (rC);

    return(rVel);
}

static double LowPassFilter(double dT, double Wc, double X, double preY) //sampling time, cutoff freq, input, previous output
{
    double tau = 1.0 / Wc;
    double y = tau / (tau + dT) * preY + dT / (tau + dT) * X;
    return y;
}



static Eigen::Vector3d GetBodyRotationAngle(Eigen::Matrix3d RotMat)
{
    double rollangle, pitchangle, yawangle;
    Eigen::Vector3d BodyAngle;
    BodyAngle.setZero();

    //  pitchangle = atan2(-RotMat(2,0),sqrt(RotMat(0,0)*RotMat(0,0)+RotMat(1,0)*RotMat(1,0)));
    //  yawangle = atan2(RotMat(1,0)/cos(pitchangle),RotMat(0,0)/cos(pitchangle));
    //  rollangle = atan2(RotMat(2,1)/cos(pitchangle), RotMat(2,2)/cos(pitchangle));

    double threshold = 0.001;
    pitchangle = -asin(RotMat(2, 0));
    if (RotMat(2, 0) > 1.0 - threshold && RotMat(2, 0) < 1.0 + threshold) //when RotMat(2,0) == 1
    {//Gimbal lock, pitch = -90deg
        rollangle = atan2(-RotMat(0, 1), -RotMat(0, 2));
        yawangle = 0.0;
    }
    else if (RotMat(2, 0) < -1.0 + threshold && RotMat(2, 0) > -1.0 - threshold) //when RotMat(2,0) == -1
    {//Gimbal lock, pitch = 90deg
        rollangle = atan2(RotMat(0, 1), RotMat(0, 2));
        yawangle = 0.0;
    }
    else //general solution
    {
        rollangle = atan2(RotMat(2, 1), RotMat(2, 2));
        yawangle = atan2(RotMat(1, 0), RotMat(0, 0));
    }

    BodyAngle(0) = rollangle;
    BodyAngle(1) = pitchangle;
    BodyAngle(2) = yawangle;

    return BodyAngle;
}

static double GetBodyPitchAngle(Eigen::Matrix3d RotMat)
{
    double pitchangle;
    pitchangle = -asin(RotMat(2, 0));

    return pitchangle;
}

static double GetBodyRollAngle(Eigen::Matrix3d RotMat)
{
    double rollangle, pitchangle;
    double threshold = 0.001;
    pitchangle = -asin(RotMat(2, 0));
    if (RotMat(2, 0) > 1.0 - threshold && RotMat(2, 0) < 1.0 + threshold) //when RotMat(2,0) == 1
    {//Gimbal lock, pitch = -90deg
        rollangle = atan2(-RotMat(0, 1), -RotMat(0, 2));
    }
    else if (RotMat(2, 0) < -1.0 + threshold && RotMat(2, 0) > -1.0 - threshold) //when RotMat(2,0) == -1
    {//Gimbal lock, pitch = 90deg
        rollangle = atan2(RotMat(0, 1), RotMat(0, 2));
    }
    else //general solution
    {
        rollangle = atan2(RotMat(2, 1), RotMat(2, 2));
    }

    return rollangle;
}

static double GetBodyYawAngle(Eigen::Matrix3d RotMat)
{
    double pitchangle, yawangle;

    double threshold = 0.001;
    pitchangle = -asin(RotMat(2, 0));
    if (RotMat(2, 0) > 1.0 - threshold && RotMat(2, 0) < 1.0 + threshold) //when RotMat(2,0) == 1
    {//Gimbal lock, pitch = -90deg
        yawangle = 0.0;
    }
    else if (RotMat(2, 0) < -1.0 + threshold && RotMat(2, 0) > -1.0 - threshold) //when RotMat(2,0) == -1
    {//Gimbal lock, pitch = 90deg
        yawangle = 0.0;
    }
    else //general solution
    {
        yawangle = atan2(RotMat(1, 0), RotMat(0, 0));
    }

    return yawangle;
}

static Eigen::Matrix3d GetBodyRotationMatrix(double Roll, double Pitch, double Yaw)
{
    Eigen::Matrix3d R_yaw;
    R_yaw.setZero();
    //yawŽÂ zÃà¿¡ ŽëÇÑ ÈžÀü
    R_yaw(2, 2) = 1.0;
    R_yaw(0, 0) = cos(Yaw);
    R_yaw(0, 1) = -sin(Yaw);
    R_yaw(1, 0) = sin(Yaw);
    R_yaw(1, 1) = cos(Yaw);

    Eigen::Matrix3d R_pitch;
    R_pitch.setZero();
    //pitchŽÂ yÃà¿¡ ŽëÇÑ ÈžÀü
    R_pitch(1, 1) = 1.0;
    R_pitch(0, 0) = cos(Pitch);
    R_pitch(2, 2) = cos(Pitch);
    R_pitch(0, 2) = sin(Pitch);
    R_pitch(2, 0) = -sin(Pitch);

    Eigen::Matrix3d R_roll;
    R_roll.setZero();
    //rollÀº xÃà¿¡ ŽëÇÑ ÈžÀü
    R_roll(0, 0) = 1.0;
    R_roll(1, 1) = cos(Roll);
    R_roll(2, 2) = cos(Roll);
    R_roll(1, 2) = -sin(Roll);
    R_roll(2, 1) = sin(Roll);

    Eigen::Matrix3d RGyro;
    RGyro.noalias() = R_yaw * R_pitch * R_roll;
    //dMatrix RGyro_inv(3,3);
    //RGyro.inv(RGyro_inv);

    //return RGyro_inv;
    return RGyro;
}

static Eigen::Matrix3d rotateWithZ(double yaw_angle)
{
    Eigen::Matrix3d rotate_wth_z(3, 3);

    rotate_wth_z(0, 0) = cos(yaw_angle);
    rotate_wth_z(1, 0) = sin(yaw_angle);
    rotate_wth_z(2, 0) = 0.0;

    rotate_wth_z(0, 1) = -sin(yaw_angle);
    rotate_wth_z(1, 1) = cos(yaw_angle);
    rotate_wth_z(2, 1) = 0.0;

    rotate_wth_z(0, 2) = 0.0;
    rotate_wth_z(1, 2) = 0.0;
    rotate_wth_z(2, 2) = 1.0;

    return rotate_wth_z;
}

static Eigen::Matrix3d rotateWithY(double pitch_angle)
{
    Eigen::Matrix3d rotate_wth_y(3, 3);

    rotate_wth_y(0, 0) = cos(pitch_angle);
    rotate_wth_y(1, 0) = 0.0;
    rotate_wth_y(2, 0) = -sin(pitch_angle);

    rotate_wth_y(0, 1) = 0.0;
    rotate_wth_y(1, 1) = 1.0;
    rotate_wth_y(2, 1) = 0.0;

    rotate_wth_y(0, 2) = sin(pitch_angle);
    rotate_wth_y(1, 2) = 0.0;
    rotate_wth_y(2, 2) = cos(pitch_angle);

    return rotate_wth_y;
}

static Eigen::Matrix3d rotateWithX(double roll_angle)
{
    Eigen::Matrix3d rotate_wth_x(3, 3);

    rotate_wth_x(0, 0) = 1.0;
    rotate_wth_x(1, 0) = 0.0;
    rotate_wth_x(2, 0) = 0.0;

    rotate_wth_x(0, 1) = 0.0;
    rotate_wth_x(1, 1) = cos(roll_angle);
    rotate_wth_x(2, 1) = sin(roll_angle);

    rotate_wth_x(0, 2) = 0.0;
    rotate_wth_x(1, 2) = -sin(roll_angle);
    rotate_wth_x(2, 2) = cos(roll_angle);

    return rotate_wth_x;
}

static Eigen::Matrix3d skew(Eigen::Vector3d src)
{
    Eigen::Matrix3d skew;
    skew.setZero();
    skew(0, 1) = -src[2];
    skew(0, 2) = src[1];
    skew(1, 0) = src[2];
    skew(1, 2) = -src[0];
    skew(2, 0) = -src[1];
    skew(2, 1) = src[0];

    return skew;
}

static Eigen::Vector3d OrientationVelocity(Eigen::Matrix3d Rot, Eigen::Matrix3d Rotdot) //rotation matrix, derivative of rotation matrix
{
    Eigen::Matrix3d RdotRT;
    RdotRT = Rotdot * Rot.transpose();
    Eigen::Vector3d OriVel;
    OriVel(0) = RdotRT(2, 1);
    OriVel(1) = RdotRT(0, 2);
    OriVel(2) = RdotRT(1, 0);

    return OriVel;
}

static double Cubic(double rT, double rT_0, double rT_f, double rx_0, double rx_dot_0, double rx_f, double rx_dot_f)
{
    double rx_q;

    if (rT < rT_0)
    {
        rx_q = rx_0;
    }
    else if (rT > rT_f)
    {
        rx_q = rx_f;
    }
    else {

        rx_q = rx_0 + rx_dot_0 * (rT - rT_0)
            + (3.0 * (rx_f - rx_0) / ((rT_f - rT_0) * (rT_f - rT_0)) - 2.0 * rx_dot_0 / (rT_f - rT_0) - rx_dot_f / (rT_f - rT_0)) * (rT - rT_0) * (rT - rT_0)
            + (-2.0 * (rx_f - rx_0) / ((rT_f - rT_0) * (rT_f - rT_0) * (rT_f - rT_0)) + (rx_dot_0 + rx_dot_f) / ((rT_f - rT_0) * (rT_f - rT_0))) * (rT - rT_0) * (rT - rT_0) * (rT - rT_0);
    }
    return (rx_q);
}

static double CubicDot(double rT, double rT_0, double rT_f, double rx_0, double rx_dot_0, double rx_f, double rx_dot_f)
{
    double rx_q_dot;

    if (rT < rT_0)
    {
        rx_q_dot = rx_dot_0;
    }
    else if (rT > rT_f)
    {
        rx_q_dot = rx_dot_f;
    }
    else {
        rx_q_dot = rx_dot_0 + 2.0 * (3.0 * (rx_f - rx_0) / ((rT_f - rT_0) * (rT_f - rT_0)) - 2.0 * rx_dot_0 / (rT_f - rT_0) - rx_dot_f / (rT_f - rT_0)) * (rT - rT_0)
            + 3.0 * (-2.0 * (rx_f - rx_0) / ((rT_f - rT_0) * (rT_f - rT_0) * (rT_f - rT_0)) + (rx_dot_0 + rx_dot_f) / ((rT_f - rT_0) * (rT_f - rT_0))) * (rT - rT_0) * (rT - rT_0);
    }
    return (rx_q_dot);
}

}

#endif

