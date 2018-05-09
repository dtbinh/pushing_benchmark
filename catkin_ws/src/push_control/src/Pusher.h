/*
 */
#include "StructuresLinePusher.h"
#include "Friction.h"
#include <Eigen/Dense>

using namespace std;
using namespace Eigen;

#ifndef PUSH_CONTROL_PUSHER_H
#define PUSH_CONTROL_PUSHER_H

class Pusher {
public:
    //Static Properties
    double lp;
    double d;
    int numxcStates;
    int numucStates;
    int numxsStates;
    int numusStates;
    int num_contact_points;
    string pusher_type;
    Eigen::VectorXd uc_eq;
    Eigen::VectorXd xc_eq;
    Eigen::MatrixXd uc_star;
    Eigen::MatrixXd xc_star;
    Eigen::MatrixXd us_star;
    Eigen::MatrixXd xs_star;
    Eigen::MatrixXd A_star;
    Eigen::MatrixXd B_star;
    Eigen::VectorXd t_star;
    Eigen::MatrixXd uc_star2;
    Eigen::MatrixXd xc_star2;
    Eigen::MatrixXd us_star2;
    Eigen::MatrixXd xs_star2;
    Eigen::VectorXd t_star2;
    PusherSlider* ppusher_slider;
    Friction* pfriction;

    //Static Methods
//    virtual VectorXd buildUcEq() = 0;
//    virtual VectorXd buildXcEq() = 0;
    virtual VectorXd coordinateTransformSC(VectorXd xs) = 0;
    virtual VectorXd coordinateTransformCS(VectorXd xc) = 0;
    virtual VectorXd force2Velocity(VectorXd xc, VectorXd uc) = 0;
//    virtual outStarStruct buildNominalTrajectory(double h_star) = 0;
    virtual outStateNominal getStateNominal(double t) = 0;
    virtual outStateNominal getStateNominalGPData(double t) = 0;
//    virtual outStateNominal getStateNominal2(double t) = 0;
//    virtual outStarStruct buildStraightLineTrajectory(string v_eq) = 0;
//    virtual outStarStruct build8TrackTrajectory(string v_eq) = 0;
    virtual VectorXd getError(VectorXd xc, double t) = 0;
    virtual MatrixXd B_fun(VectorXd xc_star, VectorXd uc_star, double rx, double d, MatrixXd A_ls) = 0;
    virtual MatrixXd A_fun(VectorXd xc_star, VectorXd uc_star, double rx, double d, MatrixXd A_ls) = 0;
};


#endif //PUSH_CONTROL_PUSHER_H
