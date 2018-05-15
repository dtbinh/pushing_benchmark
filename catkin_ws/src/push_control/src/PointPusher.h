//
// Created by mcube10 on 10/5/17.
//
#include "StructuresLinePusher.h"
#include "PusherSlider.h"
//#include "Friction.h"
#include "Pusher.h"
#include <Eigen/Dense>

using namespace std;
using namespace Eigen;

#ifndef PUSH_CONTROL_POINTPUSHER_H
#define PUSH_CONTROL_POINTPUSHER_H

class PointPusher: public Pusher {
public:
    //Static Methods
    PointPusher(PusherSlider* _pusher_slider, Friction* _friction, string trajectory_name, int _numuc_states);
//    VectorXd buildUcEq();
//    VectorXd buildXcEq();
    VectorXd coordinateTransformSC(VectorXd xs);
    VectorXd offsetPusher(VectorXd xs);
    VectorXd coordinateTransformCS(VectorXd xc);
    VectorXd force2Velocity(VectorXd xc, VectorXd uc);
//    outStarStruct buildNominalTrajectory(double h_star);
    outStateNominal getStateNominal(double t);
    outStateNominal getStateNominalGPData(double t);
//    outStateNominal getStateNominal2(double t);
//    outStarStruct buildStraightLineTrajectory(string v_eq);
//    outStarStruct build8TrackTrajectory(string v_eq);
    outStarStruct buildTrajectory(string v_eq);
    VectorXd getError(VectorXd xc, double t);
    MatrixXd B_fun(VectorXd xc_star, VectorXd uc_star, double rx, double d, MatrixXd A_ls);
    MatrixXd A_fun(VectorXd xc_star, VectorXd uc_star, double rx, double d, MatrixXd A_ls);
};

#endif //PUSH_CONTROL_POINTPUSHER_H
