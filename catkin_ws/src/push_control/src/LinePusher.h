/*
 */
#include "StructuresLinePusher.h"
#include "PusherSlider.h"
#include "Friction.h"
#include "Pusher.h"
#include <Eigen/Dense>

using namespace std;
using namespace Eigen;


#ifndef LINEPUSHER_H
#define LINEPUSHER_H

class LinePusher: public Pusher {
    public:


        //Static Methods
        LinePusher(PusherSlider* _pusher_slider, Friction* _friction, string trajectory_name);
        VectorXd buildUcEq();
        VectorXd buildXcEq();
        VectorXd coordinateTransformSC(VectorXd xs);
        VectorXd coordinateTransformCS(VectorXd xc);
        VectorXd force2Velocity(VectorXd xc, VectorXd uc);
//        Matrix3d buildSib(double theta);
        outStarStruct buildNominalTrajectory(double h_star);
        outStateNominal getStateNominal(double t);
        outStateNominal getStateNominal2(double t);
        outStarStruct buildStraightLineTrajectory(string v_eq);
        outStarStruct build8TrackTrajectory(string v_eq);
        outStarStruct buildTrajectory(string v_eq);
        VectorXd getError(VectorXd xc, double t);
        MatrixXd B_fun(VectorXd xc_star, VectorXd uc_star, double rx, double d, MatrixXd A_ls);
        MatrixXd A_fun(VectorXd xc_star, VectorXd uc_star, double rx, double d, MatrixXd A_ls);
};

#endif


