//
// Created by mcube10 on 5/4/18.
//
#include "LoopControl.h"
#include "OptProgram.h"
#include "StructuresOptProgram.h"
#include "StructuresMain.h"
#include "Pusher.h"
#include "MPC_thread.h"
#include "MPC.h"

#ifndef PUSH_CONTROL_GPCONTROLLER_H
#define PUSH_CONTROL_GPCONTROLLER_H

class GPDataController {
public:
    //Properties
    MPC *controller;
    outSolutionStruct *out_solution;
    outMatrices out_matrices;
    Pusher *line_pusher;
    Friction *friction;
    PusherSlider *pusher_slider;
    int numucStates;

    //Methods
    GPDataController(PusherSlider *pusher_slider, Pusher *_line_pusher, Friction *_friction, MatrixXd Q, MatrixXd Qf, MatrixXd R);
    VectorXd solveMPC(VectorXd xc, double _time);
    VectorXd get_robot_velocity(VectorXd xc, VectorXd uc);
};

#endif //PUSH_CONTROL_GPCONTROLLER_H
