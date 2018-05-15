//
// Created by mcube10 on 5/8/18.
//

#include "LoopControl.h"
#include "OptProgram.h"
#include "StructuresOptProgram.h"
#include "StructuresMain.h"
#include "Pusher.h"
#include "MPC_thread.h"
#include "MPC.h"
#include "Controller.h"

#ifndef PUSH_CONTROL_HYBRIDCONTROLLER_H
#define PUSH_CONTROL_HYBRIDCONTROLLER_H

class HybridController: public Controller {
public:
    //Properties
    MPC *controller;
    MPC_thread_data *thread_data_array;
    MPC_thread_data thread_data_array_tmp;
    outSolutionStruct *out_solution;
    MPC *list_controller[3];
    MPC_thread_data *thread_data_list[3];
    outSolutionStruct *list_out_solution[3];
    outMatrices out_matrices;
    Pusher *line_pusher;
    Friction *friction;
    PusherSlider *pusher_slider;
    int num_families;
    int numucStates;
    pthread_t my_thread[3];
    //Methods
    HybridController(int _num_families, PusherSlider *pusher_slider, Pusher *_line_pusher, Friction *_friction, MatrixXd Q, MatrixXd Qf, MatrixXd R, double _h, int _steps);
    VectorXd solveMPC(VectorXd xc, double _time);
    VectorXd get_robot_velocity(VectorXd xc, VectorXd uc);
};

#endif //PUSH_CONTROL_HYBRIDCONTROLLER_H
