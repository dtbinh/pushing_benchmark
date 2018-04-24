//
// Created by robot on 10/12/17.
//


//~ //System
//Externals
#include <Eigen/Dense>
#include <pthread.h>
#include "Helper.h"
#include "MPC.h"


#ifndef PUSH_CONTROL_MPC_THREAD_H
#define PUSH_CONTROL_MPC_THREAD_H

extern pthread_mutex_t nonBlockMutex;

//***************
struct MPC_thread_data{
    MPC *controller;
    outSolutionStruct *out_solution;
    VectorXd delta_xc;
    VectorXd mode_schedule;
    double time;

};

using namespace std;
using Eigen::MatrixXd;

void *MPC_thread(void *thread_arg);



#endif //PUSH_CONTROL_MPC_THREAD_H
