//
// Created by robot on 10/12/17.
//

//~ //System
#include <iostream>
#include <stdio.h>
#include <cmath>
#include <unistd.h>
//Externals
#include <Eigen/Dense>
#include "gurobi_c++.h"
#include "json/json.h"
//Custom
#include "MPC_thread.h"
//#include "OptProgram.h"
//#include "StructuresOptProgram.h"
//#include "StructuresMain.h"
//#include "MPC.h"
#include "Pusher.h"
#include "PointPusher.h"
#include "FOM.h"
//ROS
#include <ros/ros.h>

using namespace std;
using Eigen::MatrixXd;



//********************************************************************
// Optimization Thread
//********************************************************************
void *MPC_thread(void *thread_arg)
{

    struct MPC_thread_data *my_data;
    my_data = (struct MPC_thread_data *) thread_arg;
    outSolutionStruct out_solution;
    VectorXd mode_schedule(my_data->controller->steps);

    //clear data
    my_data->controller->matricesMPC.Aeq_vec.clear();
    my_data->controller->matricesMPC.Ain_vec.clear();
    my_data->controller->matricesMPC.beq = VectorXd::Zero(my_data->controller->matricesMPC.beq.size());
    my_data->controller->matricesMPC.bin = VectorXd::Zero(my_data->controller->matricesMPC.bin.size());
    my_data->controller->matricesMPC.row_start_eq=0;
    my_data->controller->matricesMPC.row_start_ineq=0;

    //build data
    if (my_data->is_gp==true){
        my_data->controller->buildConstraintMatricesHybrid(my_data->time, my_data->mode_schedule, my_data->delta_xc);
    }
    else{
        my_data->controller->buildConstraintMatrices(my_data->time, my_data->mode_schedule, my_data->delta_xc);
    }

    //convert triplets to sparse matrices
    Eigen::SparseMatrix<double,Eigen::RowMajor> Aeq(my_data->controller->matricesMPC.row_start_eq,my_data->controller->num_variables);
    Aeq.setFromTriplets(my_data->controller->matricesMPC.Aeq_vec.begin(), my_data->controller->matricesMPC.Aeq_vec.end());////    my_data->controller->opt_program->addEqConstraintsGur(list_controller[0]->matricesMPC.Aeq.middleRows(0, list_controller[0]->matricesMPC.row_start_eq), list_controller[0]->matricesMPC.beq.segment(0, list_controller[0]->matricesMPC.row_start_eq));
    Eigen::SparseMatrix<double,Eigen::RowMajor> Ain(my_data->controller->matricesMPC.row_start_ineq,my_data->controller->num_variables);
    Ain.setFromTriplets(my_data->controller->matricesMPC.Ain_vec.begin(), my_data->controller->matricesMPC.Ain_vec.end());////    my_data->controller->opt_program->addEqConstraintsGur(list_controller[0]->matricesMPC.Aeq.middleRows(0, list_controller[0]->matricesMPC.row_start_eq), list_controller[0]->matricesMPC.beq.segment(0, list_controller[0]->matricesMPC.row_start_eq));
    Eigen::SparseMatrix<double,Eigen::RowMajor> H(my_data->controller->num_variables,my_data->controller->num_variables);
    H.setFromTriplets(my_data->controller->matricesMPC.Q_vec.begin(), my_data->controller->matricesMPC.Q_vec.end());////    my_data->controller->opt_program->addEqConstraintsGur(list_controller[0]->matricesMPC.Aeq.middleRows(0, list_controller[0]->matricesMPC.row_start_eq), list_controller[0]->matricesMPC.beq.segment(0, list_controller[0]->matricesMPC.row_start_eq));

    //solve quadratic program
    VectorXd c = VectorXd::Zero(my_data->controller->num_variables);
    VectorXd z_sol(my_data->controller->num_variables);
//
//    cout<< "Q"<<MatrixXd(H)<<endl;
////    if (my_data->mode_schedule(0)==2){
//    cout<< "Aeq"<<endl<<MatrixXd(Aeq)<<endl;
//    cout<< "Ain"<<endl<<MatrixXd(Ain)<<endl;
//    cout<< "bin"<<endl<<my_data->controller->matricesMPC.bin.segment(0, my_data->controller->matricesMPC.row_start_ineq)<<endl;
//    cout<< "beq"<< endl<<my_data->controller->matricesMPC.beq.segment(0, my_data->controller->matricesMPC.row_start_eq)<<endl;

//    sleep(10);
//    cout<< "Q"<<MatrixXd(H)<<endl;
//    cout<< "Aeq"<<MatrixXd(Aeq)<<endl;
//    cout<< "Ain"<<MatrixXd(Ain)<<endl;
//    cout<< "beq"<<my_data->controller->matricesMPC.beq<<endl;
//    cout<< "bin"<<my_data->controller->matricesMPC.bin<<endl;
//
//    sleep(10);

//    double t0 = Helper::gettime();
//  double t0 = Helper::gettime();
    Helper::solve_quadprog(H, c,
                           Ain, my_data->controller->matricesMPC.bin.segment(0, my_data->controller->matricesMPC.row_start_ineq),
                           Aeq, my_data->controller->matricesMPC.beq.segment(0, my_data->controller->matricesMPC.row_start_eq),
                           my_data->controller->out_solution.objVal,
                           z_sol);

//  double tf = Helper::gettime();
//  cout<<"building time: "<<tf-t0<<endl;
    my_data->controller->out_solution.solution = z_sol.head(my_data->controller->line_pusher->numucStates);
    pthread_exit(NULL);

}
