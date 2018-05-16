//
// Created by mcube10 on 5/4/18.
//

//
// Created by mcube10 on 5/8/17.
//f

#include "StructuresOptProgram.h"
#include "StructuresMain.h"
#include "MPC.h"
#include "GPDataController.h"
#include "LinePusher.h"
#include <unistd.h>
#include <ros/ros.h>


typedef SparseMatrix<double> SparseMatrixXd;

GPDataController::GPDataController(PusherSlider *_pusher_slider, Pusher *_line_pusher, Friction *_friction, MatrixXd Q, MatrixXd Qf, MatrixXd R, double _h, int _steps) {
  line_pusher = _line_pusher;
  _line_pusher->numucStates=2;
  friction=_friction;
  pusher_slider=_pusher_slider;
  numucStates=_line_pusher->numucStates;

  //initialize controller and output
  controller = new MPC(_pusher_slider, _line_pusher, _friction,  Q,  Qf,  R, _h, _steps);
  out_solution = new outSolutionStruct;

  //initialize cost function weights and reserve memory for large MPC matrices
//  controller->initializeMatricesMPC();
//  controller->buildWeightMatrices();
}

VectorXd GPDataController::solveMPC(VectorXd xc, double time){

  //Initialize variables
  VectorXd delta_xc(xc.rows());
  outStateNominal out_state_nominal;

  //get nominal state
  out_state_nominal = line_pusher->getStateNominal(time);
  delta_xc = line_pusher->getError(xc, time);

  //clear constraint matrices clean
  controller->matricesMPC.Aeq_vec.clear();
  controller->matricesMPC.Ain_vec.clear();
  controller->matricesMPC.beq = VectorXd::Zero(controller->matricesMPC.beq.size());
  controller->matricesMPC.bin = VectorXd::Zero(controller->matricesMPC.bin.size());
  controller->matricesMPC.row_start_eq=0;
  controller->matricesMPC.row_start_ineq=0;

  //build data
  controller->buildConstraintMatricesGPData(time, delta_xc);
  

  //convert triplets to sparse matrices
  Eigen::SparseMatrix<double,Eigen::RowMajor> Aeq(controller->matricesMPC.row_start_eq,controller->num_variables);
  Aeq.setFromTriplets(controller->matricesMPC.Aeq_vec.begin(), controller->matricesMPC.Aeq_vec.end());////    controller->opt_program->addEqConstraintsGur(list_controller[0]->matricesMPC.Aeq.middleRows(0, list_controller[0]->matricesMPC.row_start_eq), list_controller[0]->matricesMPC.beq.segment(0, list_controller[0]->matricesMPC.row_start_eq));
  Eigen::SparseMatrix<double,Eigen::RowMajor> Ain(controller->matricesMPC.row_start_ineq,controller->num_variables);
  Ain.setFromTriplets(controller->matricesMPC.Ain_vec.begin(), controller->matricesMPC.Ain_vec.end());////    controller->opt_program->addEqConstraintsGur(list_controller[0]->matricesMPC.Aeq.middleRows(0, list_controller[0]->matricesMPC.row_start_eq), list_controller[0]->matricesMPC.beq.segment(0, list_controller[0]->matricesMPC.row_start_eq));
  Eigen::SparseMatrix<double,Eigen::RowMajor> H(controller->num_variables,controller->num_variables);
  H.setFromTriplets(controller->matricesMPC.Q_vec.begin(), controller->matricesMPC.Q_vec.end());////    controller->opt_program->addEqConstraintsGur(list_controller[0]->matricesMPC.Aeq.middleRows(0, list_controller[0]->matricesMPC.row_start_eq), list_controller[0]->matricesMPC.beq.segment(0, list_controller[0]->matricesMPC.row_start_eq));
  //solve quadratic program
  VectorXd c = VectorXd::Zero(controller->num_variables);
  VectorXd z_sol(controller->num_variables);
  
//
//    cout<< "Q"<<MatrixXd(H)<<endl;
//    if (mode_schedule(0)==2){
      // cout<<"time"<<endl;
  // cout<<time<<endl;
  
  // cout<<"delta_xc"<<endl;
  // cout<<delta_xc<<endl;
  
  // cout<<"delta_xc"<<endl;
  // cout<<delta_xc<<endl;
   // cout<< "Aeq"<<MatrixXd(Aeq)<<endl;
   // cout<< "Ain"<<MatrixXd(Ain)<<endl;
   // cout<< "bin"<<controller->matricesMPC.bin.segment(0, controller->matricesMPC.row_start_ineq)<<endl;
   // cout<< "beq"<< controller->matricesMPC.beq.segment(0, controller->matricesMPC.row_start_eq)<<endl;
   // sleep(10.);
//


//    sleep(10);
//    cout<< "Q"<<MatrixXd(H)<<endl;
//    cout<< "Aeq"<<MatrixXd(Aeq)<<endl;
//    cout<< "Ain"<<MatrixXd(Ain)<<endl;
//    cout<< "beq"<<controller->matricesMPC.beq<<endl;
//    cout<< "bin"<<controller->matricesMPC.bin<<endl;
//
//    sleep(10);

//    double t0 = Helper::gettime();
//  double t0 = Helper::gettime();

  //solve optimization program
  Helper::solve_quadprog(H, c,
                         Ain, controller->matricesMPC.bin.segment(0, controller->matricesMPC.row_start_ineq),
                         Aeq, controller->matricesMPC.beq.segment(0, controller->matricesMPC.row_start_eq),
                         controller->out_solution.objVal,
                         z_sol);

  //retrieve solution
  VectorXd delta_uc =  z_sol.head(controller->line_pusher->numucStates);
//
//  cout<< "u_solution"<<endl;
//  cout<< z_sol.head(controller->line_pusher->numucStates*35)<<endl;
//  cout<< "z_sol" <<endl;
//  cout<< z_sol <<endl;

  //Add feedforward with feedback control
  VectorXd uc = delta_uc + out_state_nominal.ucStar;
//  cout<<"uc"<<endl;
//  cout<<uc<<endl;
//  cout<<"delta_uc"<<endl;
//  cout<<delta_uc<<endl;
  return uc;
}

VectorXd GPDataController::get_robot_velocity(VectorXd xc, VectorXd uc) {
  Matrix2d Cbi;
  VectorXd vel(3);
  Vector2d vel_2d;
  Cbi = Helper::C3_2d(xc(2));
  vel_2d = Cbi.transpose()*uc;
  vel<<vel_2d,0;
  return vel;
}
