//
// Created by mcube10 on 5/8/17.
//

#include "StructuresOptProgram.h"
#include "StructuresMain.h"
#include "MPC.h"
#include "LModes.h"
#include "LinePusher.h"
#include <unistd.h>
#include "push_control/MODE_SRV.h"
#include <ros/ros.h>
#include "std_msgs/Float64MultiArray.h"


typedef SparseMatrix<double> SparseMatrixXd;

LMODES::LMODES(PusherSlider *_pusher_slider, Pusher *_line_pusher, Friction *_friction) {
    line_pusher = _line_pusher;
    friction=_friction;
    pusher_slider=_pusher_slider;
    numucStates=_line_pusher->numucStates;

    int lv1=0;
    num_families=1;
    out_matrices = readMatrices(lv1);

    controller = new MPC(out_matrices, _pusher_slider, _line_pusher, _friction);
    list_controller[lv1] = controller;

    out_solution = new outSolutionStruct;
    list_out_solution[lv1] = out_solution;

    thread_data_array = new MPC_thread_data;
    thread_data_array->controller = list_controller[lv1];
    thread_data_array->out_solution = list_out_solution[lv1];

    list_controller[lv1]->initializeMatricesMPC();
    list_controller[lv1]->buildWeightMatrices();

    thread_data_list[lv1] = thread_data_array;

}

VectorXd LMODES::learnModeSchedule(VectorXd delta_xc, double _y_star, double _theta_star){
//
  //convert error to body frame
  Matrix2d Cbi;
  Vector2d rbbi;
  VectorXd delta_xc_b;
  Cbi = Helper::C3_2d(_theta_star);
  rbbi = Cbi*delta_xc.head(2);
  delta_xc_b = delta_xc;
  delta_xc_b(0) = rbbi(0);
  delta_xc_b(1) = rbbi(1);

  ros::NodeHandle n;
  ros::ServiceClient mode_learner = n.serviceClient<push_control::MODE_SRV>("mode_learner");
  push_control::MODE_SRV srv;
  std_msgs::Float64MultiArray msg;

  //call rosservice and compute mode schedule using NNetwork
  if (_y_star>0) {
    vector<double> vec1 = {delta_xc_b(0), -delta_xc_b(1), -delta_xc_b(2), -delta_xc_b(3)};
    msg.data = vec1;
  }
  else{
    vector<double> vec1 = {delta_xc_b(0), delta_xc_b(1), delta_xc_b(2), delta_xc_b(3)};
    msg.data = vec1;
  }

  srv.request.delta_x = msg.data;
  mode_learner.call(srv);


  VectorXd _mode_schedule(list_controller[0]->steps);
  for (int i=0;i<list_controller[0]->steps;i++){
    if (_y_star>0){
      if (srv.response.mode_schedule[i]==1){
        _mode_schedule(i) = 2;
      }
      else if (srv.response.mode_schedule[i]==2){
        _mode_schedule(i) = 1;
      }
      else if (srv.response.mode_schedule[i]==0){
        _mode_schedule(i) = 0;
      }
    }
    else{
      _mode_schedule(i) = srv.response.mode_schedule[i];
    }
  }
  cout<<"5"<<endl;

  return _mode_schedule;
}
VectorXd LMODES::solveLMODES(VectorXd xc, double time){

    //Initialize variables
    VectorXd objList(num_families);
    VectorXd delta_xc(xc.rows());
//    VectorXd mode_schedule(list_controller[0]->steps);
    VectorXd mode_schedule2_learned(list_controller[0]->steps);
    outStateNominal out_state_nominal;

    //get nominal state
    out_state_nominal = line_pusher->getStateNominal(time);
    delta_xc = line_pusher->getError(xc, time);
    double y_star = out_state_nominal.xcStar(1);
    double theta_star = out_state_nominal.xcStar(2);

    //learn modes
    mode_schedule2_learned =learnModeSchedule(delta_xc, y_star, theta_star);


    for (int i=0;i<num_families;i++){
        thread_data_list[i]->delta_xc = delta_xc;
        thread_data_list[i]->time = time;
        thread_data_list[i]->mode_schedule = mode_schedule2_learned;
    }

    //Find state error at current time
    delta_xc = line_pusher->getError(xc, time);

//    cout<<"xc_star"<<out_state_nominal.xcStar<<endl;
//    cout<<"uc_star"<<out_state_nominal.ucStar<<endl;
    //create threads (and run controllers in parallel)
    for (int i=0;i<num_families;i++){
        pthread_create(&my_thread[i], NULL, &MPC_thread, (void *)  thread_data_list[i]);
    }

    //wait for all programs to terminate
    for (int i=0;i<num_families;i++){
        pthread_join(my_thread[i], NULL);
        objList(i) = list_controller[i]->out_solution.objVal;
//      cout<<"delta_u "<<i<<endl<<list_controller[i]->out_solution.solution<<endl;
    }

    //Find minimum objective value
    MatrixXf::Index   minIndex;
    double minVal = objList.minCoeff(&minIndex);

    VectorXd delta_uc = list_controller[minIndex]->out_solution.solution;
    VectorXd uc = delta_uc + out_state_nominal.ucStar;

    return uc;
}

// Read matrices from JSON file
outMatrices LMODES::readMatrices(int flag){
  //~ Read Matrices
  Json::Value root;
  Json::Reader reader;
  char const* tmp = getenv( "FPUSH_BASE" );
  string envStr( tmp );
  string fileName;
  if (flag==0){fileName = envStr + "/Simulation/Data/MatricesMode0_" + line_pusher->pusher_type + ".json"; }
  else if(flag==1){fileName = envStr + "/Simulation/Data/MatricesMode1_" + line_pusher->pusher_type + ".json"; }
  else{fileName = envStr + "/Simulation/Data/MatricesMode2_" + line_pusher->pusher_type + ".json";}

  ifstream file(fileName);
  file >> root;
  int num_eq_constraints = root["Matrices"]["beq"].size();
  int num_ineq_constraints = root["Matrices"]["bin"].size();
  int num_variables = root["Matrices"]["Q"].size();
  Eigen::MatrixXd Q(num_variables, num_variables);
  Eigen::MatrixXd Aeq(num_eq_constraints,num_variables);
  Eigen::MatrixXd Ain(num_ineq_constraints,num_variables);
  Eigen::MatrixXd A_bar(4,4);
  Eigen::VectorXd fobj(num_variables);
  Eigen::VectorXd beq(num_eq_constraints);
  Eigen::VectorXd bin(num_ineq_constraints);
  Eigen::VectorXd lb(num_variables);
  Eigen::VectorXd ub(num_variables);

  Helper::write_matrix_JSON(root["Matrices"]["Q"], Q);
  Helper::write_matrix_JSON(root["Matrices"]["Aeq"], Aeq);
  Helper::write_matrix_JSON(root["Matrices"]["Ain"], Ain);
  Helper::write_matrix_JSON(root["Matrices"]["A_bar"], A_bar);
  Helper::write_vector_JSON(root["Matrices"]["f"], fobj);
  Helper::write_vector_JSON(root["Matrices"]["bin"], bin);
  Helper::write_vector_JSON(root["Matrices"]["beq"], beq);
  Helper::write_vector_JSON(root["Matrices"]["lb"], lb);
  Helper::write_vector_JSON(root["Matrices"]["ub"], ub);

  outMatrices out_matrices;
  out_matrices.Q=Q;
  out_matrices.fobj=fobj;
  out_matrices.A_bar=A_bar;
  out_matrices.Ain=Ain;
  out_matrices.bin=bin;
  out_matrices.Aeq=Aeq;
  out_matrices.beq=beq;
  out_matrices.lb=lb;
  out_matrices.ub=ub;

  return out_matrices;
}
