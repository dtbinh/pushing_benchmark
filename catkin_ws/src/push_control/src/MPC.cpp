//Common libraries
#include <iostream>
#include <stdio.h>
#include <math.h>
//External libraries
#include <Eigen/Dense>
//Custom Libraries
#include "PusherSlider.h"
#include "Pusher.h"
#include "Helper.h"
#include "LinePusher.h"
#include "MPC.h"
#include "OptProgram.h"
#include "StructuresOptProgram.h"
#include <unistd.h>


using namespace std;
using namespace Eigen;
using namespace Eigen;

MPC::MPC(PusherSlider *_pusher_slider, Pusher *_line_pusher, Friction *_friction, MatrixXd Q, MatrixXd Qf, MatrixXd R)
{
    pusher_slider = _pusher_slider;
    friction = _friction;
    line_pusher = _line_pusher;
    h = 0.02;
    steps = 35;
    num_variables = (line_pusher->numucStates + line_pusher->numxcStates)*steps;
    initializeMatricesMPC();
    buildWeightMatrices(Q, Qf, R);
}

int MPC::getControlIndex(int lv1){
  int start_index = line_pusher->numucStates*lv1;
  return start_index;
}

int MPC::getStateIndex(int lv1){
  int start_index = (line_pusher->numucStates*(steps)) + line_pusher->numxcStates*(lv1);
  return start_index;
}

void MPC::initializeMatricesMPC(){
//
  matricesMPC.Q_vec.reserve(315);//9*steps 315
  matricesMPC.Aeq_vec.reserve(900);//25*steps 900
  matricesMPC.Ain_vec.reserve(500);//14*steps 500
  matricesMPC.beq =VectorXd::Zero(200);//5*steps 200
  matricesMPC.row_start_eq = 0;
  matricesMPC.bin =VectorXd::Zero(200);//5*steps 200
  matricesMPC.row_start_ineq =0;
}
//
void MPC::buildWeightMatrices(MatrixXd Q, MatrixXd Qf, MatrixXd R){
//
//  MatrixXd Q = MatrixXd::Zero(line_pusher->numxcStates, line_pusher->numxcStates);
//  MatrixXd Qf = MatrixXd::Zero(line_pusher->numxcStates, line_pusher->numxcStates);
//  MatrixXd R = MatrixXd::Zero(line_pusher->numucStates, line_pusher->numucStates);

  int x_index;
  int u_index;

//  if (line_pusher->num_contact_points==1) {
//
//    if (line_pusher->numucStates==3){
//      Q.diagonal() << 3,3,.1,0.0;
//      Q=Q*10;
//      Qf.diagonal() << 3,3,.1,0.0;
//      Qf=Qf*2000;
//      R.diagonal() << 1,1,0.01;
////    R = R*.5; //LMODES
//      R = R*.01; //FOM
//    }
//    else{
//      Q.diagonal() << 1,1,.01,0.1;
//      Q=Q*1;
//      Qf.diagonal() << 1,1,.1,0.1;
//      Qf=Qf*2000;
//      R.diagonal() << 1,1;
//      R = R*1; //FOM
//      }
//  }
//  else{
////    Q.diagonal() << 1,5,0.1,0.1;
////    Q=Q*10;
////    Qf.diagonal() << 1,5,1,0.1;
////    Qf=Qf*2000;
////    R.diagonal() << 1,1,1,1,0.1;
////    R = R*.5;
//    Q.diagonal() << 1,1,1,0.1;
//    Q=Q*10;
//    Qf.diagonal() << 1,1,1,0.1;
//    Qf=Qf*2000;
//    R.diagonal() << 1,1,1,1,0.1;
//    R = R*.5;
//  }


  for (int i= 0; i<steps; i++){

    x_index = getStateIndex(i);
    u_index = getControlIndex(i);
    //1. build cost
    if (i==steps-1){
      Helper::assign_sparse(matricesMPC.Q_vec, Qf, x_index, x_index);
    }
    else {
      Helper::assign_sparse(matricesMPC.Q_vec, Q, x_index, x_index);
    }
    Helper::assign_sparse(matricesMPC.Q_vec, R, u_index, u_index);
  }
}



void MPC::addMotionConstraints(VectorXd& xc_star,VectorXd& uc_star, int lv1){

  outBuildMotionConstraints out_solution;
  out_solution = buildMotionConstraints(xc_star, uc_star, -pusher_slider->a/2., line_pusher->d, friction->A_ls);

  int x1_index = getStateIndex(lv1-1);
  int x2_index = getStateIndex(lv1);
  int u1_index = getControlIndex(lv1);

  Helper::assign_sparse(matricesMPC.Aeq_vec, -out_solution.A_bar, matricesMPC.row_start_eq, x1_index);
  Helper::assign_sparse(matricesMPC.Aeq_vec, MatrixXd::Identity(line_pusher->numxcStates, line_pusher->numxcStates), matricesMPC.row_start_eq, x2_index);
  Helper::assign_sparse(matricesMPC.Aeq_vec, -out_solution.B_bar, matricesMPC.row_start_eq, u1_index);

  matricesMPC.row_start_eq = matricesMPC.row_start_eq + line_pusher->numxcStates;
}

void MPC::addVelConstraintsGPData(outStateNominal out_state_nominal, int lv1){

  outBuildVelConstraintsGPData out_solution;
  out_solution = buildVelConstraintsGPData(out_state_nominal);

  int x1_index = getStateIndex(lv1-1);
  int x2_index = getStateIndex(lv1);
  int u1_index = getControlIndex(lv1);

  Helper::assign_sparse(matricesMPC.Ain_vec, out_solution.Ain, matricesMPC.row_start_ineq, u1_index);
  matricesMPC.bin.segment(matricesMPC.row_start_ineq, out_solution.Ain.rows()) = out_solution.bin;
  matricesMPC.row_start_ineq = matricesMPC.row_start_ineq + out_solution.Ain.rows();

}

void MPC::addMotionConstraintsGPData(outStateNominal out_state_nominal, int lv1){

  MatrixXd A_bar=MatrixXd::Identity(line_pusher->numxcStates, line_pusher->numxcStates) + h*out_state_nominal.AStar;
  MatrixXd B_bar=h*out_state_nominal.BStar;

  int x1_index = getStateIndex(lv1-1);
  int x2_index = getStateIndex(lv1);
  int u1_index = getControlIndex(lv1);
//
  Helper::assign_sparse(matricesMPC.Aeq_vec, -A_bar, matricesMPC.row_start_eq, x1_index);
  Helper::assign_sparse(matricesMPC.Aeq_vec, MatrixXd::Identity(line_pusher->numxcStates, line_pusher->numxcStates), matricesMPC.row_start_eq, x2_index);
  Helper::assign_sparse(matricesMPC.Aeq_vec, -B_bar, matricesMPC.row_start_eq, u1_index);

  matricesMPC.row_start_eq = matricesMPC.row_start_eq + line_pusher->numxcStates;
}

void MPC::addICConstraints(VectorXd& xc_star, VectorXd& uc_star, int lv1, VectorXd delta_xc){
  outBuildMotionConstraints out_solution;
  out_solution = buildMotionConstraints(xc_star, uc_star, -pusher_slider->a/2., line_pusher->d, friction->A_ls);

  int x2_index = getStateIndex(lv1);
  int u1_index = getControlIndex(lv1);

  Helper::assign_sparse(matricesMPC.Aeq_vec,  MatrixXd::Identity(line_pusher->numxcStates, line_pusher->numxcStates), matricesMPC.row_start_eq, x2_index);
  Helper::assign_sparse(matricesMPC.Aeq_vec,  -out_solution.B_bar, matricesMPC.row_start_eq, u1_index);
  matricesMPC.beq.segment(matricesMPC.row_start_eq, line_pusher->numxcStates) = out_solution.A_bar*delta_xc;
  matricesMPC.row_start_eq = matricesMPC.row_start_eq + line_pusher->numxcStates;
}

void MPC::addICConstraintsGPData(outStateNominal out_state_nominal, int lv1, VectorXd delta_xc){
  int x2_index = getStateIndex(lv1);
  int u1_index = getControlIndex(lv1);

  MatrixXd A_bar=MatrixXd::Identity(line_pusher->numxcStates, line_pusher->numxcStates) + h*out_state_nominal.AStar;
  MatrixXd B_bar=h*out_state_nominal.BStar;
//  cout<<"delta_xc"<<endl;
//  cout<<delta_xc<<endl;
//  cout<<"A_bar"<<endl;
//  cout<<A_bar<<endl;
//  cout<<"B_bar"<<endl;
//  cout<<B_bar<<endl;

  Helper::assign_sparse(matricesMPC.Aeq_vec,  MatrixXd::Identity(line_pusher->numxcStates, line_pusher->numxcStates), matricesMPC.row_start_eq, x2_index);
  Helper::assign_sparse(matricesMPC.Aeq_vec,  -B_bar, matricesMPC.row_start_eq, u1_index);
  matricesMPC.beq.segment(matricesMPC.row_start_eq, line_pusher->numxcStates) = A_bar*delta_xc;
  matricesMPC.row_start_eq = matricesMPC.row_start_eq + line_pusher->numxcStates;
}

void MPC::addForceDepConstraints(VectorXd& xc_star,VectorXd& uc_star, int mode, int lv1){
  //build constraints matrices (in local time step coordinates)
  outBuildForceDepConstraints out_small_matrices;
  out_small_matrices = buildForceDepConstraints(xc_star, uc_star, mode);

  //~ //build constraints matrices (in global time step coordinates)
  int u_index = getControlIndex(lv1);

  Helper::assign_sparse(matricesMPC.Aeq_vec, out_small_matrices.Aeq, matricesMPC.row_start_eq, u_index);
  matricesMPC.beq.segment(matricesMPC.row_start_eq, out_small_matrices.Aeq.rows()) = out_small_matrices.beq;
  matricesMPC.row_start_eq = matricesMPC.row_start_eq + out_small_matrices.Aeq.rows();

  if (out_small_matrices.Ain.nonZeros()!=0){

    Helper::assign_sparse(matricesMPC.Ain_vec, out_small_matrices.Ain, matricesMPC.row_start_ineq, u_index);
    matricesMPC.bin.segment(matricesMPC.row_start_ineq, out_small_matrices.Ain.rows()) = out_small_matrices.bin;
    matricesMPC.row_start_ineq = matricesMPC.row_start_ineq + out_small_matrices.Ain.rows();

  }
}

void MPC::addForceIndConstraints(VectorXd& xc_star,VectorXd& uc_star, int lv1){
  //build constraints matrices (in local time step coordinates)
  outBuildForceIndConstraints out_small_matrices;
  out_small_matrices = buildForceIndConstraints(xc_star, uc_star);

  //build constraints matrices (in global time step coordinates)
  int u_index = getControlIndex(lv1);

  Helper::assign_sparse(matricesMPC.Ain_vec, out_small_matrices.Ain, matricesMPC.row_start_ineq, u_index);
  matricesMPC.bin.segment(matricesMPC.row_start_ineq, out_small_matrices.Ain.rows()) = out_small_matrices.bin;
  matricesMPC.row_start_ineq = matricesMPC.row_start_ineq + out_small_matrices.Ain.rows();
}

void MPC::buildConstraintMatrices(double time, VectorXd mode_schedule, VectorXd delta_xc){

  outStateNominal out_state_nominal;

//  double t0 = Helper::gettime();
  for (int i= 0; i<steps; i++){

    out_state_nominal = line_pusher->getStateNominal(time);

    //1. build cost
    //2. build motion constraints

    if (i==0){
      addICConstraints(out_state_nominal.xcStar, out_state_nominal.ucStar, i, delta_xc);
      addVelConstraints(out_state_nominal.xcStar, out_state_nominal.ucStar, i, delta_xc);
    }
    else {
      addMotionConstraints(out_state_nominal.xcStar, out_state_nominal.ucStar, i);
    }
    //3. build force independant constraints
    addForceIndConstraints(out_state_nominal.xcStar, out_state_nominal.ucStar, i);
    //4. build force dependant constraints
    addForceDepConstraints(out_state_nominal.xcStar, out_state_nominal.ucStar, mode_schedule(i), i);

    //5. increment time
    time = time + h;
  }

}
void MPC::buildConstraintMatricesGPData(double time, VectorXd delta_xc){

  outStateNominal out_state_nominal;

//  double t0 = Helper::gettime();
  for (int i= 0; i<steps; i++){

    out_state_nominal = line_pusher->getStateNominalGPData(time);

    if (i==0){
      addICConstraintsGPData(out_state_nominal, i, delta_xc);
    }
    else {
      addMotionConstraintsGPData(out_state_nominal, i);
    }
    addVelConstraintsGPData(out_state_nominal, i);
    time = time + h;
  }
}

outBuildMotionConstraints MPC::buildMotionConstraints(VectorXd& xc_star, VectorXd& uc_star, double rx, double d, MatrixXd A_ls){
  MatrixXd A_bar_tmp(xc_star.size(),xc_star.size());
  MatrixXd B_bar_tmp(xc_star.size(),uc_star.size());
//
//  cout<<"d"<<d<<endl;
//  cout<<"xc_star"<<xc_star<<endl;
//  cout<<"uc_star"<<uc_star<<endl;
//  cout<<"line_pusher->A_fun(xc_star, uc_star, rx, d, A_ls)"<<line_pusher->A_fun(xc_star, uc_star, rx, d, A_ls)<<endl;
//  cout<<"line_pusher->B_fun(xc_star, uc_star, rx, d, A_ls)"<<line_pusher->B_fun(xc_star, uc_star, rx, d, A_ls)<<endl;
  A_bar_tmp = MatrixXd::Identity(xc_star.size(),xc_star.size()) + h*line_pusher->A_fun(xc_star, uc_star, rx, d, A_ls);
  B_bar_tmp = h*line_pusher->B_fun(xc_star, uc_star, rx, d, A_ls);

  outBuildMotionConstraints out_solution;
  out_solution.A_bar = A_bar_tmp;
  out_solution.B_bar = B_bar_tmp;

  return out_solution;
  }

outBuildForceIndConstraints MPC::buildForceIndConstraints(VectorXd& xc_star,VectorXd& uc_star) {

    MatrixXd Ain(2 * line_pusher->num_contact_points, uc_star.size());
    Ain = MatrixXd::Zero(2 * line_pusher->num_contact_points, uc_star.size());
    VectorXd bin(2 * line_pusher->num_contact_points);

    VectorXd index(2);
    MatrixXd Ain_tmp1(1, 2);
    MatrixXd Ain_tmp2(1, 2);
    VectorXd bin_tmp(1);
    int num_constraints;
    int counter = 0;

    for (int j = 0; j < line_pusher->num_contact_points; j++) {
        index << j, j + line_pusher->num_contact_points;
        VectorXd un_tmp(index.size());
        Ain_tmp1 << -friction->nu_p, 1;
        Ain_tmp2 << -friction->nu_p, -1;
        for (int lv1=0; lv1 < index.size(); lv1++) {
            un_tmp(lv1) = uc_star(index(lv1));
            Ain(counter,index(lv1))  =  Ain_tmp1(0, lv1);
            Ain(counter + 1,index(lv1)) = Ain_tmp2(0, lv1);
        }

//        //upper border
        num_constraints = 1;
        bin_tmp = -Ain_tmp1 * un_tmp;
        bin(counter) = bin_tmp(0);
        //lower border
        num_constraints = 1;
        bin_tmp = -Ain_tmp2 * un_tmp;
        bin(counter + 1) = bin_tmp(0);
        counter = counter +2;
    }

  outBuildForceIndConstraints out_solution;
    out_solution.Ain = Ain;
    out_solution.bin = bin;

    return out_solution;
}

outBuildVelConstraintsGPData MPC::buildVelConstraintsGPData(outStateNominal out_state_nominal) {

    float low_bound = .01;
    float up_bound = 1.;
    MatrixXd Ain(4, line_pusher->numucStates);
    Ain << 1,0,0,1,-1,0,0,-1;
    VectorXd bin(4);
    bin<<up_bound-out_state_nominal.ucStar(0), up_bound - out_state_nominal.ucStar(1),
      out_state_nominal.ucStar(0)-low_bound, up_bound + out_state_nominal.ucStar(1);

    outBuildVelConstraintsGPData out_solution;
    out_solution.Ain = Ain;
    out_solution.bin = bin;

    return out_solution;
}

outBuildForceDepConstraints MPC::buildForceDepConstraints(VectorXd& xc_star, VectorXd& uc_star, int mode) {

    int num_constraints;
    outBuildForceDepConstraints out_solution;

    if (mode==0){
        VectorXd index(1);
        MatrixXd Aeq_tmp(1,1);
        MatrixXd Aeq(1,uc_star.size());
        Aeq = MatrixXd::Zero(1,uc_star.size());
        VectorXd beq(1);
        MatrixXd Ain(1,1);
        VectorXd bin(1);
        num_constraints = 1;
        index << 2*line_pusher->num_contact_points;
        VectorXd un_tmp(index.size());
        Aeq_tmp << 1;
        for (int lv1=0; lv1 < index.size(); lv1++) {
          un_tmp(lv1) = uc_star(index(lv1));
          Aeq(0,index(lv1))  =  Aeq_tmp(0, lv1);
        }

        beq = -Aeq_tmp*un_tmp;
        out_solution.Aeq = Aeq;
        out_solution.beq = beq;
    }
    else if (mode==1){
        MatrixXd Aeq(line_pusher->num_contact_points, uc_star.size());
        MatrixXd Ain(1, uc_star.size());
        Aeq = MatrixXd::Zero(line_pusher->num_contact_points, uc_star.size());
        Ain = MatrixXd::Zero(1, uc_star.size());
        VectorXd beq(line_pusher->num_contact_points);
        MatrixXd Aeq_tmp(1,2);
        VectorXd beq_tmp(1);
        MatrixXd Ain_tmp(1, 1);
        VectorXd bin(1);

        //sliding up
        num_constraints = 1;
        VectorXd index1(1);
        index1 << 2*line_pusher->num_contact_points;
        VectorXd un_tmp1(index1.size());
        Ain_tmp << -1;
        for (int lv1=0; lv1 < index1.size(); lv1++) {
            un_tmp1(lv1) = uc_star(index1(lv1));
            Ain(0,index1(lv1))  =  Ain_tmp(0, lv1);
        }
        bin = -Ain_tmp*un_tmp1;
        //friction cone borders
        for (int j = 0; j < line_pusher->num_contact_points; j++) {
            VectorXd index2(2);
            index2 << j, j + line_pusher->num_contact_points;
            VectorXd un_tmp2(index2.size());

            Aeq_tmp << -friction->nu_p, 1;
            for (int lv1=0; lv1 < index2.size(); lv1++) {
                un_tmp2(lv1) = uc_star(index2(lv1));
                Aeq(j,index2(lv1))  =  Aeq_tmp(0, lv1);
            }
//            //upper border
            num_constraints = 1;

            beq_tmp = -Aeq_tmp * un_tmp2;
            beq(j) = beq_tmp(0);
        }
        out_solution.Aeq = Aeq;
        out_solution.beq = beq;
        out_solution.Ain = Ain;
        out_solution.bin = bin;
    }
    else if (mode==2){
      
        MatrixXd Aeq(line_pusher->num_contact_points, uc_star.size());
        MatrixXd Ain(1, uc_star.size());
        Aeq = MatrixXd::Zero(line_pusher->num_contact_points, uc_star.size());
        Ain = MatrixXd::Zero(1, uc_star.size());
        VectorXd beq(line_pusher->num_contact_points);
        MatrixXd Aeq_tmp(1,2);
        VectorXd beq_tmp(1);
        MatrixXd Ain_tmp(1, 1);
        VectorXd bin(1);
      
        //sliding up
        num_constraints = 1;
        VectorXd index1(1);
        index1 << 2*line_pusher->num_contact_points;
        VectorXd un_tmp1(index1.size());
        Ain_tmp << 1;
        for (int lv1=0; lv1 < index1.size(); lv1++) {
            un_tmp1(lv1) = uc_star(index1(lv1));
            Ain(0,index1(lv1))  =  Ain_tmp(0, lv1);
        }
        bin = -Ain_tmp*un_tmp1;
        
        //friction cone borders
        for (int j = 0; j < line_pusher->num_contact_points; j++) {
            VectorXd index2(2);
            index2 << j, j + line_pusher->num_contact_points;
            VectorXd un_tmp2(index2.size());
            Aeq_tmp << friction->nu_p, 1;
            for (int lv1=0; lv1 < index2.size(); lv1++) {
                un_tmp2(lv1) = uc_star(index2(lv1));
                Aeq(j,index2(lv1))  =  Aeq_tmp(0, lv1);
            }
            //upper border
            num_constraints = 1;
            beq_tmp = -Aeq_tmp * un_tmp2;
            beq(j) = beq_tmp(0);
        }
        out_solution.Aeq = Aeq;
        out_solution.beq = beq;
        out_solution.Ain = Ain;
        out_solution.bin = bin;
    }

    out_solution.mode = mode;
    return out_solution;
}


void MPC::addVelConstraints(VectorXd& xc_star, VectorXd& uc_star, int lv1, VectorXd delta_xc)
{
  VectorXd xc(delta_xc.size());
  xc = delta_xc + xc_star;
  //int
  double rx, ry;
  //Vector
  Vector2d rbpb;
  Vector2d sign_vec;
  Vector3d n3;
  VectorXd tp_max(3);
  VectorXd tp_max2(3);
  Vector3d bin_tp;
  Vector3d bin_hat;
  Vector3d bin_hat2;
  VectorXd bin_tot(6);
  Vector2d rbcb;
  Vector2d rbcm;
  Vector2d n;
  Vector2d t;
  //Matrices
  MatrixXd G(2, uc_star.size());
  MatrixXd I12(2, 3);
  MatrixXd N_tot(line_pusher->num_contact_points, 3);
  MatrixXd T_tot(line_pusher->num_contact_points, 3);
  MatrixXd N(1, 3);
  MatrixXd T(1, 3);
  MatrixXd Jb(2,3);
  MatrixXd E(3,uc_star.size());
  MatrixXd F(2,uc_star.size());
  MatrixXd O(2,uc_star.size());
  MatrixXd H(3,uc_star.size());
  MatrixXd H1(2,uc_star.size());
  MatrixXd H2(1,uc_star.size());
  MatrixXd W(2,2);
  MatrixXd Ain_tp(3,3);
  MatrixXd Ain_hat(3,uc_star.size());
  MatrixXd Ain_hat2(3,uc_star.size());
  MatrixXd Ain_tot(6,uc_star.size());

  rx = -pusher_slider->a/2.0;
  ry = xc[3];
  rbpb<<rx,ry;
  G  = MatrixXd::Zero(2,uc_star.size());
  G.bottomRightCorner(1,1) <<  1;
  I12 << 1,0,0,0,1,0;
  n3<<0,0,1;
  sign_vec << 1,-1;
  for (int i=0;i<line_pusher->num_contact_points;i++){
    rbcm << 0, line_pusher->d;
    rbcb = rbpb + sign_vec(i)*rbcm; //position of contact point lv1
    Jb <<  1, 0, -rbcb(1),0, 1, rbcb(0);
    n << 1,0;
    t << 0,1;

    N = n.transpose()*Jb;
    T = t.transpose()*Jb;

    N_tot.middleRows(i, 1) = N;
    T_tot.middleRows(i, 1) = T;
  }

  E <<N_tot.transpose(), T_tot.transpose(), MatrixXd::Zero(3,1);// [N_tot' T_tot' zeros(3,1)];
  F = I12*friction->A_ls*E;
  W<<0,-1,1,0;
  O = W*rbpb*n3.transpose()*friction->A_ls*E;
  H1 = F + G + O;
  H2 = n3.transpose()*friction->A_ls*E;
  H<<H1,H2;

  tp_max << 0.3, 0.4, 1;
  tp_max2 << -0.03, 0.4, 1;

  //upper bound twist constraints
  Ain_tp << 1,0,0,0,1,0,0,0,1;
  bin_tp = tp_max;
  Ain_hat =Ain_tp*H;
  bin_hat =bin_tp - Ain_tp*H*uc_star;

  //lower bound twist constraints
  Ain_tp << 1,0,0,0,1,0,0,0,1;
  bin_tp = tp_max2;
  Ain_hat2 =-Ain_tp*H;
  bin_hat2 =bin_tp + Ain_tp*H*uc_star;

  //concatenate matrices
  Ain_tot << Ain_hat,Ain_hat2;
  bin_tot << bin_hat,bin_hat2;

  int u1_index = getControlIndex(lv1);
//
  MatrixXd Ain_tmp = MatrixXd::Zero(6, num_variables);
  VectorXd bin_tmp = VectorXd::Zero(6);

  Ain_tmp.middleCols(u1_index, line_pusher->numucStates)= Ain_tot;
  bin_tmp = bin_tot;

  Helper::assign_sparse(matricesMPC.Ain_vec, Ain_tot, matricesMPC.row_start_ineq, u1_index);
  matricesMPC.bin.segment(matricesMPC.row_start_ineq, bin_tot.size()) = bin_tot;
  matricesMPC.row_start_ineq = matricesMPC.row_start_ineq + Ain_tot.rows();

}
