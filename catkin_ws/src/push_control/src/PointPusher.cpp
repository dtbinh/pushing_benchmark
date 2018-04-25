//
// Created by mcube10 on 10/5/17.
//

//Common libraries
#include <iostream>
#include <Eigen/Dense>
#include <fstream>
#include <unistd.h>


//Custom Libraries
#include "PusherSlider.h"
#include "Helper.h"
#include "PointPusher.h"
//#include "StructuresPointPusher.h"
#include "Friction.h"

using namespace std;
using namespace Eigen;

PointPusher::PointPusher(PusherSlider* _pusher_slider, Friction* _friction): Pusher()
{
    lp=0.03;
    d = 0.;
    numxcStates = 4;
    numucStates = 3;
    numxsStates = 6;
    numusStates = 3;
    pusher_type = "point";
    num_contact_points = 1;
    ppusher_slider = _pusher_slider;
    pfriction = _friction;


//Static Eigen Constructor
    uc_eq = buildUcEq();
    xc_eq = buildXcEq();;
    struct outStarStruct outStar;
//    outStar = buildNominalTrajectory(0.001);
//    outStar = buildStraightLineTrajectory("0_05");
//    outStar = build8TrackTrajectory("0_05");
//    outStar = buildTrajectory("/Simulation/Data/8Track_point_pusher_vel_0_05.json");
//    outStar = buildTrajectory("/Simulation/Data/inf_circle_point_pusher_vel_0_05.json");
//    outStar = buildTrajectory("/Simulation/Data/inf_circle_point_pusher_vel_0_05_radius_neg_0_015.json");
    outStar = buildTrajectory("/Simulation/Data/8Track_point_pusher_radius_0_15_vel_0_05_infinity.json");
    xc_star = outStar.xcStar;
    uc_star = outStar.ucStar;
    xs_star = outStar.xsStar;
    us_star = outStar.usStar;
    t_star  = outStar.tStar;

  struct outStarStruct outStar2;
    outStar2 = outStar;
//  outStar2 = buildNominalTrajectory(0.03);
//    outStar = buildStraightLineTrajectory("0_05");
//    outStar = build8TrackTrajectory("0_05");
//
//
//  xc_star2 = outStar2.xcStar;
//    uc_star2 = outStar2.ucStar;
//  xs_star2 = outStar2.xsStar;
//  us_star2 = outStar2.usStar;
//    cout<< outStar.tStar;
//    t_star2 = outStar2.tStar;
//    cout<<"finish"<<endl;

}

VectorXd PointPusher::buildUcEq()
{
    VectorXd tmp(numusStates);
    tmp <<  0.3268,0,0;
    return tmp;
}

VectorXd PointPusher::buildXcEq()
{
    VectorXd tmp(numxcStates);
    tmp << 0,0,0,0;
    return tmp;
}

outStarStruct PointPusher::buildNominalTrajectory(double h_star){
    double v_eq = 0.05;
    double t0 = 0;
    double tf = 7;
//    double h_star = 0.001;
    int N_star = (1/h_star)*(tf-t0);
    VectorXd t_star  = VectorXd::Zero(N_star);
    MatrixXd xsStar(N_star, numxsStates);
    MatrixXd usStar(N_star, numusStates);
    MatrixXd xcStar(N_star, numxcStates);
    MatrixXd ucStar(N_star, numucStates);
    VectorXd tStar(N_star);

    for ( int lv1 =0; lv1<N_star; lv1+=1){
        //~ Define nominal values (simulator coordinates)
        xsStar.row(lv1) <<  v_eq*tStar(lv1) ,0, 0, v_eq*tStar(lv1)-ppusher_slider->a/2, 0, 0;
        usStar.row(lv1) <<  v_eq, 0, 0;
        //~ %Define nominal values (controller coordinates)
        xcStar.row(lv1) = coordinateTransformSC(xsStar.row(lv1));
        ucStar.row(lv1)= uc_eq;
        //~ Build nominal time vector
        if (lv1<N_star-1){
            tStar(lv1+1)  = tStar(lv1) + h_star;
        }
    }
    // Return outStar object
    struct outStarStruct outStar;
    outStar.xcStar = xcStar;
    outStar.ucStar = ucStar;
    outStar.xsStar = xsStar;
    outStar.usStar = usStar;
    outStar.tStar = tStar;
    return outStar;
}


outStarStruct PointPusher::buildStraightLineTrajectory(string v_eq){

    Json::Value root;
    Json::Reader reader;
    char const* tmp = getenv( "PUSHING_BENCHMARK_BASE" );
    string envStr( tmp );
    string fileName;
    fileName = envStr + "/Simulation/Data/Straight_point_pusher_vel_" + v_eq + ".json";
//    cout<<fileName<<endl;
    ifstream file(fileName);
    file >> root;

    int N_star = root["Matrices"]["t_star"].size();
    VectorXd t_star  = VectorXd::Zero(N_star);
    MatrixXd xc_star(N_star, numxcStates);
    MatrixXd uc_star(N_star, numucStates);
    MatrixXd xs_star(N_star, numxsStates);
    MatrixXd us_star(N_star, numusStates);
    VectorXd tStar(N_star);

    Helper::write_matrix_JSON(root["Matrices"]["xc_star"], xc_star);
    Helper::write_matrix_JSON(root["Matrices"]["uc_star"], uc_star);
    Helper::write_matrix_JSON(root["Matrices"]["xs_star"], xs_star);
    Helper::write_matrix_JSON(root["Matrices"]["us_star"], us_star);
    Helper::write_vector_JSON(root["Matrices"]["t_star"], t_star);

    // Return outStar object
    struct outStarStruct outStar;
    outStar.xcStar = xc_star;
    outStar.ucStar = uc_star;
    outStar.xsStar = xs_star;
    outStar.usStar = us_star;
    outStar.tStar = t_star;

    return outStar;
}

outStarStruct PointPusher::build8TrackTrajectory(string v_eq){

    Json::Value root;
    Json::Reader reader;
    char const* tmp = getenv( "PUSHING_BENCHMARK_BASE" );
    string envStr( tmp );
    string fileName;
    fileName = envStr + "/Simulation/Data/8Track_point_pusher_vel_" + v_eq + ".json";
//    cout<<fileName<<endl;
    ifstream file(fileName);
    file >> root;

    int N_star = root["Matrices"]["t_star"].size();
    VectorXd t_star  = VectorXd::Zero(N_star);
    MatrixXd xc_star(N_star, numxcStates);
    MatrixXd uc_star(N_star, numucStates);
    MatrixXd xs_star(N_star, numxsStates);
    MatrixXd us_star(N_star, numusStates);
    VectorXd tStar(N_star);

    Helper::write_matrix_JSON(root["Matrices"]["xc_star"], xc_star);
    Helper::write_matrix_JSON(root["Matrices"]["uc_star"], uc_star);
    Helper::write_matrix_JSON(root["Matrices"]["xs_star"], xs_star);
    Helper::write_matrix_JSON(root["Matrices"]["us_star"], us_star);
    Helper::write_vector_JSON(root["Matrices"]["t_star"], t_star);

    // Return outStar object
    struct outStarStruct outStar;
    outStar.xcStar = xc_star;
    outStar.ucStar = uc_star;
    outStar.xsStar = xs_star;
    outStar.usStar = us_star;
    outStar.tStar = t_star;

    return outStar;
}
outStarStruct PointPusher::buildTrajectory(string file_name){

    Json::Value root;
    Json::Reader reader;

    char const* tmp = getenv( "PUSHING_BENCHMARK_BASE" );
    string envStr( tmp );
    string fileName;
    fileName = envStr + file_name;
//    cout<<fileName<<endl;
    ifstream file(fileName);

    file >> root;

    int N_star = root["Matrices"]["t_star"].size();
    VectorXd t_star  = VectorXd::Zero(N_star);
    MatrixXd xc_star(N_star, numxcStates);
    MatrixXd uc_star(N_star, numucStates);
    MatrixXd xs_star(N_star, numxsStates);
    MatrixXd us_star(N_star, numusStates);
    VectorXd tStar(N_star);

    Helper::write_matrix_JSON(root["Matrices"]["xc_star"], xc_star);
    Helper::write_matrix_JSON(root["Matrices"]["uc_star"], uc_star);
    Helper::write_matrix_JSON(root["Matrices"]["xs_star"], xs_star);
    Helper::write_matrix_JSON(root["Matrices"]["us_star"], us_star);
    Helper::write_vector_JSON(root["Matrices"]["t_star"], t_star);

    // Return outStar object
    struct outStarStruct outStar;
    outStar.xcStar = xc_star;
    outStar.ucStar = uc_star;
    outStar.xsStar = xs_star;
    outStar.usStar = us_star;
    outStar.tStar = t_star;

    return outStar;
}

outStateNominal PointPusher::getStateNominal(double t) {
//            % Get nominal trajectory values at time T
//            %
//            %   Parameters:
//            %   t       -   Time at which the nominal state is evaluated

    VectorXd vecDif(t_star.rows());
    MatrixXf::Index   minIndex;
    double minVal;
    outStateNominal out_state_nominal;

    vecDif = (t*VectorXd::Ones(t_star.rows()) - t_star).cwiseAbs();
    minVal = vecDif.minCoeff(&minIndex);

    out_state_nominal.xcStar = xc_star.row(minIndex);
    out_state_nominal.ucStar = uc_star.row(minIndex);
    out_state_nominal.xsStar = xs_star.row(minIndex);
    out_state_nominal.usStar = us_star.row(minIndex);

    return out_state_nominal;
}


outStateNominal PointPusher::getStateNominal2(double t) {
//            % Get nominal trajectory values at time T
//            %
//            %   Parameters:
//            %   t       -   Time at which the nominal state is evaluated

  VectorXd vecDif(t_star2.rows());
  MatrixXf::Index   minIndex;
  double minVal;
  outStateNominal out_state_nominal;

  double t0 = Helper::gettime();
  vecDif = (t*VectorXd::Ones(t_star2.rows()) - t_star2).cwiseAbs();
  double tf = Helper::gettime();

  t0 = Helper::gettime();
  minVal = vecDif.minCoeff(&minIndex);
  tf = Helper::gettime();

  out_state_nominal.xcStar = xc_star2.row(minIndex);
  out_state_nominal.ucStar = uc_star2.row(minIndex);
  out_state_nominal.xsStar = xs_star2.row(minIndex);
  out_state_nominal.usStar = us_star2.row(minIndex);

  return out_state_nominal;
}



VectorXd PointPusher::getError(VectorXd xc, double t) {
    VectorXd delta_xc(xc.rows());
    outStateNominal out_state_nominal = getStateNominal(t);
//    cout<<"out_state_nominal.xcStar"<<out_state_nominal.xcStar<<endl;
//    cout<<"out_state_nominal.ucStar"<<out_state_nominal.ucStar<<endl;
    delta_xc = xc - out_state_nominal.xcStar;

    return delta_xc;
}

VectorXd PointPusher::coordinateTransformSC(VectorXd xs){
//    xs = offsetPusher(xs);
    //Declare matrices
    Matrix2d Cbi;
    Vector2d rbbi;
    Vector2d rbpi;
    Vector2d rbpb;
    VectorXd xc(numxcStates);
    double ry;
    struct xsStateStruct xsState(xs);
    //~ %Direction Cosine Matrices
    Cbi = Helper::C3_2d(xsState.theta);
    //~ %Convert state to body frame
    rbbi = Cbi*xsState.ribi;
    rbpi = Cbi*xsState.ripi;
    //~ %Build xc
    rbpb = rbpi - rbbi;
    ry = rbpb(1);
    //~ %Output
    xc << xsState.ribi, xsState.theta, ry;
    return xc;
}

VectorXd PointPusher::offsetPusher(VectorXd xs){
    Vector2d ribi;
    Vector2d ripi;
    Vector2d rici;
    Vector2d rpcp;
    ribi = xs.segment(0,2);
    double theta = xs(2);

    ripi = xs.segment(3,2);


    double thetap = xs(5);

    Matrix2d Cbi;
    Matrix2d Cpi;
    Cpi = Helper::C3_2d(thetap);
    Cbi = Helper::C3_2d(theta);
    rpcp << d, 0;
    rici = ripi + Cpi.transpose()*rpcp;
    VectorXd xs_offset(6);
    xs_offset << ribi, theta, rici, thetap;

    return xs_offset;
}

VectorXd PointPusher::coordinateTransformCS(VectorXd xc)
{
    //Declare matrices
    Matrix2d Cbi;
    Vector2d ripi;
    Vector2d ripb;
    Vector2d rbbi;
    Vector2d rbpb;
    VectorXd xs(numxsStates);
    struct xcStateStruct xcState(xc);
    //~ %Direction Cosine Matrices
    Cbi = Helper::C3_2d(xcState.theta);
    //~ %Convert state to body frame
    rbbi = Cbi*xcState.ribi;
    rbpb << -ppusher_slider->a/2, xcState.ry;
    ripb =Cbi.transpose()*rbpb;
    ripi =xcState.ribi+ripb;
    //~ %Output
    xs << xcState.ribi, xcState.theta, ripi, 0;
    return xs;
}

VectorXd PointPusher::force2Velocity(VectorXd xc, VectorXd uc){
    //Declare matrices
    struct xcStateStruct xcState(xc);
    struct ucStateStruct ucState(uc);
    Matrix2d Cbi;
    Vector2d drbpb;
    Vector2d rbbi;
    Vector2d rbpb;
    Vector2d ripb;
    Vector2d np_b;
    Vector2d tp_b;
    MatrixXd Jp(2,3);
    MatrixXd N_tot(num_contact_points,3);
    MatrixXd T_tot(num_contact_points,3);
    Vector3d twist_body_b;
    Vector3d twist_body_i;
    Vector3d twist_pusher_i;
    MatrixXd Sib(3,3);
    //~ %Direction Cosine Matrices
    Cbi = Helper::C3_2d(xcState.theta);
    //~ %Convert state to body frame
    rbbi = Cbi*xcState.ribi;
    rbpb << -ppusher_slider->a/2, xcState.ry;
    ripb =Cbi.transpose()*rbpb;
    drbpb << 0, ucState.dry;
    //~ %kinematics
    np_b << 1,0;
    tp_b << 0,1;
    Jp << 1, 0, -rbpb(1), 0, 1, rbpb(0);
    //build matrices point a
    N_tot = np_b.transpose()*Jp;
    T_tot = tp_b.transpose()*Jp;
    //Compute twist
    twist_body_b = pfriction->A_ls*(N_tot.transpose()*ucState.fn + T_tot.transpose()*ucState.ft);
    Sib = Helper::buildSib(xcState.theta);//[Cbi' [0;0];0 0 1];
    twist_body_i = Sib*twist_body_b;
//    ~ %Compute twist of pusher
    twist_pusher_i.segment(0,2) = twist_body_i.segment(0,2) + Cbi.transpose()*drbpb + Helper::cross3d(twist_body_i(2), ripb);
    twist_pusher_i(2) = 0;

    return twist_pusher_i;
}

MatrixXd PointPusher::A_fun(VectorXd xc_star, VectorXd uc_star, double rx, double d, MatrixXd A_ls){

    double A_ls1_1 = A_ls(0,0);
    double A_ls1_2 = A_ls(1,0);
    double A_ls1_3 = A_ls(2,0);
    double A_ls2_1 = A_ls(0,1);
    double A_ls2_2 = A_ls(1,1);
    double A_ls2_3 = A_ls(2,1);
    double A_ls3_1 = A_ls(0,2);
    double A_ls3_2 = A_ls(1,2);
    double A_ls3_3 = A_ls(2,2);
    double rbpb1 = rx;
    double rbpb2 = xc_star(3);
    double uc1 = uc_star(0);
    double uc2 = uc_star(1);
    double xo3 = xc_star(2);
    double t2 = d+rbpb2;
    double t3 = rbpb1*uc2;
    double t9 = t2*uc1;
    double t4 = t3-t9;
    double t5 = sin(xo3);
    double t6 = cos(xo3);
    double t7 = A_ls1_1*uc1;
    double t8 = A_ls1_2*uc2;
    double t10 = A_ls2_1*uc1;
    double t11 = A_ls2_2*uc2;
    double t12 = A_ls2_3*t4;
    double t13 = t10+t11+t12;
    MatrixXd A_tmp(xc_star.size(),xc_star.size());
    MatrixXd A(xc_star.size(),xc_star.size());
    A_tmp <<0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,-t5*(t7+t8+A_ls1_3*t4)-t6*t13,-t5*t13+t6*(t7+t8+A_ls1_3*(t3-t9)),0.0,0.0,-A_ls1_3*t6*uc1+A_ls2_3*t5*uc1,-A_ls1_3*t5*uc1-A_ls2_3*t6*uc1,-A_ls3_3*uc1,0.0;
    A = A_tmp.transpose();

    return A;
}

MatrixXd PointPusher::B_fun(VectorXd xc_star, VectorXd uc_star, double rx, double d, MatrixXd A_ls){

    double A_ls1_1 = A_ls(0,0);
    double A_ls1_2 = A_ls(1,0);
    double A_ls1_3 = A_ls(2,0);
    double A_ls2_1 = A_ls(0,1);
    double A_ls2_2 = A_ls(1,1);
    double A_ls2_3 = A_ls(2,1);
    double A_ls3_1 = A_ls(0,2);
    double A_ls3_2 = A_ls(1,2);
    double A_ls3_3 = A_ls(2,2);
    double rbpb1 = rx;
    double rbpb2 = xc_star(3);
    double xo3 = xc_star(2);
    double t2 = d+rbpb2;
    double t3 = cos(xo3);
    double t4 = sin(xo3);
    double t5 = A_ls2_1-A_ls2_3*t2;
    double t6 = A_ls1_1-A_ls1_3*t2;
    double t7 = A_ls2_3*rbpb1;
    double t8 = A_ls2_2+t7;
    double t9 = A_ls1_3*rbpb1;
    double t10 = A_ls1_2+t9;

    MatrixXd B_tmp(uc_star.size(), xc_star.size());
    MatrixXd B(xc_star.size(),uc_star.size());

    B_tmp << t3*t6-t4*t5,t3*t5+t4*t6,A_ls3_1-A_ls3_3*t2,0.0,-t4*t8+t3*t10,t3*t8+t4*t10,A_ls3_2+A_ls3_3*rbpb1,0.0,0.0,0.0,0.0,1.0;
    B = B_tmp.transpose();

    return B;
}

