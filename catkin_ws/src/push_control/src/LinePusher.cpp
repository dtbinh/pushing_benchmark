//Common libraries
#include <iostream>
#include <Eigen/Dense>
#include <unistd.h>
#include <fstream>

//Custom Libraries
#include "PusherSlider.h"
#include "Helper.h"
#include "LinePusher.h"
#include "StructuresMain.h"
#include "Friction.h"

using namespace std;
using namespace Eigen;

LinePusher::LinePusher(PusherSlider* _pusher_slider, Friction* _friction, string trajectory_name): Pusher()
{
lp=0.03;
d = lp/2;
numxcStates = 4;
numucStates = 5;
numxsStates = 6;
numusStates = 3;
pusher_type = "line";
num_contact_points = 2;
ppusher_slider = _pusher_slider;
pfriction = _friction;

//Static Eigen Constructor
uc_eq = buildUcEq();
xc_eq = buildXcEq();;
struct outStarStruct outStar;
//outStar = buildNominalTrajectory(0.001);
//outStar = buildStraightLineTrajectory("0_05");
//outStar = build8TrackTrajectory("0_05");
    outStar = buildTrajectory("/Data/" + trajectory_name +".json");
xc_star = outStar.xcStar;
uc_star = outStar.ucStar;
xs_star = outStar.xsStar;
us_star = outStar.usStar;
t_star = outStar.tStar;

struct outStarStruct outStar2;
//outStar2 = buildNominalTrajectory(0.03);
outStar2 = build8TrackTrajectory("0_05");
xc_star2 = outStar2.xcStar;
uc_star2 = outStar2.ucStar;
xs_star2 = outStar2.xsStar;
us_star2 = outStar2.usStar;
t_star2 = outStar2.tStar;

}

VectorXd LinePusher::buildUcEq()
{
    VectorXd tmp(5);
    tmp << .1634,.1634,0,0,0;
    return tmp;
}

VectorXd LinePusher::buildXcEq()
{
    VectorXd tmp(4);
    tmp << 0,0,0,0;
    return tmp;
}

outStarStruct LinePusher::buildTrajectory(string file_name){

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

outStarStruct LinePusher::buildNominalTrajectory(double h_star){
    double v_eq = 0.05;
    double t0 = 0;
    double tf = 7;
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

outStarStruct LinePusher::buildStraightLineTrajectory(string v_eq){

    Json::Value root;
    Json::Reader reader;
    char const* tmp = getenv( "PUSHING_BENCHMARK_BASE" );
    string envStr( tmp );
    string fileName;
    fileName = envStr + "/Simulation/Data/Straight_line_pusher_vel_" + v_eq + ".json";
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

outStarStruct LinePusher::build8TrackTrajectory(string v_eq){

    Json::Value root;
    Json::Reader reader;
    char const* tmp = getenv( "PUSHING_BENCHMARK_BASE" );
    string envStr( tmp );
    string fileName;
    fileName = envStr + "/Simulation/Data/8Track_line_pusher_vel_" + v_eq + ".json";
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

outStateNominal LinePusher::getStateNominal(double t) {
//            % Get nominal trajectory values at time T
//            %
//            %   Parameters:
//            %   t       -   Time at which the nominal state is evaluated


    VectorXd vecDif(t_star.rows());
    MatrixXf::Index   minIndex;
    double minVal;
    outStateNominal out_state_nominal;

    double t0 = Helper::gettime();
    vecDif = (t*VectorXd::Ones(t_star.rows()) - t_star).cwiseAbs();
    double tf = Helper::gettime();
//    cout<<"time diff: "<<tf-t0<<endl;
//    cout<<"t_star.rows(): "<<t_star.rows()<<endl;

    t0 = Helper::gettime();
    minVal = vecDif.minCoeff(&minIndex);
    tf = Helper::gettime();
//    cout<<"time min: "<<tf-t0<<endl;

    out_state_nominal.xcStar = xc_star.row(minIndex);
    out_state_nominal.ucStar = uc_star.row(minIndex);
    out_state_nominal.xsStar = xs_star.row(minIndex);
    out_state_nominal.usStar = us_star.row(minIndex);

    return out_state_nominal;
}
outStateNominal LinePusher::getStateNominal2(double t) {
//            % Get nominal trajectory values at time T
//            %
//            %   Parameters:
//            %   t       -   Time at which the nominal state is evaluated
//    double t0 = Helper::gettime();
    VectorXd vecDif(t_star2.rows());
    MatrixXf::Index   minIndex;
    double minVal;
    outStateNominal out_state_nominal;

    vecDif = (t*VectorXd::Ones(t_star2.rows()) - t_star2).cwiseAbs();
    minVal = vecDif.minCoeff(&minIndex);

    out_state_nominal.xcStar = xc_star2.row(minIndex);
    out_state_nominal.ucStar = uc_star2.row(minIndex);
    out_state_nominal.xsStar = xs_star2.row(minIndex);
    out_state_nominal.usStar = us_star2.row(minIndex);

    return out_state_nominal;
}

VectorXd LinePusher::getError(VectorXd xc, double t) {
    VectorXd delta_xc(xc.rows());
    outStateNominal out_state_nominal = getStateNominal(t);

    delta_xc = xc - out_state_nominal.xcStar;
    return delta_xc;
}

VectorXd LinePusher::coordinateTransformSC(VectorXd xs){
    //Declare matrices
    Matrix2d Cbi;
    Vector2d rbbi;
    Vector2d rbpi;
    Vector2d rbpb;
    VectorXd xc(4);
    double ry;
    //~ cout<<" xs "<<xs<<endl;
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

VectorXd LinePusher::coordinateTransformCS(VectorXd xc)
{
    //Declare matrices
    Matrix2d Cbi;
    Vector2d ripi;
    Vector2d ripb;
    Vector2d rbbi;
    Vector2d rbpb;
    VectorXd xs(6);
    struct xcStateStruct xcState(xc);
    //~ %Direction Cosine Matrices 
    Cbi = Helper::C3_2d(xcState.theta);
    //~ %Convert state to body frame
    rbbi = Cbi*xcState.ribi;
    rbpb << -ppusher_slider->a/2, xcState.ry;
    ripb =Cbi.transpose()*rbpb;
    ripi =xcState.ribi+ripb;
    //~ %Output
    xs << xcState.ribi, xcState.theta, ripi, xcState.theta;
    return xs;
}

VectorXd LinePusher::force2Velocity(VectorXd xc, VectorXd uc){
    //Declare matrices
    struct xcStateStruct xcState(xc);
    struct ucStateStruct ucState(uc);
    Matrix2d Cbi;
    Vector2d drbpb;
    Vector2d rbap;
    Vector2d rbcp;    
    Vector2d rbab;
    Vector2d rbcb;
    Vector2d rbbi;
    Vector2d rbpb;
    Vector2d ripb;
    Vector2d npa_b;
    Vector2d tpa_b;
    Vector2d Tpa_b;
    MatrixXd Jpa(2,3);
    Vector2d npc_b;
    Vector2d tpc_b;
    Vector2d Tpc_b;
    MatrixXd Jpc(2,3);
    MatrixXd Na(1,3);
    MatrixXd Nc(1,3);
    MatrixXd N_tot(2,3);
    MatrixXd Ta(1,3);
    MatrixXd Tc(1,3);
    MatrixXd T_tot(2,3);
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
    //~ #%Find rx, ry: In body frame   
    rbap << 0,d;
    rbcp << 0,-d;    
    rbab = rbpb + rbap;
    rbcb = rbpb + rbcp;
    //~ %kinematics
    npa_b << 1,0;
    tpa_b << 0,1;
    Tpa_b = tpa_b;
    Jpa << 1, 0, -rbab(1), 0, 1, rbab(0);
    //~ %kinematics
    npc_b << 1,0;
    tpc_b << 0,1;
    Tpc_b = tpc_b;
    Jpc << 1, 0, -rbcb(1), 0, 1, rbcb(0);
    //build matrices point a
    Na = npa_b.transpose()*Jpa;
    Ta = tpa_b.transpose()*Jpa;
    //build matrices point c
    Nc = npc_b.transpose()*Jpc;
    Tc = tpc_b.transpose()*Jpc;
    //Build total matrices
    N_tot << Na,Nc;
    T_tot << Ta,Tc;
    //Compute twist
    twist_body_b = pfriction->A_ls*(N_tot.transpose()*ucState.fn + T_tot.transpose()*ucState.ft);
    Sib = Helper::buildSib(xcState.theta);//[Cbi' [0;0];0 0 1];
    twist_body_i = Sib*twist_body_b;
    //~ %Compute twist of pusher
    twist_pusher_i.segment(0,2) = twist_body_i.segment(0,2) + Cbi.transpose()*drbpb + Helper::cross3d(twist_body_i(2), ripb);
    twist_pusher_i(2) = twist_body_i(2);

    return twist_pusher_i;
}

MatrixXd LinePusher::A_fun(VectorXd xc_star, VectorXd uc_star, double rx, double d, MatrixXd A_ls){
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
  double uc3 = uc_star(2);
  double uc4 = uc_star(3);
  double xo3 = xc_star(2);
  double t2 = d+rbpb2;
  double t3 = rbpb1*uc3;
  double  t4 = rbpb1*uc4;
  double t5 = d-rbpb2;
  double t6 = t5*uc2;
  double t12 = t2*uc1;
  double t7 = t3+t4+t6-t12;
  double t8 = uc1+uc2;
  double t9 = uc3+uc4;
  double t10 = cos(xo3);
  double t11 = sin(xo3);
  double t13 = A_ls1_3*t7;
  double t14 = A_ls1_1*t8;
  double t15 = A_ls1_2*t9;
  double  t16 = t13+t14+t15;
  double t17 = A_ls2_3*t7;
  double t18 = A_ls2_1*t8;
  double t19 = A_ls2_2*t9;
  double t20 = t17+t18+t19;


  MatrixXd A_tmp(xc_star.size(),xc_star.size());
  MatrixXd A(xc_star.size(),xc_star.size());
  A_tmp << 0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,-t11*t16-t10*t20,t10*t16-t11*t20,0.0,0.0,-A_ls1_3*t8*t10+A_ls2_3*t8*t11,-A_ls1_3*t8*t11-A_ls2_3*t8*t10,-A_ls3_3*t8,0.0;
  A = A_tmp.transpose();

  return A;
}

MatrixXd LinePusher::B_fun(VectorXd xc_star, VectorXd uc_star, double rx, double d, MatrixXd A_ls){

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

    double t5 = d-rbpb2;
    double t6 = A_ls1_3*rbpb1;
    double t7 = A_ls1_2+t6;
    double t8 = t3*t7;
    double t9 = A_ls2_3*rbpb1;
    double t10 = A_ls2_2+t9;
    double t11 = t8-t4*t10;
    double t12 = A_ls2_1-A_ls2_3*t2;
    double t13 = A_ls1_1-A_ls1_3*t2;
    double t14 = A_ls2_3*t5;
    double t15 = A_ls2_1+t14;
    double t16 = A_ls1_3*t5;
    double t17 = A_ls1_1+t16;
    double t18 = t3*t10;
    double t19 = t4*t7;
    double t20 = t18+t19;
    double t21 = A_ls3_3*rbpb1;
    double t22 = A_ls3_2+t21;

    MatrixXd B_tmp(uc_star.size(), xc_star.size());
    MatrixXd B(xc_star.size(),uc_star.size());

    B_tmp <<t3*t13-t4*t12,t3*t12+t4*t13,A_ls3_1-A_ls3_3*t2,0.0,-t4*t15+t3*t17,t3*t15+t4*t17,A_ls3_1+A_ls3_3*t5,0.0,t11,t20,t22,0.0,t11,t20,t22,0.0,0.0,0.0,0.0,1.0;
    B = B_tmp.transpose();

    return B;
}

