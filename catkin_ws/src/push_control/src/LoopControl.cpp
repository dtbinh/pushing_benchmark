//~ //System
#include <iostream>
#include <stdio.h>
#include <cmath>
//Externals
#include <Eigen/Dense>
#include "gurobi_c++.h"
#include "json/json.h"
//Custom
#include "LoopControl.h"
#include "Pusher.h"
#include "PointPusher.h"
#include "FOM.h"
#include "LModes.h"
#include "GPDataController.h"
#include "HybridController.h"

//ROS
#include <ros/ros.h>

using namespace std;
using Eigen::MatrixXd;

// ****************************8
void initializeThread(void *thread_arg){
  //Mutex
  pthread_mutex_init(&nonBlockMutex, NULL);
  pthread_t rriThread;
  pthread_attr_t attrR;
  pthread_attr_init(&attrR);
  pthread_attr_setdetachstate(&attrR, PTHREAD_CREATE_JOINABLE);
  pthread_create(&rriThread, &attrR, loopControl, thread_arg);
}

//********************************************************************
// Optimization Thread
//********************************************************************
void *loopControl(void *thread_arg)
{
    //***********Variable initialization************
    struct thread_data *my_data;
    my_data = (struct thread_data *) thread_arg;
    pthread_mutex_lock(&nonBlockMutex);
    //-------Protected---------------------
    Vector3d *pq_slider     = my_data->q_slider;
    Vector3d *pq_pusher     = my_data->q_pusher;
    Vector3d *ptwist_pusher    = my_data->twist_pusher;
    double *ptime    = my_data->time;
    VectorXd *pxc    = my_data->xc;
    VectorXd *pxs    = my_data->xs;
    VectorXd *puc    = my_data->uc;
    VectorXd *pus    = my_data->us;
    Pusher * ppusher    = my_data->ppusher;

    Vector3d &q_slider     = *pq_slider;
    Vector3d &q_pusher     = *pq_pusher;
    Vector3d &twist_pusher    = *ptwist_pusher;
    VectorXd &xc_thread    = *pxc;
    VectorXd &xs_thread    = *pxs;
    VectorXd &uc_thread    = *puc;
    VectorXd &us_thread    = *pus;
    double &time = *ptime;
    //--------------------------------------
    pthread_mutex_unlock(&nonBlockMutex);

    //Build objects
    outMatrices out_matrices;
    PusherSlider pusher_slider;
    Friction friction(&pusher_slider);

    MatrixXd Q = MatrixXd::Zero(ppusher->numxcStates, ppusher->numxcStates);
    MatrixXd Qf = MatrixXd::Zero(ppusher->numxcStates, ppusher->numxcStates);
    MatrixXd R = MatrixXd::Zero(ppusher->numucStates, ppusher->numucStates);

    double _time;
    VectorXd delta_xc(ppusher->numxcStates);
    Vector3d _q_slider; //pose of object
    Vector3d _q_pusher; //pose of pusher
    Vector3d _twist_pusher; //commanded twist of pusher [dx,dy,dtheta]
    VectorXd xs(ppusher->numxsStates);
    VectorXd xc(ppusher->numxcStates);
    VectorXd us(ppusher->numusStates);
    VectorXd uc(ppusher->numucStates);
    VectorXd delta_uc(ppusher->numucStates);
    VectorXd _q_offset_pusher(_q_pusher.rows());
    VectorXd _q_offset_slider(_q_slider.rows());
    VectorXd _q_slider_zeroed(_q_slider.rows());
    VectorXd _q_pusher_zeroed(_q_pusher.rows());

    /* ************ TO EDIT ************** */
    //FOM control parameters
//    Q.diagonal() << 3,3,.1,0.0;Q=Q*10;
//    Qf.diagonal() << 3,3,.1,0.0;Qf=Qf*2000;
//    R.diagonal() << 1,1,0.01;R = R*.01;
    //LMODES control parameters
//    Q.diagonal() << 3,3,.1,0.0;Q=Q*10;
//    Qf.diagonal() << 3,3,.1,0.0;Qf=Qf*2000;
//    R.diagonal() << 1,1,0.01;R = R*.5;
    //GPDataController control parameters
    Q.diagonal() << 1,1,.01,10;Q=Q*1;
    Qf.diagonal() << 1,1,.1,10;Qf=Qf*1000;
    R.diagonal() << 1,.1;R = R*10;
//    FOM mpc(3, &pusher_slider, ppusher, &friction, Q, Qf, R);
//    LMODES mpc(&pusher_slider, ppusher, &friction, Q, Qf, R);
    GPDataController mpc(&pusher_slider, ppusher, &friction, Q, Qf, R);

//    HybridController mpc(3, &pusher_slider, ppusher, &friction, Q, Qf, R);


  //8Track
    _q_offset_slider << 0.3484033942222595, 0, 0; //point pusher
    _q_offset_pusher << 0.3484033942222595, 0, 0.0;//point pusher
    /* ************ TO EDIT ************** */

    //Straight Line
//    _q_offset_slider << 0.19867394381957065, 0, 0; //point pusher
//    _q_offset_pusher << 0.19867394381957065, 0, 0.0;//point pusher

    //**********************************************************************
    //************************ Begin Loop **********************************
    //**********************************************************************
    int counter = 0;
    ros::Rate r(125);
    while(ros::ok()) {
        cout<<counter<<endl;

        double t0 = Helper::gettime();
        bool is_exit;
        ros::NodeHandle n1;
        n1.getParam("is_exit", is_exit);

        if (is_exit==true){
          break;
        }

//        ---------------Protected----------------
        pthread_mutex_lock(&nonBlockMutex);
        _q_slider = q_slider;
        _q_pusher= q_pusher;
        _time = time;
        xc_thread = xc;
        xs_thread = xs;
        uc_thread = uc;
        us_thread = us;
        pthread_mutex_unlock(&nonBlockMutex);
//        //--------------------------------------
        //define state variables from vicon and pusher states
        outStateNominal out_state_nominal;
        out_state_nominal = ppusher->getStateNominalGPData(_time);

        _q_slider_zeroed = _q_slider - _q_offset_slider;
        _q_pusher_zeroed = _q_pusher - _q_offset_pusher;
        _q_slider_zeroed(2) = Helper::find_best_angle(_q_slider_zeroed(2), out_state_nominal.xcStar(2));
        xs << _q_slider_zeroed, _q_pusher_zeroed;
//        xs(2) = (xs(5)+xs(2))/2.0;

        xc =  ppusher->coordinateTransformSC(xs);
        //Compute MPC control input
        uc = mpc.solveMPC(xc, _time);
//


//      outStateNominal out_state_nominal;
//      out_state_nominal = ppusher->getStateNominal(_time);
//      us = ppusher->force2Velocity(out_state_nominal.xcStar, out_state_nominal.ucStar);

//        us = ppusher->force2Velocity(xc, uc);

        us = mpc.get_robot_velocity(xc, uc);

//      //-------Protected---------------------
        pthread_mutex_lock(&nonBlockMutex);
        twist_pusher = us;
        pthread_mutex_unlock(&nonBlockMutex);
//        --------------------------------------

        ++counter;
        r.sleep();
      double tf = Helper::gettime();
      cout<<"freq: "<<tf-t0<<endl;
      cout<<" "<<endl;
        }
    //*********** End Loop **************************************************
    pthread_exit((void*) 0);
}
