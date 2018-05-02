#include <iostream>
#include <stdio.h>
#include "Friction.h"
#include "PusherSlider.h"
#include "Helper.h"
#include <Eigen/Dense>
 
using namespace std;
using namespace Eigen;

Friction::Friction(PusherSlider* _pusher_slider)
{
    //Static Constructor
    ppusher_slider = _pusher_slider;
    nu = 0.35;
    nu_p = 1.;
    f_max = nu*ppusher_slider->m*Helper::g;
    m_max =setmMax();
    c = m_max/f_max;
    A_ls = setAls();
}	

double Friction::setmMax()
{
    return nu*Helper::doubleGaussQuad(-ppusher_slider->a/2,ppusher_slider->a/2, -ppusher_slider->b/2,ppusher_slider->b/2);
}

MatrixXd Friction::setAls()
{
    MatrixXd Als(3,3);
    double d12 = 2/pow((f_max),2);
    double d3  = 2/pow((m_max),2);;
    Als << d12,0,0,0,d12,0,0,0,d3;
    return Als;
}


