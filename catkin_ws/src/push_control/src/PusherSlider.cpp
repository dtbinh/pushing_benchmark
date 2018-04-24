#include <iostream>
#include <stdio.h>
#include <math.h>
//External libraries
#include <Eigen/Dense>
#include "json/json.h"
//Custom classes
#include "Helper.h"
#include "Friction.h"
#include "PusherSlider.h"


using namespace std;
using namespace Eigen;

//Static Constructor

PusherSlider::PusherSlider()
{
//~ Initialize constants
a=0.09;
b=0.09;
rho=10000;
height=0.013;

A = a*b;
V = A*height;
m = rho*V;

}

double PusherSlider::geometry(Vector2d P)
{
    double fun = (m*Helper::g)/A * P.norm();
    return fun;
}
