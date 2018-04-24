/*
 */
#include <Eigen/Dense>
using namespace std;
using namespace Eigen;
//~ using namespace 

#ifndef PUSHERSLIDER_H
#define PUSHERSLIDER_H

class PusherSlider 
{
    public:
        double a;
        double b;
        double rho;
        double height;

        double A;
        double V;
        double m;

        PusherSlider();
        double geometry(Vector2d P);
} ; 

#endif

