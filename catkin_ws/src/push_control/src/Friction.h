/*
 */
 #include "Helper.h"
 #include "PusherSlider.h"


#ifndef FRICTION_H
#define FRICTION_H

class Friction
{
    public:
        double nu;
        double nu_p;
        
        double c;
        double f_max;
        double m_max;
        MatrixXd A_ls;
        PusherSlider* ppusher_slider;
private:

public:
    Friction(PusherSlider* _pusher_slider);
    double setmMax();
    MatrixXd setAls();
};
#endif



