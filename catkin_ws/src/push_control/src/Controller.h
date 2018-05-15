//
// Created by mcube on 5/15/18.
//

#include "StructuresLinePusher.h"
#include "Friction.h"
#include <Eigen/Dense>

using namespace std;
using namespace Eigen;

#ifndef PUSH_CONTROL_CONTROLLER_H
#define PUSH_CONTROL_CONTROLLER_H

class Controller {
public:
//  //Properties
//  //Static Methods
  virtual VectorXd solveMPC(VectorXd xc, double _time) = 0;
  virtual VectorXd get_robot_velocity(VectorXd xc, VectorXd uc) = 0;

};

#endif //PUSH_CONTROL_CONTROLLER_H

