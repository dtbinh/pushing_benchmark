//
// Created by robot on 1/7/18.
//

#ifndef PUSH_CONTROL_IK_FAST_INTERFACE_PYTHON_H
#define PUSH_CONTROL_IK_FAST_INTERFACE_PYTHON_H


//Methods
bool in_joint_range(double q[]);
double cost(double q[], double q0[], double weight[]);
void _copy(double x[], double y[], const int size = 6);
int select_best_js(double best_q[], double candidates[], int ncandidate, double q0[], double weight[]);
float SIGN(float x);
float NORM(float a, float b, float c, float d);
void ikfastPython(double* solutionList, int* nsol, double* pose);
void ikfastAndFindBest(double* solution, double* pose, double* weight, double* q0, int* hassol);
void fkfastPython(double* pose, double* joints);


#endif //PUSH_CONTROL_IK_FAST_INTERFACE_PYTHON_H




