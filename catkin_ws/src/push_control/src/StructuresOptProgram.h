/*
 */
 
#include <Eigen/Dense>
//~ using namespace std;
using Eigen::MatrixXd;
//~ using Eigen::ArrayXf;

 
#ifndef STRUCTUREOPTPROGRAM_H
#define STRUCTUREOPTPROGRAM_H

//**************
struct outSolutionStruct{
    Eigen::VectorXd solution;
    double objVal;
    std::string status;
};
//**************
struct outMatrices{
    Eigen::MatrixXd Q;
    Eigen::MatrixXd Aeq;
    Eigen::MatrixXd Ain;
    Eigen::MatrixXd A_bar;
    Eigen::VectorXd fobj;
    Eigen::VectorXd beq;
    Eigen::VectorXd bin;
    Eigen::VectorXd lb;
    Eigen::VectorXd ub;
};
//**************



#endif
