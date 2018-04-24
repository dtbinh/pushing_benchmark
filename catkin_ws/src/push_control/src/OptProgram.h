/*
 */
//~ //System
//Externals
#include <Eigen/Dense>
#include "gurobi_c++.h"
//~ Custom Classes
#include "PusherSlider.h"
#include "Friction.h"
#include "Helper.h"
#include "LinePusher.h"
#include "StructuresOptProgram.h"

using namespace std;
using namespace Eigen;

#ifndef OPTPROGRAM_H
#define OPTPROGRAM_H

class OptProgram {
    public:
        //Doubles
        //~ double h;
        //~ int steps;
        int num_variables;
        int num_ineq_constraints;
        int num_eq_constraints;
        int num_constraints;
        //Matrices
        Eigen::MatrixXd Q;
        Eigen::MatrixXd Aeq;
        Eigen::MatrixXd Ain;
        Eigen::VectorXd fobj;
        Eigen::VectorXd beq;
        Eigen::VectorXd bin;
        Eigen::VectorXd lb;
        Eigen::VectorXd ub;
		double* lbd;
		double* ubd;
        //~ Eigen::VectorXd solutionU;
        Eigen::VectorXd solution;
        //Character arrays
        char* vtype;
        //~ double solution;
        double objval;
        //Boolean
        bool success;
        //Gurobi parameters
        GRBEnv env;
        GRBModel model;
        GRBVar* vars;
        GRBLinExpr lhs;
        GRBConstr* constr_ineq;
        GRBConstr* constr_eq;
        GRBConstr* constr_vel;

private:

public:
    OptProgram (outMatrices _out_matrices);
    void getDimensions();
    void setVariableType();
    void addVariables();
    void addCost();
    void addIneqConstraints();
    void addEqConstraints();
    void addEqConstraintsGur(MatrixXd Aeq_tmp, VectorXd beq_tmp);
    void updateIC(MatrixXd AeqIC, VectorXd beqIC);
    void removeConstraints();
    outSolutionStruct optimizeModel();
    void  editConstraintsIC(VectorXd b_tmp, int num_x_variables);
    void  addVelocityConstraints(MatrixXd A_in, VectorXd b_in);
    void removeVelocityConstraints(int num_constraints);

    
};
#endif


