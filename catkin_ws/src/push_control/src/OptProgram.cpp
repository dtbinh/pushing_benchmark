//~ //System
#include <iostream>
#include <stdio.h>
#include <cmath>
#include <string> 
#include <fstream>
#include <typeinfo>
//Externals
#include <Eigen/Dense>
#include <Eigen/Sparse>
#include "gurobi_c++.h"
#include "json/json.h"
//~ Custom Classes
#include "PusherSlider.h"
#include "Friction.h"
#include "Helper.h"
#include "LinePusher.h"
#include "OptProgram.h"
#include "StructuresOptProgram.h"

using namespace std;
using namespace Eigen;

OptProgram::OptProgram (outMatrices _out_matrices):
 env(), model(env), lhs(0)
{
	//~ Initialize
    Q = _out_matrices.Q;
    fobj = _out_matrices.fobj;
    Ain= _out_matrices.Ain;
    bin= _out_matrices.bin;
    Aeq= _out_matrices.Aeq;
    beq= _out_matrices.beq;
    lb= _out_matrices.lb;
    ub= _out_matrices.ub;
    ////Initialization of variables
    getDimensions();
    setVariableType();
    addVariables();
    addCost();
    addIneqConstraints();
    addEqConstraints();
}

////*******************************************************************************************
void OptProgram::getDimensions()
{
    //Get relevant constants
    num_eq_constraints = beq.rows();
    num_ineq_constraints = bin.rows();
	num_variables = Q.rows();
    num_constraints = num_ineq_constraints + num_eq_constraints;
    //Resize matrices
    solution.resize(num_variables);
    //Dynamic arrays
    vtype = new char[num_variables];
//    constr = new GRBConstr[num_constraints];
    constr_ineq = new GRBConstr[num_ineq_constraints];
    constr_eq = new GRBConstr[num_eq_constraints];
    constr_vel = new GRBConstr[5];
    //Frank hack
    //~ constrIC = new GRBConstr[num_x_variables];
}
////*******************************************************************************************
void OptProgram::setVariableType()
{
	for (int i=0;i<num_variables;i++){
		vtype[i] = 'C';
    }
}
////*******************************************************************************************
void OptProgram::addVariables()
{
    //~ Convert matrices to double arrays 
    double lbd[lb.rows()];
    double ubd[ub.rows()];
    Helper::vector_to_array(lb.rows(), 1, lbd, lb);
    Helper::vector_to_array(ub.rows(), 1, ubd, ub);
    //Define model variables
    vars = model.addVars(lbd,ubd,NULL,vtype,NULL,num_variables);
    model.update();
}
//*******************************************************************************************
void OptProgram::addCost()
{
    //Add linear cost
    //            cout<< "i "<<i<<endl;
//            cout<< "j "<<j<<endl;
    GRBQuadExpr obj = 0;
    for (int j = 0; j < num_variables; j++){
        obj += fobj(j)*vars[j]; 
	}
    //Add quadratic cost 
    for (int i = 0; i < num_variables; i++){
        for (int j = 0; j < num_variables; j++){
            if (Q(i,j) != 0){
                obj += Q(i,j)*vars[i]*vars[j];
			}
		}
	}
    model.setObjective(obj);
    model.update();
}
//*******************************************************************************************
void OptProgram::addIneqConstraints()
{

    for (int i = 0; i < num_ineq_constraints; i++) {
		lhs = 0;
        for (int j = 0; j < num_variables; j++){
            if (Ain(i,j) != 0){
                lhs += Ain(i,j)*vars[j];
			}
		}
        constr_ineq[i] = model.addConstr(lhs, '<', bin(i));
    }
    model.update();
}
////*******************************************************************************************
void OptProgram::addEqConstraints()
{

    for (int i = 0; i < num_eq_constraints; i++) {
		lhs = 0;
        for (int j = 0; j < num_variables; j++){
            if (Aeq(i,j) != 0){
                lhs += Aeq(i,j)*vars[j];
			}
		}
        constr_eq[i] = model.addConstr(lhs, '=', beq(i));
    }
    model.update();
}
////*******************************************************************************************
void OptProgram::addEqConstraintsGur(MatrixXd Aeq_tmp, VectorXd beq_tmp)
{

//  lhs = 1*vars[0];
//  lhs_vec[0] = 1*vars[0];
//  cout<<"lhs: "<<lhs<<endl;
//  cout<<"lhs_vec[0]: "<<lhs_vec[0]<<endl;

  GRBLinExpr lhs_vec[Aeq_tmp.rows()];
  Eigen::SparseMatrix<double,Eigen::RowMajor> Test(Aeq_tmp.rows(), Aeq_tmp.cols());
  Test = Aeq_tmp.sparseView();

  double t0 = Helper::gettime();

  for (int k = 0; k < Test.outerSize(); ++k){
    for (SparseMatrix<double,Eigen::RowMajor>::InnerIterator it(Test, k); it; ++it){
      lhs += Aeq_tmp(it.row(),it.col())*vars[it.col()];
    }
    model.addConstr(lhs, '=', beq_tmp(k));
  }

  double tf = Helper::gettime();
  cout<<"constraint time: "<<tf-t0<<endl;
//    for (int i = 0; i < Aeq_tmp.rows(); i++) {
//		lhs = 0;
//        for (int j = 0; j < Aeq_tmp.cols(); j++){
//            if (Aeq(i,j) != 0){
//                lhs += Aeq_tmp(i,j)*vars[j];
//			}
//		}

//        cout<< "lhs"<<lhs<<endl;
//        cout<< "lhs"<<beq_tmp(i)<<endl;
//    }
//    model.addConstrs(lhs_vec, '=', beq_tmp(i));
    model.update();
}
////*******************************************************************************************
outSolutionStruct OptProgram::optimizeModel()
{   
	double counter1 = 0;
    string status;
    outSolutionStruct out_solution_struct;
    
    model.getEnv().set(GRB_IntParam_OutputFlag,0);
    model.optimize();

	int optimstatus = model.get(GRB_IntAttr_Status);{
        status = "Optimization complete";  
        }
	if (optimstatus == GRB_OPTIMAL) {
        status= "Optimal objective: ";
        objval = model.get(GRB_DoubleAttr_ObjVal);
        for (int i = 0; i < num_variables; i++){
            solution(i) = vars[i].get(GRB_DoubleAttr_X);
            success = true;
        }
        out_solution_struct.objVal = objval;
        out_solution_struct.solution = solution;
    } 
    else if (optimstatus == GRB_INF_OR_UNBD) {
       status= "Model is infeasible or unbounded" ;
    } 
    else if (optimstatus == GRB_INFEASIBLE) {
       status="Model is infeasible" ;
    } 
    else if (optimstatus == GRB_UNBOUNDED) {
       status= "Model is unbounded" ;
    } 
    else {
       status= "Optimization was stopped with status = ???";
    }
    
	out_solution_struct.status = status;
//    cout<<"[OptProgram] status "<<status<<endl;
	return out_solution_struct;
}
////*******************************************************************************************
void OptProgram::editConstraintsIC(VectorXd b_tmp, int num_x_variables)
{
    for (int i=0;i<num_x_variables;i++){
        constr_eq[i].set(GRB_DoubleAttr_RHS, b_tmp(i));
    }
}
////*******************************************************************************************
void OptProgram::addVelocityConstraints(MatrixXd A_in_small, VectorXd b_in_small)
{

    for (int i = 0; i < b_in_small.size(); i++) {
        lhs = 0;
        for (int j = 0; j < A_in_small.cols(); j++){
            if (A_in_small(i,j) != 0){
                lhs += A_in_small(i,j)*vars[j];
            }
        }
        constr_vel[i] = model.addConstr(lhs, '<', b_in_small(i));
    }
    model.update();

}
////*******************************************************************************************
void OptProgram::removeVelocityConstraints(int num_constraints)
{
//    Remove IC constraints
    for (int i=0;i<num_constraints;i++){
        model.remove(constr_vel[i]);}
}
