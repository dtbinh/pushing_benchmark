/*
 */
#include "StructuresMain.h"
#include "StructuresOptProgram.h"
#include "PusherSlider.h"
#include "Pusher.h"
#include "OptProgram.h"
#include "Friction.h"
#include <Eigen/Dense>

using namespace std;
using namespace Eigen;

#ifndef MPC_H
#define MPC_H

typedef Eigen::Triplet<double> T;

struct outMatricesMPC{
//    Eigen::SparseMatrix<double,Eigen::RowMajor> Aeq;
    vector<T> Q_vec;
    vector<T> Aeq_vec;
    vector<T> Ain_vec;
    double beq_vec[];
    double bin_vec[];
    double c_vec[];
    MatrixXd Aeq;
    VectorXd beq;
    int row_start_eq;
//    Eigen::SparseMatrix<double,Eigen::RowMajor> Ain;
    MatrixXd Ain;
    VectorXd bin;
    int row_start_ineq;
//    Eigen::SparseMatrix<double,Eigen::RowMajor> H;
    MatrixXd H;
    VectorXd lb;
    VectorXd ub;
};
struct outBuildMotionConstraints{
    Eigen::MatrixXd A_bar;
    Eigen::MatrixXd B_bar;
};

struct outAddMotionConstraints{
    Eigen::MatrixXd Aeq;
    Eigen::MatrixXd beq;
};
struct outAddForceDepConstraints{
    Eigen::MatrixXd Aeq;
    Eigen::MatrixXd beq;
    Eigen::MatrixXd Ain;
    Eigen::MatrixXd bin;
};

struct outBuildForceIndConstraints{
    Eigen::MatrixXd Ain;
    Eigen::VectorXd bin;
};

struct outBuildForceDepConstraints{
    Eigen::MatrixXd Ain;
    Eigen::VectorXd bin;
    Eigen::MatrixXd Aeq;
    Eigen::VectorXd beq;
    int mode;
};


class MPC {
public:

    double h;
    int steps;
    int num_variables;
    int num_constraints;
    int num_xc_states;
    int num_uc_states;
    outMatricesMPC matricesMPC;
//    int num_families;
    OptProgram *opt_program;
    outSolutionStruct out_solution;
    outMatrices out_matrices;
    PusherSlider *pusher_slider;
    Pusher *line_pusher;
    Friction *friction;

    //Static Methods
    MPC(outMatrices _out_matrices, PusherSlider *pusher_slider, Pusher *line_pusher, Friction *friction);
    void buildProgram();
    int getControlIndex(int lv1);
    int getStateIndex(int lv1);
    void updateIC(VectorXd delta_xc);
    void addVelocityConstraints(VectorXd xn, VectorXd un, VectorXd xc);
    void removeVelocityConstraints(int num_constraints);
    outSolutionStruct solveMPC();
    void initializeMatricesMPC();
    void buildConstraintMatrices(double time, VectorXd mode_scedule, VectorXd delta_xc);
    void buildWeightMatrices();
    void addMotionConstraints(VectorXd& xc_star,VectorXd& uc_star, int lv1);
    void addICConstraints(VectorXd& xc_star,VectorXd& uc_star, int lv1, VectorXd delta_xc);
    void addVelConstraints(VectorXd& xc_star, VectorXd& uc_star, int lv1, VectorXd delta_xc);
    void addForceDepConstraints(VectorXd& xc_star,VectorXd& uc_star, int mode, int lv1);
    void addForceIndConstraints(VectorXd& xc_star,VectorXd& uc_star, int lv1);
    outBuildForceIndConstraints buildForceIndConstraints(VectorXd& xc_star,VectorXd& uc_star);
    outBuildForceDepConstraints buildForceDepConstraints(VectorXd& xc_star,VectorXd& uc_star, int mode);
    outBuildMotionConstraints buildMotionConstraints(VectorXd& xc_star, VectorXd& uc_star, double rx, double d, MatrixXd A_ls);

};
#endif
