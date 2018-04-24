/*
 */
#include <Eigen/Dense>
#include <Eigen/Sparse>

#include "PusherSlider.h"
#include "json/json.h"

//#include "StructuresMain.h"


using namespace std;
using namespace Eigen;

#ifndef HELPER_H
#define HELPER_H

typedef double (PusherSlider::* vFunctionCall)(Vector2d args);
typedef Eigen::Triplet<double> T;

class Helper {
    public:
        //Properties
        static double g;

        //Methods
        Helper();
        static double doubleGaussQuad(double ag, double bg, double cg, double dg);
        static double cross2d(Vector2d r, Vector2d v);
        static Vector2d cross3d(double r, Vector2d x);
        static MatrixXd C3_2d(double theta);
        static Matrix3d buildSib(double theta);

        static MatrixXd cross_op(MatrixXd w);
        static double gettime();
        static double smooth(double data, float filterVal, double smoothedVal);
        static void write_file(FILE* myFile, int num_rows, int num_cols, double *A);
        //~ void write_matrix(Json::Value root, double *A);
        static void write_array_JSON(Json::Value root, int num_rows, int num_cols, double *A);
        static void write_matrix_JSON(Json::Value root, MatrixXd &A);
        static void write_vector_JSON(Json::Value root, VectorXd &b);
        static void write_int_JSON(Json::Value root, int& A);
        static void JSON_to_array(int rows, int cols, Json::Value root, double *A);
        static void print_array(int num_rows, int num_cols, double *A);
        static void array_to_matrix(int num_rows, int num_cols, double *A, MatrixXd &A_matrix);
        static void array_to_vector(int num_rows, int num_cols, double *A, VectorXd &b_vec);
        static void matrix_to_array(int num_rows, int num_cols, double *A, MatrixXd &A_matrix);
        static void vector_to_array(int num_rows, int num_cols, double *A, VectorXd &b_vec);
        static double toDeg(double _theta);
        static double toRad(double _theta);
        static double unwrap(double &theta);
        static double find_best_angle(double given_theta, double nom_angle);
        static void push_back(MatrixXd& A, MatrixXd &A_tmp, VectorXd& b, VectorXd &b_tmp, int &row_start);
        static void convertVector2Double(VectorXd b, double* c);
        static void convertMatrix2SymSparse(Eigen::SparseMatrix<double,Eigen::RowMajor> A_sparse, int* irowA, int* jrowA, double* dA);
        static void convertMatrix2Sparse(Eigen::SparseMatrix<double,Eigen::RowMajor> A_sparse, int* irowA, int* jrowA, double* dA);
        static void solve_quadprog(Eigen::SparseMatrix<double,Eigen::RowMajor> Q_sparse, VectorXd f,
                                      Eigen::SparseMatrix<double,Eigen::RowMajor> Ain_sparse, VectorXd bin,
                                      Eigen::SparseMatrix<double,Eigen::RowMajor> Aeq_sparse, VectorXd beq,
                                      double& objval,
                                      VectorXd &sol );
        static void placeSparse(Eigen::SparseMatrix<double,Eigen::RowMajor> &A, MatrixXd B, int i, int j);
        static void pushSparse(Eigen::SparseMatrix<double,Eigen::RowMajor>& A, MatrixXd& A_tmp, VectorXd& b, VectorXd& b_tmp, int &row_start);
        static void assign_sparse(vector<T>& A, MatrixXd A_tmp, int row_start, int j);
};



#endif

