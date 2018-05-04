//Common libraries
#include <iostream>
#include <stdio.h>
#include <math.h>
#include <sys/time.h>
#include <stdlib.h>
#include <fstream>
//External libraries
#include <Eigen/Dense>
#include <unistd.h>

#include "QpGenData.h"
#include "QpGenDense.h"
#include "QpGenVars.h"
#include "QpGenResiduals.h"
#include "GondzioSolver.h"
#include "QpGenSparseMa27.h"
#include "cQpGenSparse.h"

//Custom Libraries
#include "PusherSlider.h"
#include "Helper.h"
#define PRINT_LEVEL 0

using namespace std;
using namespace Eigen;


//Static Constructor
double Helper::g = 9.81;

double Helper::doubleGaussQuad(double ag, double bg, double cg, double dg)
{

	double h1 = (bg-ag)/2;
	double h2 = (bg+ag)/2;
	double h3 = (dg-cg)/2;
	double h4 = (dg+cg)/2;

	double w1g = 1;
	double w2g = 1;

	double x1g = sqrt(1.0f/3);
	double x2g = -sqrt(1.0f/3);
    
    Vector2d point1;
    Vector2d point2;
    Vector2d point3;
    Vector2d point4;

    point1 <<h1*x1g+h2, h3*x1g+h4;
    point2 <<h1*x1g+h2, h3*x2g+h4;
    point3 <<h1*x2g+h2, h3*x1g+h4;
    point4 <<h1*x2g+h2, h3*x2g+h4;
    
    PusherSlider pusher_slider;
    vFunctionCall fun = &PusherSlider::geometry;

    double integral = h1*h3 *( w1g*w1g*(pusher_slider.*fun)(point1) + w1g*w2g*(pusher_slider.*fun)(point2) + w2g*w1g*(pusher_slider.*fun)(point3) + w2g*w2g* (pusher_slider.*fun)(point4));
    return integral;
}	

//~ double Helper::cross2d(Vector2d r, Vector2d v)
double Helper::cross2d(Vector2d r, Vector2d v)
{
    Vector3d r3;
    Vector3d v3;
    Vector3d w3;
    MatrixXd r_cross(3,3);
    double w;

    r3 << r(0),r(1),0;
    v3 << v(0),v(1),0;
    r_cross << 0, -r3(2), r3(1), r3(2), 0, -r3(0), -r3(1), r3(0), 0;
    w3 = r_cross*v3;
    w = w3(2);
    return w;
}

Vector2d Helper::cross3d(double r, Vector2d x)
{
    Vector3d r3;
    Vector3d x3;
    Vector3d w3;
    Vector2d w;
    MatrixXd r_cross(3,3);
    
    r3 << 0,0,r;
    x3 << x(0),x(1),0;
    
    r_cross << 0, -r3(2), r3(1), r3(2), 0, -r3(0), -r3(1), r3(0), 0;
    w3 = r_cross*x3;
    w = w3.segment<2>(0);
    
    return w;
}

MatrixXd Helper::C3_2d(double theta)
{
    MatrixXd C(2,2);
    C << cos(theta), sin(theta), -sin(theta), cos(theta);  
    return C;
}

Matrix3d Helper::buildSib(double theta){
    //Initialize matrices
    Matrix2d Cbi;
    Vector2d tmp1;
    MatrixXd tmp2(2,3);
    MatrixXd tmp3(1,3);
    Matrix3d Sib;

    Cbi = Helper::C3_2d(theta);
    tmp1 << 0,0;
    tmp2 << Cbi.transpose(), tmp1;
    tmp3 <<0,0,1;
    Sib << tmp2, tmp3;
    return Sib;
}
    
//*************************************************************************
MatrixXd  Helper::cross_op(MatrixXd w)
{
	MatrixXd Omega(3,3);
	Omega << 0,-w(2),w(1),w(2),0,-w(0),-w(1),w(0),0;
	return Omega;
}

//********************************************************************
double Helper::gettime()
{
	char fmt[64];
	char buf[64];
	struct timeval tv;
	struct tm *tm;

	double test_sec;
	double test_usec;
	double test_usec2;
	double t_ini;

	gettimeofday(&tv, NULL);
	tm = localtime(&tv.tv_sec);
	test_sec = tv.tv_sec;
	test_usec = tv.tv_usec;
	test_usec2 = test_usec/(1000000);
	t_ini = test_sec+test_usec2;

	return t_ini;
}
//******************************
double Helper::smooth(double data, float filterVal, double smoothedVal)
{
  if (filterVal > 1){      // check to make sure param's are within range
    filterVal = .99;
  }
  else if (filterVal <= 0){
    filterVal = 0;
  }
  smoothedVal = (data * (1 - filterVal)) + (smoothedVal  *  filterVal);
  return smoothedVal;
}
//***************************************
void Helper::write_file(FILE* myFile, int num_rows, int num_cols, double *A)
{
    int i;
    int j;
    float value;
        const int RowOffset = (0 * num_cols);
        for (j=0;j<num_cols;j++)
        {;
        value = A[RowOffset + j] ;
        if (j==num_cols-1){
            fprintf(myFile, "%e \n",  value);}
        else
        {
            fprintf(myFile, "%e ",  value);
        }
    }
}
//***************************************
void Helper::write_array_JSON(Json::Value root, int num_rows, int num_cols, double *A)
{
    for (int i=0;i<num_rows;i++){
	const int RowOffset = (i * num_cols);
	for (int j=0;j<num_cols;j++)
	    {
	    A[RowOffset + j] = root[i][j].asDouble();
	    }
    }
}
//***************************************
void Helper::write_matrix_JSON(Json::Value root, MatrixXd &A)
{

  for (int i=0;i<A.rows();i++){
    for (int j=0;j<A.cols();j++)
        {
        A(i,j) = root[i][j].asDouble();
        }
    }
}
//***************************************
void Helper::JSON_to_array(int rows, int cols, Json::Value root, double *A)
{
    for (int i=0;i<rows;i++){
	const int RowOffset = (i * cols);
	for (int j=0;j<cols;j++)
	    {
	    A[RowOffset + j] = root[i][j].asDouble();
	    //~ cout<<A[RowOffset + j] << " ";
	    }
	    //~ cout<<"" <<endl;
    }
}
//***************************************
void Helper::write_vector_JSON(Json::Value root, VectorXd &A)
{
    for (int i=0;i<A.size();i++){
			A(i) = root[i][0].asDouble();
		}
}
//***************************************
void Helper::write_int_JSON(Json::Value root, int& A)
{
	A = root.asInt();
}
//***************************************
void Helper::print_array(int num_rows, int num_cols, double *A)
{
    MatrixXd M(num_rows, num_cols);
    array_to_matrix(num_rows, num_cols, A, M);
    cout<< M << endl;
}
//***************************************
void Helper::array_to_matrix(int num_rows, int num_cols, double *A, MatrixXd &A_matrix)
{
    for (int i=0;i<num_rows;i++){
	const int RowOffset = (i * num_cols);
	for (int j=0;j<num_cols;j++)
	    {
	    A_matrix(i,j) = A[RowOffset + j];
	    }
    }
}
//***************************************
void Helper::array_to_vector(int num_rows, int num_cols, double *A, VectorXd &b_vec)
{
    for (int i=0;i<num_rows;i++){
	    b_vec(i) = A[i];
    }
}
//***************************************
void Helper::matrix_to_array(int num_rows, int num_cols, double *A, MatrixXd &A_matrix)
{
    for (int i=0;i<num_rows;i++){
	const int RowOffset = (i * num_cols);
	for (int j=0;j<num_cols;j++)
	    {
	    A[RowOffset + j] = A_matrix(i,j);
	    }
    }
}
//***************************************
void Helper::vector_to_array(int num_rows, int num_cols, double *A, VectorXd &b_vec)
{
    for (int i=0;i<num_rows;i++){
	    A[i] = b_vec(i);
    }
}
//***************************************
double Helper::toDeg(double _theta)
{
    double theta = (_theta*180.0)/3.1415926535897;
    return theta;
}
//***************************************
double Helper::toRad(double _theta)
{
    double theta = (_theta*3.1415926535897)/180.0;
    return theta;
}
//***************************************
double Helper::unwrap(double &theta)
{
  if (theta<-3.141592653589793){
    theta += 6.283185307179586;
  }
  else if (theta>3.141592653589793) {
    theta -= 6.283185307179586;
  }
}

//***************************************
double Helper::find_best_angle(double given_theta, double nom_angle)
{
  double pi = 3.141592653589793;
  double min_angle;
  double test_angle;
  double dif_angle;
  VectorXd vecDif(11);
  VectorXd vecValues(11);
  MatrixXf::Index   minIndex;
  double minVal;

  int counter = 0;
  for (int i=-5;i<6;i++){
    test_angle = given_theta + i*2*pi;
    vecValues(counter) = test_angle;
    vecDif(counter) = abs(test_angle - nom_angle);
    counter ++;
  }
  minVal = vecDif.minCoeff(&minIndex);
  return vecValues(minIndex);

}

void Helper::convertVector2Double(VectorXd b, double* c){
  for (int i=0;i<b.size();i++){
    c[i] = b(i);
  }
}

void Helper::convertMatrix2SymSparse(Eigen::SparseMatrix<double,Eigen::RowMajor> A_sparse, int* irowA, int* jrowA, double* dA){

  int counter = 0;
  for (int k = 0; k < A_sparse.outerSize(); ++k){
    for (SparseMatrix<double,Eigen::RowMajor>::InnerIterator it(A_sparse, k); it; ++it){
      if (it.col()<=it.row()) {
        dA[counter] = it.value();
        irowA[counter] = it.row();
        jrowA[counter] = it.col();
        counter = counter + 1;
      }
    }
  }
}

void Helper::convertMatrix2Sparse(Eigen::SparseMatrix<double,Eigen::RowMajor> A_sparse, int* irowA, int* jrowA, double* dA){

  int counter = 0;
  for (int k = 0; k < A_sparse.outerSize(); ++k){
    for (SparseMatrix<double,Eigen::RowMajor>::InnerIterator it(A_sparse, k); it; ++it){
      dA[counter] = it.value();
      irowA[counter] = it.row();
      jrowA[counter] = it.col();
      counter = counter + 1;
    }
  }
}

void Helper::solve_quadprog(Eigen::SparseMatrix<double,Eigen::RowMajor> Q_sparse, VectorXd f,
                            Eigen::SparseMatrix<double,Eigen::RowMajor> Ain_sparse, VectorXd bin,
                            Eigen::SparseMatrix<double,Eigen::RowMajor> Aeq_sparse, VectorXd beq,
                            double& objval, VectorXd& sol){

//  double t0 = Helper::gettime();
  //Cost
  const int nx   = Q_sparse.rows();
  const int nnzQ = Q_sparse.nonZeros();
  int    irowQ[nnzQ];// = {  0,   1,   1 };
  int    jcolQ[nnzQ];// = {  0,   0,   1 };
  double    dQ[nnzQ];// = {  8,   2,  10 };
  double c[nx];

  //Variable bounds
  double  xupp[nx] = {0};
  char   ixupp[nx] = {0};
  double  xlow[nx] = {0};
  char   ixlow[nx] = {0};

  //Equality constraints
  int m_eq       = Aeq_sparse.rows();
  double b[m_eq];

  //Inequality constraints
  int nnzA       = Aeq_sparse.nonZeros();
  int   irowA[nnzA];
  int   jcolA[nnzA];
  double   dA[nnzA];

  //Inequality constraints
  const int m_ineq   = Ain_sparse.rows();
  double clow[m_ineq] = {0};
  char  iclow[m_ineq] = {0};
  double cupp[m_ineq];
  char  icupp[m_ineq];
  for (int i=0;i<nx;i++){icupp[i] = 1;}

  //Inequality constraints
    const int nnzC = Ain_sparse.nonZeros();
  int   irowC[nnzC];
  int   jcolC[nnzC];
  double   dC[nnzC];

  Helper::convertMatrix2SymSparse(Q_sparse, irowQ, jcolQ, dQ);
  Helper::convertVector2Double(f, c);
  Helper::convertVector2Double(beq, b);
  Helper::convertVector2Double(bin, cupp);
  Helper::convertMatrix2Sparse(Aeq_sparse, irowA, jcolA, dA);
  Helper::convertMatrix2Sparse(Ain_sparse, irowC, jcolC, dC);

  double x[nx], gamma[nx], phi[nx];
  double y[m_eq];
  double z[m_ineq], lambda[m_ineq], pi[m_ineq];
  int ierr;
//  double t0 = Helper::gettime();

  qpsolvesp( c, nx, irowQ, nnzQ, jcolQ, dQ, xlow, ixlow, xupp, ixupp,
             irowA, nnzA, jcolA, dA, b, m_eq,
             irowC, nnzC, jcolC, dC,
             clow,  m_ineq,   iclow, cupp, icupp,
             x, gamma, phi,
             y,
             z, lambda, pi, &objval, PRINT_LEVEL, &ierr );
//  double tf = Helper::gettime();
//  cout<<"transormation:  "<<tf-t0<<endl;

  for (int i = 0; i < nx; i++) {
    sol(i) = x[i];
  }

//  if( ierr != 0 ) {
//    fprintf( stderr, "Couldn't solve it.\n" );
//  } else {
//    int i;
//
//    printf(" Final Objective: %g\n\n", objval);
//    printf("Solution:...\n");
//    for (i = 0; i < nx; i++) {
//      printf("x[%2d] = %g\n", i, x[i]);
//    }
//  }

}

void Helper::push_back(MatrixXd& A, MatrixXd &A_tmp, VectorXd& b, VectorXd &b_tmp, int &row_start)
{
  int counter = 0;
  for (int i=row_start; i<row_start + A_tmp.rows();i++){
    A.row(i) = A_tmp.row(counter);
    b(i) = b_tmp(counter);
    counter++;
  }
  row_start = row_start + A_tmp.rows();
  //final call
  //A.middleRows(0, row_start)
}

void Helper::pushSparse(Eigen::SparseMatrix<double,Eigen::RowMajor>& A, MatrixXd &A_tmp, VectorXd& b, VectorXd &b_tmp, int &row_start)
{
  int counter = 0;
  int counter2 = 0;
  for (int i=row_start; i<row_start + A_tmp.rows();i++){
//    A.row(i) = A_tmp.row(counter);
    for (int lv2 =0; lv2<A_tmp.cols(); lv2++){
      A.insert(i,lv2) = A_tmp(counter, counter2);
      counter2++;
    }
    b(i) = b_tmp(counter);
    counter++;
    counter2 =0 ;
  }
  row_start = row_start + A_tmp.rows();
  //final call
  //A.middleRows(0, row_start)
}
void Helper::assign_sparse(vector<T>& A, MatrixXd A_tmp, int row_start, int j)
{
  int counter = 0;
  int counter2 = 0;
  for (int i=row_start; i<row_start + A_tmp.rows();i++){
    for (int lv2 =j; lv2<j+A_tmp.cols(); lv2++){
      if (A_tmp(counter, counter2)!=0){
        A.push_back(T(i,lv2,A_tmp(counter, counter2)));
      }

      counter2++;
    }
    counter++;
    counter2 =0 ;
  }
}

void Helper::placeSparse(Eigen::SparseMatrix<double,Eigen::RowMajor> &A, MatrixXd B, int i, int j){

  int counter1 = 0;
  vector<int> a;
  a.reserve(30);
//  int a[500];
  int counter2 = 0;
  int counter3 = 0;
  for (int lv1 =i; lv1<i+ B.rows(); lv1++){
    for (int lv2 =j; lv2<j+ B.cols(); lv2++){
      A.insert(lv1,lv2) = B(counter1, counter2);
//      a.push_back(B(counter1, counter2));
      counter2++;
      counter3++;
//      a++;
    }
    counter2 = 0;
    counter1++;
  }
//  cout<<"acounter "<< a<<endl;
}
//void Helper::block_sparse(Eigen::SparseMatrix<double,Eigen::RowMajor> &A, MatrixXd &B, int i, int j){
//
//  int counter1 = 0;
//  int counter2 = 0;
//  for (int lv1 =i; lv1<i+ B.rows(); lv1++){
//    for (int lv2 =j; lv2<j+ B.cols(); lv2++){
//      A.insert(lv1,lv2) = B(counter1, counter2);
//      counter2++;
//    }
//    counter2 = 0;
//    counter1++;
//  }
//}counter1