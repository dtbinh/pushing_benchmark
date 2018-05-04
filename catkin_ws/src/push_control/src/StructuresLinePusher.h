/*
 */

//~ using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;
using Eigen::Vector2d;
using Eigen::ArrayXf;

 
#ifndef STRUCTURE_H
#define STRUCTURE_H

//**************
struct outStarStruct{
    MatrixXd xcStar;
    MatrixXd ucStar;
    MatrixXd xsStar;
    MatrixXd usStar;
    MatrixXd AStar;
    MatrixXd BStar;
    MatrixXd tStar;
};
//**************
struct outStateNominal{
    VectorXd xcStar;
    VectorXd ucStar;
    VectorXd xsStar;
    VectorXd usStar;
    MatrixXd AStar;
    MatrixXd BStar;
};
//**************
struct xcStateStruct{
    Vector2d ribi;
    double theta;
    double ry;
    xcStateStruct(VectorXd xc){
        ribi = xc.segment(0,2);
        theta = xc(2);
        ry = xc(3);
        } ;
};
////**************
struct ucStateStruct{
    VectorXd fn;
    VectorXd ft;
    double dry;
    ucStateStruct(VectorXd uc){
                if (uc.size()==3){
                        fn.resize(1);
                        ft.resize(1);
                        fn << uc(0);
                        ft << uc(1);
                        dry = uc(2);
                }
                else{
                        fn.resize(2);
                        ft.resize(2);
                        fn = uc.segment(0,2);
                        ft = uc.segment(2,2);
                        dry = uc(4);
                }
        } ;
};
////~ **************
struct xsStateStruct{
    Vector2d ribi;
    Vector2d ripi;
    double theta;
    double thetap;
    xsStateStruct(VectorXd xs){
        ribi = xs.segment(0,2);
        theta = xs(2);
        ripi = xs.segment(3,2);
        thetap = xs(5);
        } ;
};
////~ **************
struct usStateStruct{
    Vector2d vp;
    double dthetap;
    usStateStruct(VectorXd us){
        vp = us.segment(0,2);
        dthetap = us(2);
        } ;
};

#endif


