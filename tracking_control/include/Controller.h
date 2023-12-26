#include <iostream>
#include <Eigen/Dense>
#include "Tool.h"
using namespace std;
 
typedef Eigen::Matrix<double, 3, 3> Matrix3x3;
typedef Eigen::Matrix<double, 3, 1> Matrix3x1;
typedef Eigen::Matrix<double, 2, 1> Matrix2x1;
typedef Eigen::Matrix<double, 2, 2> Matrix2x2;
typedef Eigen::Matrix<double, 3, 2> Matrix3x2;
typedef Eigen::Matrix<double, 2, 3> Matrix2x3;
 
//state variables: X = [x_e  y_e  yaw_e]^T
//control input: U = [v_e  kesi_e]^T
 
class Controller
{
private:

	double L;//vehicle wheelbase
	double T;//sampling interval
	double x_car, y_car, yaw_car, x_d, y_d, yaw_d;//vehicle pose and target point pose
	double v_d, kesi_d;//desired speed and front wheel deviation angle

	int temp = 0;

	//depending on the controller desigh
	Matrix3x3 A_d;
	Matrix3x2 B_d;
	Matrix3x3 Q;
	Matrix2x2 R;
	Matrix3x1 X_e;
	Matrix2x1 U_e;
	
	double Q3[3];//LQR 
	double R2[2];//LQR
	
public:

	void initial();
	
	void param_struct();
	Matrix2x3 cal_Riccati();
	U cal_vel();//Caculate the control 
	void test();
};
 
 
 