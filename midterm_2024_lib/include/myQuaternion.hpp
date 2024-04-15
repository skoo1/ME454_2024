#ifndef _MYQUATERNION_H_
#define _MYQUATERNION_H_
#include "mymat.hpp"
#include <cmath>
// Define a small epsilon value for floating-point comparisons
#include <limits> // For std::numeric_limits
// Define a small epsilon value for floating-point comparisons
const double _FLOAT_EPS = std::numeric_limits<double>::epsilon();
const double _EPS4 = _FLOAT_EPS * 4.0;

class Quaternion : public Vec4{
public:
    Quaternion(); // constructor
    Quaternion(double w, double x, double y, double z); // constructor with values, [w, x, y, z]
    Quaternion(const Vec4& vec) : Vec4(vec) {  };// cast to MatBase
    double w(); // get value of w in quaternion [w,x,y,z]
    double x(); // get value of x in quaternion [w,x,y,z]
    double y(); // get value of y in quaternion [w,x,y,z]
    double z(); // get value of z in quaternion [w,x,y,z]
    void normalized(); // nomralized the quaternion, to become its norm 1;
    void inversed(); // get inverse of the quaternion. q*q_inv becomes quaternion [1,0,0,0];
    // print the quaternion like below form
    // Quaternion = {w : 1, x : 0, y : 0, z: 0}
    void display(); // vector visualization (optional)

};
// angular velocity and timestep makes rotation
// assume that angular acceleration is zero
// return quaternion that explain the rotation between time step
Quaternion getQuaternionBetweenTimeStep(Vec3 w, double delta_t);

// quaternion multiplication
// return quaternion that explain q1*q2;
Quaternion quatmulquat(Quaternion q1, Quaternion q2);

// return rotated vector by quaternion
Vec3 vecRotatedByQuat(Vec3 v, Quaternion q);

// return quaternion that is converted from rotation matrix
Quaternion mat2quat(const Mat33& mat);

// return rotation matrix that is converted from quaternion
Mat33 quat2mat(Quaternion& q);

#endif // _MYQUATERNION_H_
