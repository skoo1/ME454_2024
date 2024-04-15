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
    double w();
    double x();
    double y();
    double z();
    // nomralized the quaternion, to become its norm 1;
    void normalized();
    void inversed();
    // print the quaternion like below form
    // Quaternion = {w : 1, x : 0, y : 0, z: 0}
    void display(); // vector visualization (optional)

};
// angular velocity, timestep -> quaternion between timestep;
// assume that angular acceleration is zero;
Quaternion getQuaternionBetweenTimeStep(Vec3 w, double delta_t);
// quaternion multiplication q1*q2;
Quaternion quatmulquat(Quaternion q1, Quaternion q2);
// rotate vector by quaternion
Vec3 vecRotatedByQuat(Vec3 v, Quaternion q);
// convert rotation matrix to quaternion; // optional
Quaternion mat2quat(const Mat33& mat);
// convert quaternion to rotation matrix; // optional
Mat33 quat2mat(Quaternion& q);

#endif // _MYQUATERNION_H_
