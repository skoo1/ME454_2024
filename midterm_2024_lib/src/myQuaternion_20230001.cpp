#include "../include/myQuaternion.hpp"
#include <limits> // For std::numeric_limits

//////////////////// TO DO  ////////////////////
// TODO: refer "myQuaternion.hpp" and complete below functions.
Quaternion::Quaternion() {
}
Quaternion::Quaternion(double w, double x, double y, double z) {
}
double Quaternion::w() {
    return 0.0;
}
double Quaternion::x() {
    return 0.0;
}
double Quaternion::y() {
    return 0.0;
}
double Quaternion::z() {
    return 0.0;
}
void Quaternion::inversed(){
}
void Quaternion::normalized(){
}
void Quaternion::display()
{
}
Quaternion getQuaternionBetweenTimeStep(Vec3 w, double delta_t){
    Quaternion q;
    return q;
}
Quaternion quatmulquat(Quaternion q1, Quaternion q2){
    return q2;
}
Vec3 vecRotatedByQuat(Vec3 v, Quaternion q){
    return v;
}
Quaternion mat2quat(const Mat33& mat){
    Quaternion quat;
    return quat;
}
Mat33 quat2mat(Quaternion& q) {
    Mat33 mat;
    return mat;
}
//////////////////// TO DO end ////////////////////
