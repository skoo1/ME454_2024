#include "../include/mymat.hpp"
#include "../include/myQuaternion.hpp"

int main()
{
    // feel free to test your function.
    Quaternion q(1,2,3,4);
    Vec3 omega(1,2,3);
    q.normalized();
    q.display();
    q = getQuaternionBetweenTimeStep(omega, 0.1);
    q.normalized();
    q.display();
    Quaternion q1(1,0,0,1);
    Quaternion q2(1,0,1,0);
    q1.normalized();
    q2.normalized();
    q = quatmulquat(q1, q2);
    q.normalized();
    q.display();
    Mat33 mat1 = quat2mat(q);
    mat1.display();
    q = mat2quat(mat1);
    q.normalized();
    q.display();

    /// Above sample test cases are printed like below
    //Quaternion : {w : 0.182574, x : 0.365148, y : 0.547723, z : 0.730297}
    //Quaternion : {w : 0.982551, x : 0.0497088, y : 0.0994177, z : 0.149127}
    //Quaternion : {w : 0.5, x : -0.5, y : 0.5, z : 0.5}
    //Matrix :
    //0       -1      0
    //0       0       1
    //                -1      0       0
    //
    //Quaternion : {w : 0.5, x : -0.5, y : 0.5, z : 0.5}

    return 0;
}