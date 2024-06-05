#include <iostream>
#include <fstream>
#include <string>
#include <thread>
#include <chrono>
#include <Eigen/Dense>

double L1_, L2_, L3_, L0_;
double m1_, m2_, m3_;
double ixx1_, iyy1_, izz1_;
double ixx2_, iyy2_, izz2_;
double ixx3_, iyy3_, izz3_;
double y1_init_, y2_init_, y3_init_, z1_init_,z2_init_,z3_init_;
double q1_init_, q2_init_, q3_init_;
double q1dot_init_, q2dot_init_, q3dot_init_;

Eigen::MatrixXd M_(9,9), M_inv_(9,9);

double g = 9.81;

void initialize(){
    ////////////////// TODO //////////////////////
    L0_ = 3.0;

    m1_ = 1.0;
    L1_ = 0.5;
    ixx1_ = (1.0/12.0)*m1_*pow((2*L1_),2);
    iyy1_ = 0.0;
    izz1_ = 0.0;
    q1_init_ = 1.578;
    q1dot_init_ = 0.0;
    y1_init_ = L1_*cos(q1_init_);
    z1_init_ = L1_*sin(q1_init_);

    m2_ = 4.0;
    L2_ = 2.0;
    ixx2_ = (1.0/12.0)*m2_*pow((2*L2_),2);
    iyy2_ = 0.0;
    izz2_ = 0.0;
    q2_init_ = 0.3533;
    q2dot_init_ = 0.0;
    y2_init_ = 2*L1_*cos(q1_init_) + L2_*cos(q2_init_);
    z2_init_ = 2*L1_*sin(q1_init_) + L2_*sin(q2_init_);

    m3_ = 2.5;
    L3_ = 1.25;
    ixx3_ = (1.0/12.0)*m3_*pow((2*L3_),2);
    iyy3_ = 0.0;
    izz3_ = 0.0;
    q3_init_ = 1.2649;
    q3dot_init_ = 0.0;
    y3_init_ = L0_ + L3_*cos(q3_init_);
    z3_init_ = L3_*sin(q3_init_);

    M_.diagonal() << m1_, m1_, ixx1_, m2_, m2_, ixx2_, m3_, m3_, ixx3_;
    M_inv_.diagonal() << 1.0/m1_, 1.0/m1_, 1.0/ixx1_, 1.0/m2_, 1.0/m2_, 1.0/ixx2_, 1.0/m3_,1.0/m3_, 1.0/ixx3_;


    ////////////////// TODO END //////////////////////

}

Eigen::MatrixXd calculate_jacobian(Eigen::VectorXd q){
    Eigen::MatrixXd J;
    J.setZero(8, 9);

    ////////////////// TODO //////////////////////
    /// Calculate the Jacobian using the input parameter q
    double th1 = q(2);
    double th2 = q(5);
    double th3 = q(8);

    J(0,0) = 1.0;
    J(0,2) = L1_*sin(th1);

    J(1,1) = 1.0;
    J(1,2) = -L1_*cos(th1);

    J(2,2) = 2*L1_*sin(th1);
    J(2,3) = 1.0;
    J(2,5) = L2_*sin(th2);
    
    J(3,2) = -2*L1_*cos(th1);
    J(3,4) = 1.0;
    J(3,5) = -L2_*cos(th2);

    J(4,6) = 1.0;
    J(4,8) = L3_*sin(th3);

    J(5,7) = 1.0;
    J(5,8) = -L3_*cos(th3);

    J(6, 2) = -L1_ * sin(th1);
    J(6, 5) = -L2_ * sin(th2);
    J(6, 8) = L3_ * sin(th3);


    J(7, 2) = L1_ * cos(th1);
    J(7, 5) = L2_ * cos(th2);
    J(7, 8) = -L3_ * cos(th3);

    ////////////////// TODO END //////////////////////
    return J;
}

Eigen::MatrixXd calculate_jacobian_dot(Eigen::VectorXd q, Eigen::VectorXd qdot){
    Eigen::MatrixXd Jdot;
    Jdot.setZero(8, 9);

    ////////////////// TODO //////////////////////
    /// Calculate the time derivative of Jacobian using the input parameter q and qdot
    double th1 = q(2);
    double th2 = q(5);
    double th3 = q(8);

    double thdot1 = qdot(2);
    double thdot2 = qdot(5);
    double thdot3 = qdot(8);

    Jdot(0,2) = thdot1 * L1_*cos(th1);

    Jdot(1,2) = thdot1 * L1_*sin(th1);

    Jdot(2,2) = 2.0 * thdot1 * L1_*cos(th1);
    Jdot(2,5) = thdot2 * L2_*cos(th2);

    Jdot(3,2) = 2.0 * thdot1 * L1_*sin(th1);
    Jdot(3,5) = thdot2 * L2_*sin(th2);

    Jdot(4,8) = thdot3 * L3_*cos(th3);

    Jdot(5,8) = thdot3 * L3_*sin(th3);

    Jdot(6, 2) = -thdot1 * L1_ * cos(th1);
    Jdot(6, 5) = -thdot2 * L2_ * cos(th2);
    Jdot(6, 8) = thdot3 * L3_ * cos(th3);


    Jdot(7, 2) = -thdot1 * L1_ * sin(th1);
    Jdot(7, 5) = -thdot2 * L2_ * sin(th2);
    Jdot(7, 8) = thdot3 * L3_ * sin(th3);

    ////////////////// TODO END //////////////////////
    return Jdot;
}

Eigen::VectorXd calculate_Fext(Eigen::VectorXd q){
    Eigen::VectorXd Fext;
    Fext.setZero(9);
    ////////////////// TODO //////////////////////
    /// Calculate the external force using the input parameter q and return

    Fext(1) = -m1_*g;
    Fext(4) = -m2_*g;
    Fext(7) = -m3_*g;

    ////////////////// TODO END //////////////////////
    return Fext;
}

Eigen::VectorXd calculate_lambda(Eigen::MatrixXd J, Eigen::MatrixXd Jdot, Eigen::VectorXd Fext, Eigen::VectorXd q, Eigen::VectorXd qdot){
    Eigen::VectorXd lambda;
    lambda.setZero(8);
    ////////////////// TODO //////////////////////
    /// Calculate lambda which connects the constraint force and the jacobian

    Eigen::MatrixXd A = J * M_inv_*J.transpose();
    Eigen::VectorXd B = -Jdot * qdot;
    Eigen::VectorXd C = -J * M_inv_*Fext;

    lambda = A.inverse() * (B+C);

    ////////////////// TODO END //////////////////////
    return lambda;
}

Eigen::VectorXd calculate_Fc(Eigen::MatrixXd J, Eigen::VectorXd lambda){
    Eigen::VectorXd Fc;
    Fc.setZero(9);

    ////////////////// TODO //////////////////////
    /// Calculate the constraint force using the Jacobian and lambda

    Fc = J.transpose()*lambda;

    ////////////////// TODO END //////////////////////
    return Fc;
}

Eigen::VectorXd calculate_constraint_error(Eigen::VectorXd q){
    Eigen::VectorXd ConstraintErr;
    ////////////////// TODO //////////////////////
    /// Calculate the constraint errors under given state vector q
    ConstraintErr.setZero(8);
    double x1 = q(0);
    double y1 = q(1);
    double th1 = q(2);
    double x2 = q(3);
    double y2 = q(4);
    double th2 = q(5);
    double x3 = q(6);
    double y3 = q(7);
    double th3 = q(8);

    ConstraintErr(0) = x1 - L1_*cos(th1);
    ConstraintErr(1) = y1 - L1_*sin(th1);

    ConstraintErr(2) = -2*L1_*cos(th1) + x2 - L2_*cos(th2);
    ConstraintErr(3) = -2*L1_*sin(th1) + y2 - L2_*sin(th2);

    ConstraintErr(4) = x3 - L3_*cos(th3) - L0_;
    ConstraintErr(5) = y3 - L3_*sin(th3);

    ConstraintErr(6) = 2*L1_*cos(th1) + 2*L2_*cos(th2) - 2*L3_*cos(th3) - L0_;
    ConstraintErr(7) = 2*L1_*sin(th1) + 2*L2_*sin(th2) - 2*L3_*sin(th3);

    ////////////////// TODO END //////////////////
    return ConstraintErr;
}

int main()
{
    std::ofstream q_results("q_results_cpp.csv");
    std::ofstream qdot_results("qdot_results_cpp.csv");
    std::ofstream constraint_error_results("constraint_error_cpp.csv");

    double h = 0.001; // simulation timestep, 0.001 seconds (1 ms)
    int n_sim = 50000; // number of simulation steps (50 s)
    int rec_steps = 10; // record the result every n steps (every 0.1 s)
    initialize();

    Eigen::VectorXd q(9);
    Eigen::VectorXd qdot(9);
    q << y1_init_, z1_init_, q1_init_, y2_init_, z2_init_, q2_init_, y3_init_, z3_init_, q3_init_;
    qdot.setZero();

    std::cout << "Simulation start" << std::endl;
    
    //////////////// TODO ////////////////
    // TODO: you can set yout own variavble.

    double t = 0;
    Eigen::VectorXd current_qddot;
    current_qddot.setZero();

    for (int i_sim = 0; i_sim < n_sim; i_sim++)
    {

        // TODO: implement semi-implicit Euler method to simulate the four bar linkage motion

        Eigen::VectorXd Fext = calculate_Fext(q);
        Eigen::MatrixXd J = calculate_jacobian(q);
        Eigen::MatrixXd Jdot = calculate_jacobian_dot(q, qdot);
        Eigen::VectorXd lambda = calculate_lambda(J, Jdot, Fext, q, qdot);
        Eigen::VectorXd Fc = calculate_Fc(J, lambda);
        Eigen::VectorXd ConstraintError = calculate_constraint_error(q);

        current_qddot = M_inv_*(Fext + Fc);
        Eigen::VectorXd qdot_prev = qdot;
        qdot = qdot + h*current_qddot;
        q = q + h*qdot;

        if (i_sim % rec_steps == 0)
        {
            q_results << t << ",";
            qdot_results << t << ",";
            
            // TODO: record q_results_cpp.csv contains the value of the angle q1, q2, and q3 in radian (q1,q2,q3)

            q_results << q(2) << "," << q(5) << ","<< q(8) <<"\n";

            // TODO: record qdot_results_cpp.csv contains the value of the angular velocity qdot1, qdot2, and qdot3 in radian (qdot1,qdot2,qdot3)

            qdot_results << qdot(2) << "," << qdot(5) << ","<< qdot(8) <<"\n";

             // TODO: record constraint_error_cpp.csv contains the constraint error at this time step (error1,error2,...)
            
            for (int i=0;i<8;i++){
                if (i<7){
                    constraint_error_results << ConstraintError(i) << ", ";
                }
                else{
                    constraint_error_results << ConstraintError(i) << "\n";
                }
            }
        }
        //////////////// TODO end ////////////////
        t = t + h;
    }
    std::cout << "Simulation end" << std::endl;

    // close the file
    q_results.close();
    qdot_results.close();
    constraint_error_results.close();

    return 0;
}


