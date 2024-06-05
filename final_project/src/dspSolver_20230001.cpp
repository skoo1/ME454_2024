#include "dspSolver.h"
#include <iostream>

// Implement this function
dspSolver::dspSolver(dspLinkConfig& linkConfig) : LinkConfig(linkConfig) {
    /// Solver variable initialization
    /// The link with index 0 is the ground link
    /// There are only two joint types : revolute and prismatic
    int link_num = LinkConfig.GetNumLink();
    int joint_num = LinkConfig.GetNumJoint();

    /// You have to initialize following variables
    q.setZero(99);
    qdot.setZero(99);
    qddot.setZero(99);
    M.setZero(99, 99);
    M_inv.setZero(99, 99);
    J.setZero(99, 99);
    J_dot.setZero(99, 99);
    F_ext.setZero(99);
    lambda.setZero(99);
}

// Implement this function
bool dspSolver::CalculateConstraintError(Eigen::VectorXd& error_c) {
    /// calculate error what you define as constraint

    return true;
}

// Implement this function
bool dspSolver::Make_M(void) { 
    /// make mass matrix : M

    return true;
}

// Implement this function
bool dspSolver::Make_J(void) {
    /// make Jacobian matrix which has first-order partial derivatives of vector q's components : J


    return true;
}

// Implement this function
bool dspSolver::Make_J_dot(void) {
    /// make time derivative of Jacobian matrix  : J_dot

    return true;
}

// Implement this function
bool dspSolver::Make_F_ext(void) {
    /// make a n-dimensional vector for external force/torque : F_ext
    /// it contains x direction of force, y direction of force, and torque
    
    return true;
}

// Implement this function
bool dspSolver::CalcLinAlg(void) {
    /// calculate second derivative of vector q : qddot
    
    return true;
}

// Implement this function
bool dspSolver::UpdateCurrentInfo(void) {
    /// load and update information what you need in class variables
    /// current vector q and its derivative should be loaded : q, q_dot

    Make_M();
    Make_J();
    Make_J_dot();
    Make_F_ext();
    return true;
}

// Implement this function
bool dspSolver::UpdateNextInfo(double timestep) {
    /// update vector q and its derivative : q, q_dot
    /// also save them in class variables for nex step
    
    return true;
}

// Implement this function
bool dspSolver::SetQDot(void) {
    /// update new q_dot information in each link structure

    return true;
}

// Implement this function
bool dspSolver::SetQ(void) {
    /// update new q information in each link structure

    return true;
}

// Do not change this function
bool dspSolver::GetQ(Eigen::VectorXd& q_record) {
    /// get current q information
    q_record = q;
    return true;
}

// Do not change this function
bool dspSolver::GetQDot(Eigen::VectorXd& qdot_record) {
    /// get current q_dot information 
    qdot_record = qdot;
    return true;
}
