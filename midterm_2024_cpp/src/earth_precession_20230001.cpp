#include <iostream>
#include <fstream>
#include <string>
#include <thread>
#include <chrono>
#include "../include/mymat.hpp"
#include "../include/myQuaternion.hpp"

int main()
{
    std::ofstream q_results("q_results_cpp.csv");
    std::ofstream w_results("w_results_cpp.csv");

    double h = 0.001; // simulation timestep, 0.001 seconds (1 ms)
    int n_sim = 1000000; // number of simulation steps (1000 s)
    int rec_steps = 100; // record the result every n steps (every 0.1 s)
    Quaternion Q(0.97904547, 0, -0.20364175, 0); // initial orientation quaternion
    Mat33 I_lcs(120., 120., 120.);  // inertia of the body (in Earth local coordinate system)
    Vec3 w_lcs(0.0, 0.0, 10.0); // angular velocity in Earth local coordinate system
    Vec3 wd_lcs(0.0, 0.0, 0.0); // angular acceleration in Earth local coordinate system
    std::cout << "Simulation start" << std::endl;

    //////////////// TODO ////////////////
    // TODO: you can set yout own variavble.

    for (int i_sim = 0; i_sim < n_sim; i_sim++)
    {
        // TODO: implement semi-implicit Euler method to simulate the earth precession



        // TODO: record q_results_cpp.csv contains earth quaternion in global coordinate system (w,x,y,z)
        // TODO: record w_results_cpp.csv contains earth angular velocity in global coordinate system (x,y,z)
        if (i_sim % rec_steps == 0)
        {
            // record the orientation (gcs)

            q_results << "\n";

            // record the angualr velocity (gcs)

            w_results << "\n";
        }
        //////////////// TODO end ////////////////
    }
    std::cout << "Simulation end" << std::endl;

    // close the file
    q_results.close();
    w_results.close();

    return 0;
}