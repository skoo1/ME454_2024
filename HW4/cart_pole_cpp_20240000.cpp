#include <iostream>
#include <fstream>
#include <cmath>
#include <Eigen/Dense>


class Logger {
public:
    std::ofstream file;
    Logger(const std::string& filename) {
        file.open(filename, std::ios::out);
        file << "Time,Position,theta,Force\n";
    }
    ~Logger() {
        if (file.is_open()) {
            file.close();
        }
    }
    void log(double time, double position, double theta, double Force) {
        file << time << "," << position << "," << theta << "," << Force << "\n";
    }
};

class CartPole {
public:
    double x1, d_x1, theta2, d_theta2;
    double F;
    double m_1 = 1.0, m_2 = 8.0, I_1 = 0.024167, I_2 = 1.01, length = 0.6, g = 9.81; // Model parameters
    int count = 0;
    Logger logger;

    CartPole() : x1(0.0), d_x1(0.0), theta2(0.523599), d_theta2(0.0), F(0.0), logger("results_20240000.csv") {} // Initial condition

    void semi_implicit_integration() {
        double h = 0.001;

        pid_controller();

        ////////////////// TODO //////////////////////
        /// Implement the semi-implicit Euler method and calculate x1, d_x1, theta2, d_theta2 (Part2. 2-2)

        x1 = 0; // should be modified
        theta2 = 0; // should be modified
        d_x1 = 0; // should be modified
        d_theta2 = 0; // should be modified

        ////////////////// TODO END //////////////////

        // loogging
        logger.log(h * count, theta2, x1, F);
        count++;
    }

    Eigen::MatrixXd Calculate_Left_Matrix() {
        Eigen::MatrixXd matrix(10, 10);
        matrix.setZero();

        ////////////////// TODO //////////////////////
        /// Calculate the matrix on the left of the motion equation (Part2. 2-1)



        ////////////////// TODO END //////////////////

        return matrix;
    }

    Eigen::VectorXd Calculate_Right_Vector() {
        Eigen::VectorXd vec(10);

        ////////////////// TODO //////////////////////
        /// Calculate the vector on the right of the motion equation (Part2. 2-1)



        ////////////////// TODO END //////////////////

        return vec;
    }

    void pid_controller()
    {
        ////////////////// TODO //////////////////////
        /// Calculate the force to keep the pole vertical (Part3. 3-1)

        F = 0; // should be modified

        ////////////////// TODO END //////////////////
    }
};

int main() {
    CartPole cartpole;
    for (int i = 0; i <= 30000; ++i) {
        cartpole.semi_implicit_integration();
    }
    return 0;
}
