

#include "LQR.hpp"
using namespace Eigen;
#include <iostream>

int main(int argc, char **argv)
{
    const size_t CONTROL_DIMS = 1, STATE_DIMS = 2;


    Matrix<double ,STATE_DIMS, STATE_DIMS> A, Q;
    Matrix<double ,STATE_DIMS, CONTROL_DIMS> B;
    Matrix<double, CONTROL_DIMS, STATE_DIMS> K;
    Matrix<double, CONTROL_DIMS, CONTROL_DIMS> R;
    Q << 10, 0, 0, 1;
    R << 1;
    A << 0, 1, -2, -3;
    B << 0, 1;

    ct::optcon::LQR<2, 1> lqrSolver;
    lqrSolver.compute(Q, R, A, B, K, true, true);
    std::cout<<K<<std::endl;
}