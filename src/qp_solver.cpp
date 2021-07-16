#include "OsqpEigen/OsqpEigen.h"
#include <Eigen/Dense>
#include <iostream>


int main()
{

    OsqpEigen::Solver solver;

    solver.settings()->setVerbosity(true);
    solver.settings()->setWarmStart(true);

    solver.data()->setNumberOfVariables(7);
    solver.data()->setNumberOfConstraints(7);

    std::cout << std::endl;
}