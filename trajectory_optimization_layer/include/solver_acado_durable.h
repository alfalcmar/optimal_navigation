#ifndef SOLVER_ACADO_DURABLE_H
#define SOLVER_ACADO_DURABLE_H

#include <solver_acado.h>
#include <ros/ros.h>

namespace NumericalSolver{}

class SolverDurable : public NumericalSolver::ACADOSolver{
    public:
        SolverDurable(const float solving_rate, const int time_horizon, const std::shared_ptr<State[]> &initial_guess);
};

}
#endif