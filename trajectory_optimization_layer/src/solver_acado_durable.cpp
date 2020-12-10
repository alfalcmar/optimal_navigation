
#include <solver_acado_durable.h>

using namespace NumericalSolver;

SolverDurable::SolverDurable(const float solving_rate, const int time_horizon, const std::shared_ptr<State[]> &initial_guess) : ACADOSolver::ACADOSolver(solving_rate, time_horizon, initial_guess){
  std::cout<<"constructor solver durable "<<std::endl;   
}

