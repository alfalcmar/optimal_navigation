#ifndef SOLVER_NODELET_H
#define SOLVER_NODELET_H

/* Includes //{ */
#include <nodelet/nodelet.h>

#ifdef USE_MRS_INTERFACE
#include <backendSolverMRS.h>
#else
#include <backendSolverUAL.h>
#endif

//}

namespace optimal_control_interface
{

/* //{ class DecomposeNode */

class SolverNodelet : public nodelet::Nodelet {

public:
  virtual void onInit();

private:
  void mainTimer(const ros::TimerEvent &evt);

private:
  // --------------------------------------------------------------
  // |                ROS-related member variables                |
  // --------------------------------------------------------------

private:
  // --------------------------------------------------------------
  // |                       Other variables                      |
  // --------------------------------------------------------------

private:
  // --------------------------------------------------------------
  // |                       Helper methods                       |
  // --------------------------------------------------------------

};

//}

}  // namespace solver_nodelet

#endif  // SOLVER_NODELET_H
