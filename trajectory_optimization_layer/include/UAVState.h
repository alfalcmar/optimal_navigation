
#ifndef UAVSTATE_H_
#define UAVSTATE_H_

#include <math.h>       /* cos */
#include <memory>


  struct InitialGuess{
    InitialGuess(const int time_horizon) : x(new double[time_horizon]{0.0}),
                     y(new double[time_horizon]{0.0}),
                     z(new double[time_horizon]{0.0}),
                     vx(new double[time_horizon]{0.0}),
                     vy(new double[time_horizon]{0.0}),
                     vz(new double[time_horizon]{0.0}),
                     ax(new double[time_horizon]{0.0}),
                     ay(new double[time_horizon]{0.0}),
                     az(new double[time_horizon]{0.0}){}
    std::shared_ptr<double[]> x;  
    std::shared_ptr<double[]> y;
    std::shared_ptr<double[]> z;
    std::shared_ptr<double[]> vx;
    std::shared_ptr<double[]> vy;
    std::shared_ptr<double[]> vz;
    std::shared_ptr<double[]> ax;
    std::shared_ptr<double[]> ay;
    std::shared_ptr<double[]> az;
  };

struct Quaternion{
    double x = 0.0;
    double y = 0.0;
    double z = 0.0;
    double w = 0.0;
  };
  struct Pose{
    double x = 0.0;
    double y = 0.0;
    double z = 0.0;
  };
  struct Velocity{
    double x = 0.0;
    double y = 0.0;
    double z = 0.0;
  };

class UavState{
  public:
    Pose pose;
    Quaternion quaternion;
    Velocity velocity;
    bool has_pose = false;
  private:
   Quaternion toQuaternion(const double pitch, const double roll, const double yaw);

};

#endif