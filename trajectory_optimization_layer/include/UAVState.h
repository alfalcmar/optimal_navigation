
#ifndef UAVSTATE_H_
#define UAVSTATE_H_

#include <math.h>       /* cos */
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