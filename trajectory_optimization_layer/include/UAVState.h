
#ifndef UAVSTATE_H_
#define UAVSTATE_H_

#include <math.h>       /* cos */
#include <memory>

#define ANSI_COLOR_RED     "\x1b[31m"
#define ANSI_COLOR_GREEN   "\x1b[32m"
#define ANSI_COLOR_YELLOW  "\x1b[33m"
#define ANSI_COLOR_BLUE    "\x1b[34m"
#define ANSI_COLOR_MAGENTA "\x1b[35m"
#define ANSI_COLOR_CYAN    "\x1b[36m"
#define ANSI_COLOR_RESET   "\x1b[0m"

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
struct Acc{
  double x = 0.0;
  double y = 0.0;
  double z = 0.0;
};

struct State{
  Pose pose;
  Quaternion quaternion;
  Velocity velocity;
  Acc acc;
  float heading;
  float heading_rate;
};

class UavState{
  public:
    State state;
    std::unique_ptr<State[]> solution_;
    bool has_pose = false;
  private:
   Quaternion toQuaternion(const double pitch, const double roll, const double yaw);

};

#endif