#include <math.h>
#include <tuw_gazebo_plugins/gazebo_ros_physics_utils.h>

namespace gazebo
{
namespace physics_utils
{
double fsgn(double val)
{
  return (0 < val) - (val < 0);
}

double brake_to_stop_torque(double angularVelocity, double iyy, double simulationStepSeconds)
{
  double acceleration = angularVelocity / simulationStepSeconds;
  return iyy * acceleration;
}

double limit_brake_torque(double angularVelocity, double iyy, double brakeTorque, double appliedTorqe,
                          double simulationStepSeconds)
{
  double maxBrakeTorque = brake_to_stop_torque(angularVelocity, iyy, simulationStepSeconds) - appliedTorqe;
  return fsgn(brakeTorque) * fmin(fabs(maxBrakeTorque), fabs(brakeTorque));
}
}
}