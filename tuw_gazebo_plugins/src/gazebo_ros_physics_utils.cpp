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

double brake_to_stop_torque(double angularVelocity, double izz, double simulationStepSeconds)
{
  double acceleration = angularVelocity / simulationStepSeconds;
  return izz * acceleration;
}

double limit_brake_torque(double angularVelocity, double izz, double brakeTorque, double appliedTorqe,
                          double simulationStepSeconds)
{
  double maxBrakeTorque = brake_to_stop_torque(angularVelocity, izz, simulationStepSeconds) - appliedTorqe;
  return fsgn(brakeTorque) * fmin(fabs(maxBrakeTorque), fabs(brakeTorque));
}
}
}