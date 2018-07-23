namespace gazebo
{
namespace physics_utils
{
double fsgn(double val);

double brake_to_stop_torque(double angularVelocity, double izz, double simulationStepSeconds);

double limit_brake_torque(double angularVelocity, double izz, double brakeTorque, double appliedTorqe,
                          double simulationStepSeconds);
}
}