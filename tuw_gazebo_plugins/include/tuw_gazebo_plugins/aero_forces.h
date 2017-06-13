#ifndef AERO_FORCES_H
#define AERO_FORCES_H

#include <gazebo/common/common.hh>
#include <sdf/sdf.hh>

namespace gazebo {

class AeroForce {
 public:
  AeroForce(double fx, double fz, ignition::math::Vector3d pose);
  ~AeroForce();
  double fx;
  double fz;
  ignition::math::Vector3d pose;
};

class AeroModel {
 public:
  AeroModel();
  ~AeroModel();
  void LoadParam(sdf::ElementPtr _sdf);
  void AddForces(double velocity, physics::LinkPtr &baseLink);

 private:
  std::vector<AeroForce> aeroForces_;
  void ProcessPart(sdf::ElementPtr aeroPart);
};
}

#endif
