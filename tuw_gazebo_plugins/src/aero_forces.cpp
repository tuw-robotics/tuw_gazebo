#include <ros/ros.h>
#include <tuw_gazebo_plugins/aero_forces.h>
#include <tuw_gazebo_plugins/gazebo_ros_utils.h>
#include <gazebo/common/common.hh>
#include <sdf/sdf.hh>

namespace gazebo {
AeroForce::AeroForce(double fx_, double fz_, ignition::math::Vector3d pose_)
    : fx(fx_), fz(fz_), pose(pose_) {}
AeroForce::~AeroForce() {}

AeroModel::AeroModel() {}

AeroModel::~AeroModel() {}

void AeroModel::LoadParam(sdf::ElementPtr _sdf) {
  if (_sdf->HasElement("aeroPart")) {
    sdf::ElementPtr aeroPart = _sdf->GetElement("aeroPart");
    while (aeroPart) {
      ProcessPart(aeroPart);
      aeroPart = aeroPart->GetNextElement("aeroPart");
    }
  }
  ROS_INFO("%lu aero parts found", aeroForces_.size());
}

template <typename T>
int sgn(T val) {
  return (T(0) < val) - (val < T(0));
}

void AeroModel::ProcessPart(sdf::ElementPtr aeroPart) {
  double fx = 0;
  double fz = 0;
  ignition::math::Vector3d pose(0, 0, 0);
  if (aeroPart->HasElement("fx")) {
    fx = aeroPart->Get<double>("fx");
  }
  if (aeroPart->HasElement("fz")) {
    fz = aeroPart->Get<double>("fz");
  }
  if (aeroPart->HasElement("x")) {
    pose.X(aeroPart->Get<double>("x"));
  }
  if (aeroPart->HasElement("y")) {
    pose.Y(aeroPart->Get<double>("y"));
  }
  if (aeroPart->HasElement("z")) {
    pose.Z(aeroPart->Get<double>("z"));
  }
  AeroForce force(fx, fz, pose);
  aeroForces_.push_back(force);
}

void AeroModel::AddForces(double velocity, physics::LinkPtr& baseLink) {
  double velSquared = 0 - (velocity * velocity);
  for (auto& aeroForce : aeroForces_) {
    ignition::math::Vector3d f(sgn(velocity) * velSquared * aeroForce.fx, 0,
                               velSquared * aeroForce.fz);
    baseLink->AddLinkForce(f, aeroForce.pose);
  }
}
}