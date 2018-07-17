#ifndef TIRE_PLUGIN_H
#define TIRE_PLUGIN_H

#include <ros/ros.h>
#include <tuw_gazebo_plugins/gazebo_ros_utils.h>
#include <boost/thread.hpp>
#include <gazebo/common/common.hh>

#define TO_RADPS(rpm) (M_PI / 30 * rpm)
#define TO_RADIANS(dg) ((dg)*M_PI / 180.0)
#define TO_DEGREES(rad) ((rad)*180.0 / M_PI)

namespace gazebo {
class TireModel : public ModelPlugin {
 public:
  TireModel();
  ~TireModel();
  void Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf) override;
  void Reset() override;
  void Init() override;

 protected:
  void UpdateChild();
  void FiniChild();

 private:
  void QueueThread();
  void OnContacts(ConstContactsPtr &_msg);
  double GetFx0(double slip, double Fz, double dFz, double camber);
  double GetFy0(double slipAngle, double Fz, double dFz, double camber);
  double GetCombinedFy(double slipAngle, double slip, double Fz, double dFz,
                       double camber);
  double GetCombinedFx(double slip, double Fz, double dFz, double camber);
  double GetRollingResistance(double Fz, double wheelVelocity, double Fx);
  double GetSlip(double vehicleVelocity, double wheelVelocity);
  double GetSelfAligningTorque(double slipAngle, double dFz, double camber,
                               double slip, double Fz, double Fy, double Fx);
  double GetCamberFromToeAngle(double leftToeAngle);

  GazeboRosPtr gazebo_ros_;
  physics::ModelPtr parent_;
  event::ConnectionPtr update_connection_;
  physics::JointPtr tireJoint_;

  boost::mutex lock;

  std::string tireLinkName_;
  std::string baseLinkName_;

  bool alive_;

  double update_rate_;
  double update_period_;
  common::Time last_update_time_;

  physics::LinkPtr tireLink_;
  physics::LinkPtr carLink_;

  ignition::math::Vector3d anchorPose_;

  transport::SubscriberPtr contactSub_;

  bool wheelCollides_;
  bool isRear_;

  double radius_;
  double FNOMIN_;
  double LFZ0_;
  double PDX1_;
  double PDX2_;
  double PDX3_;
  double LGAX_;
  double LMUX_;
  double PCX1_;
  double LCX_;
  double PHX1_;
  double PHX2_;
  double LHX_;
  double PVX1_;
  double PVX2_;
  double LVX_;
  double PKX1_;
  double PKX2_;
  double PKX3_;
  double LKX_;
  double PEX1_;
  double PEX2_;
  double PEX3_;
  double PEX4_;
  double LEX_;
  double KPUMIN_;
  double KPUMAX_;
  double FZMIN_;
  double FZMAX_;

  double QSY1_;
  double QSY2_;
  double QSY3_;
  double QSY4_;
  double LONGVL_;

  // Slip Angle Range
  double ALPMIN_;
  double ALPMAX_;

  // Lateral Coefficients
  double PCY1_;  // Shape factor Cfy for lateral forces
  double PDY1_;  // Lateral friction Muy
  double PDY2_;  // Variation of friction Muy with load
  double PDY3_;  // Variation of friction Muy with squared camber
  double PEY1_;  // Lateral curvature Efy at Fznom
  double PEY2_;  // Variation of curvature Efy with load
  double PEY3_;  // Zero order camber dependency of curvature Efy
  double PEY4_;  // Variation of curvature Efy with camber
  double PKY1_;  // Maximum value of stiffness Kfy/Fznom
  double PKY2_;  // Load at which Kfy reaches maximum value
  double PKY3_;  // Variation of Kfy/Fznom with camber
  double PHY1_;  // Horizontal shift Shy at Fznom
  double PHY2_;  // Variation of shift Shy with load
  double PHY3_;  // Variation of shift Shy with camber
  double PVY1_;  // Vertical shift in Svy/Fz at Fznom
  double PVY2_;  // Variation of shift Svy/Fz with load
  double PVY3_;  // Variation of shift Svy/Fz with camber
  double PVY4_;  // Variation of shift Svy/Fz with camber and load

  // Scaling Coefficients
  double LCY_;
  double LMUY_;
  double LEY_;
  double LKY_;
  double LHY_;
  double LVY_;
  double LGAY_;
  double LYKA_;
  double LVYKA_;
  double LXAL_;
  double LGAZ_;
  double LTR_;
  double LS_;

  // Combined Fy Coefficients
  double RBY1_;
  double RBY2_;
  double RBY3_;
  double RCY1_;
  double REY1_;
  double REY2_;
  double RHY1_;
  double RHY2_;
  double RVY1_;
  double RVY2_;
  double RVY3_;
  double RVY4_;
  double RVY5_;
  double RVY6_;

  // Combined Fx Coefficients
  double RBX1_;
  double RBX2_;
  double RCX1_;
  double REX1_;
  double REX2_;
  double RHX1_;

  // Aligning Torque Coefficients
  double QHZ1_;
  double QHZ2_;
  double QHZ3_;
  double QHZ4_;
  double QDZ1_;
  double QDZ2_;
  double QDZ3_;
  double QDZ4_;
  double QDZ6_;
  double QDZ7_;
  double QDZ8_;
  double QDZ9_;
  double QBZ1_;
  double QBZ2_;
  double QBZ3_;
  double QBZ4_;
  double QBZ5_;
  double QBZ9_;
  double QBZ10_;
  double QCZ1_;
  double QEZ1_;
  double QEZ2_;
  double QEZ3_;
  double SSZ1_;
  double SSZ2_;
  double SSZ3_;
  double SSZ4_;

  // Camber Coefficients
  double CC1_;
  double CC2_;
  double CC3_;
  double CC4_;

  // Stored values for aligning torque
  double Kx_;
  double Ky_;
  double SHy_;
  double SVy_;
  double SVyk_;
  double By_;

  double FZ0T_;
  double CX_;
  double CY_;
  double camber_;
  bool useDynamicCamber_;
};
}

#endif
