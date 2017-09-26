#ifndef HV_BATTERY_H
#define HV_BATTERY_H

#include <tuw_gazebo_plugins/gazebo_ros_utils.h>
#include <tuw_vehicle_msgs/battery_state.h>
namespace gazebo {
class HVBattery {
 public:
  HVBattery();
  ~HVBattery();
  void init(int cellCount, double startVoltage, double maxVoltage,
            double minVoltage, double cellCapacity,
            double batteryInnerResistance);
  void initFromParameters(GazeboRosPtr gazebo_ros_);

  double GetCurrentVoltage();
  bool HasVoltageError();
  void AddDischargeCurrent(double current);
  void AddDischargeWithMotor(double force, double rad);
  void Update(double elapsedTime);
  double GetCurrent();
  double GetTemperature();
  void SetBatteryState(tuw::ros_msgs::BatteryState &msg);

 private:
  double CalcVoltage();

  int cellCount_;
  double currentVoltage_;
  double maxVoltage_;
  double minVoltage_;
  double currentCapacity_;
  double totalCapacity_;
  double voltageRange_;
  bool voltageError_;
  double current_;
  double lastAppliedCurrent_;
  double batteryInnerResistance_;
  double temperature_;
};
}

#endif
