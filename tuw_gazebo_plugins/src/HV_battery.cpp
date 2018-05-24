#include <tuw_gazebo_plugins/HV_battery.h>
#include <tuw_gazebo_plugins/gazebo_ros_utils.h>
#include <tuw_vehicle_msgs/battery_state.h>

namespace gazebo {
HVBattery::HVBattery() {
  totalCapacity_ = 1;
  currentCapacity_ = 0;
  minVoltage_ = 0;
  maxVoltage_ = 1;
  voltageRange_ = 1;
  cellCount_ = 1;
  current_ = 0;
  batteryInnerResistance_ = 0;
  voltageError_ = true;
  temperature_ = 40;
}

void HVBattery::initFromParameters(GazeboRosPtr gazebo_ros_) {
  int cellCount;
  double startVoltage, maxVoltage, minVoltage, cellCapacity,
      batteryInnerResistance;
  gazebo_ros_->getParameter<int>(cellCount, "batteryCellCount", 40);
  gazebo_ros_->getParameter<double>(startVoltage, "batteryStartVoltage", 4.2);
  gazebo_ros_->getParameter<double>(maxVoltage, "batteryMaxVoltage", 4.2);
  gazebo_ros_->getParameter<double>(minVoltage, "batteryMinVoltage", 3.2);
  gazebo_ros_->getParameter<double>(cellCapacity, "batteryCellCapacity", 40.0);
  gazebo_ros_->getParameter<double>(batteryInnerResistance,
                                    "batteryInnerResistance", 0.3);
  this->init(cellCount, startVoltage, maxVoltage, minVoltage, cellCapacity,
             batteryInnerResistance);
}

void HVBattery::init(int cellCount, double startVoltage, double maxVoltage,
                     double minVoltage, double cellCapacity,
                     double batteryInnerResistance) {
  totalCapacity_ = cellCapacity;
  minVoltage_ = minVoltage;
  maxVoltage_ = maxVoltage;
  voltageRange_ = maxVoltage - minVoltage;
  currentCapacity_ =
      ((voltageRange_ - (maxVoltage - startVoltage)) / (voltageRange_)) *
      totalCapacity_;
  cellCount_ = cellCount;
  voltageError_ = startVoltage > minVoltage_;
  batteryInnerResistance_ = batteryInnerResistance;
  this->CalcVoltage();
}

double HVBattery::GetCurrentVoltage() { return currentVoltage_; }

double HVBattery::CalcVoltage() {
  double voltagePerCell =
      minVoltage_ + (currentCapacity_ / totalCapacity_) * (voltageRange_);
  currentVoltage_ = voltagePerCell * cellCount_;
  voltageError_ = voltagePerCell > minVoltage_;
  double currentDrop = batteryInnerResistance_ * current_;
  currentVoltage_ -= currentDrop;
  return currentVoltage_;
}

void HVBattery::AddDischargeCurrent(double current) { current_ += current; }

void HVBattery::AddDischargeWithMotor(double force, double rad) {
  // force same direction as rad => positive discharge current
  // different directions => negative discharge current = recuperation
  current_ += (force * rad) / currentVoltage_;
}

void HVBattery::Update(double elapsedTime) {
  // apply current for one circuit
  currentCapacity_ -= current_ * (elapsedTime / 3600);
  CalcVoltage();
  lastAppliedCurrent_ = current_;
  current_ = 0;
}

bool HVBattery::HasVoltageError() { return voltageError_; }

double HVBattery::GetCurrent() { return lastAppliedCurrent_; }

double HVBattery::GetTemperature() { return temperature_; }

void HVBattery::SetBatteryState(tuw::ros_msgs::BatteryState &msg) {
  double voltagePerCell =
      minVoltage_ + (currentCapacity_ / totalCapacity_) * (voltageRange_);

  msg.cell_voltages.resize(cellCount_);
  msg.cell_temperatures.resize(cellCount_);
  for (size_t i = 0; i < cellCount_; i++) {
    msg.cell_voltages[i] = voltagePerCell;
    msg.cell_temperatures[i] = temperature_;
  }
  msg.current = lastAppliedCurrent_;
}

HVBattery::~HVBattery() {}
}
