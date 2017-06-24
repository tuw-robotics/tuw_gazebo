#ifndef TUW_GAZEBO_PLUGINS_MAP_COMMON_H
#define TUW_GAZEBO_PLUGINS_MAP_COMMON_H

#include <gazebo/common/common.hh>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <string>
#include <vector>

namespace gazebo {

typedef struct {
  double distanceMin;

  double distanceMax;

  double fovHorizontalDeg;

  double fovVerticalDeg;

} DetectionConfig;

const double coneColorBlue = 0;
const double coneColorYellow = 1;
const double coneColorRed = 2;
const double coneColorUnknown = -1;

double getConeColorShapeVariable(physics::ModelPtr model);

bool isInRangeOf(const ignition::math::Vector3d &conePos,
                 physics::ModelPtr model, const DetectionConfig &cdConfig);

std::vector<physics::ModelPtr> getConesInWorld(const physics::WorldPtr &world);

std::vector<physics::ModelPtr> getConesInWorldSeenBy(
    const physics::WorldPtr &world, const physics::ModelPtr &robot,
    const DetectionConfig &cdConfig);
}

#endif
