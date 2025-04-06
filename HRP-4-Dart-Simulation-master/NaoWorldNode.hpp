#ifndef DART_EXAMPLE_OSG_OSGATLASSIMBICON_ATLASSIMBICONWORLDNODE_HPP_
#define DART_EXAMPLE_OSG_OSGATLASSIMBICON_ATLASSIMBICONWORLDNODE_HPP_

#include <dart/dart.hpp>
#include <dart/utils/utils.hpp>
#include <dart/gui/osg/osg.hpp>
#include <dart/gui/gui.hpp>
#include "Controller.hpp"

class NaoWorldNode : public dart::gui::osg::WorldNode
{
public:
  /// Constructor
  NaoWorldNode(dart::simulation::WorldPtr world,
                         dart::dynamics::SkeletonPtr nao);

  // Documentation inherited
  void customPreStep() override;

  void reset();

  void pushForwardAtlas(double force = 500, int frames = 100);
  void pushBackwardAtlas(double force = 500, int frames = 100);
  void pushLeftAtlas(double force = 500, int frames = 100);
  void pushRightAtlas(double force = 500, int frames = 100);

  void switchToNormalStrideWalking();
  void switchToShortStrideWalking();
  void switchToNoControl();

  dart::dynamics::SkeletonPtr getRobot();
  dart::simulation::WorldPtr getWorld();
  std::shared_ptr<Controller> getController();

protected:
  std::shared_ptr<Controller> mController;
  Eigen::Vector3d mExternalForce;
  int mForceDuration;
  dart::simulation::WorldPtr mWorld;
  dart::dynamics::SkeletonPtr mRobot;
};

#endif // DART_EXAMPLE_OSG_OSGATLASSIMBICON_ATLASSIMBICONWORLDNODE_HPP_
