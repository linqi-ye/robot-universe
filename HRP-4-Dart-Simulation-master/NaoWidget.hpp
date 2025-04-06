#ifndef DART_EXAMPLE_OSG_OSGATLASSIMBICON_ATLASSIMBICONWIDGET_HPP_
#define DART_EXAMPLE_OSG_OSGATLASSIMBICON_ATLASSIMBICONWIDGET_HPP_

#include "dart/gui/osg/ImGuiWidget.hpp"
#include "dart/gui/osg/ImGuiViewer.hpp"
#include <dart/dart.hpp>

class NaoWorldNode;

class NaoWidget : public dart::gui::osg::ImGuiWidget
{
public:

  /// Constructor
  NaoWidget(dart::gui::osg::ImGuiViewer* viewer,
		NaoWorldNode* node,
		dart::simulation::WorldPtr world);

  // Documentation inherited
  void render() override;\

protected:

  void setComTargetHeight(double);
  void setReferenceVelocityX(double);
  void setReferenceVelocityY(double);
  void setReferenceVelocityOmega(double);

  dart::gui::osg::ImGuiViewer* mViewer;

  NaoWorldNode* mNode;
  dart::simulation::WorldPtr mWorld;

  float mGuiComHeight;
  float mGuiReferenceVelocityX;
  float mGuiReferenceVelocityY;
  float mGuiReferenceVelocityOmega;
  float mGuiVelGain;
  float mGuiZmpGain;

  float mGuiComRoll;
  float mGuiComPitch;
  float mGuiComYaw;
  float mGuiComX;
  float mGuiComY;
  float mGuiComZ;

  float mComHeight;

  bool mGuiHeadlights;

  /// Control mode value for GUI
  int mGuiControlMode;

  int mGuiBeheavior;

  /// Actual control mode
  ///   - 0: No control
  ///   - 1: Short-stride walking control
  ///   - 1: Normal-stride walking control
  int mControlMode;
  int mBeheavior;

};

#endif // DART_EXAMPLE_OSG_OSGATLASSIMBICON_ATLASSIMBICONWIDGET_HPP_
