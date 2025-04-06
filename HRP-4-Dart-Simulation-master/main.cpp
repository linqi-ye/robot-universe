#include <dart/dart.hpp>
#include <dart/gui/osg/osg.hpp>
#include <dart/utils/utils.hpp>
#include <dart/utils/urdf/urdf.hpp>
#include <dart/gui/gui.hpp>
#include "NaoWorldNode.hpp"
#include "NaoEventHandler.hpp"
#include "NaoWidget.hpp"
#include <dart/common/LocalResourceRetriever.hpp>



int main(int argc, char* argv[])
{
  // Create a world
  dart::simulation::WorldPtr world(new dart::simulation::World);

  // Load ground and Nao robot and add them to the world
  dart::utils::DartLoader urdfLoader;
  auto ground = urdfLoader.parseSkeleton(//"dart://sample/sdf/atlas/ground.urdf");
	"/home/filippo/DART codes/DarwinOp/ground.urdf");
  auto nao = urdfLoader.parseSkeleton(//dart::utils::SdfParser::readSkeleton(
        "/home/filippo/DART codes/DarwinOp/urdf/hrp4.urdf"); 
  world->addSkeleton(ground);
  world->addSkeleton(nao);

  // set joint actuator type
  std::vector<size_t> indices;
  std::vector<dart::dynamics::Joint::ActuatorType> types;
  double sufficient_force = 100;// 20;

  for (size_t i = 0; i < nao->getNumJoints(); i++) {
	  size_t  dim   = nao->getJoint(i)->getNumDofs();
	  if(dim==6){
		  nao->getJoint(i)->setActuatorType(dart::dynamics::Joint::PASSIVE);
	  }
	  if(dim==1){
		  nao->getJoint(i)->setActuatorType(dart::dynamics::Joint::SERVO);
		  nao->getJoint(i)->setForceUpperLimit(0, sufficient_force);
		  nao->getJoint(i)->setForceLowerLimit(0, -sufficient_force);
		  nao->getJoint(i)->setPositionLimitEnforced(true);
	  }
  }


  // Set gravity of the world
  world->setGravity(Eigen::Vector3d(0.0, 0.0, -9.81));
  world->setTimeStep(1.0/100.0);


  // Wrap a WorldNode around it
  osg::ref_ptr<NaoWorldNode> node
      = new NaoWorldNode(world, nao);
  node->setNumStepsPerCycle(1);

  // Create a Viewer and set it up with the WorldNode
  dart::gui::osg::ImGuiViewer viewer;
  viewer.addWorldNode(node);

  // Add control widget for nao
  viewer.getImGuiHandler()->addWidget(
        std::make_shared<NaoWidget>(&viewer, node.get(), world));

  // Pass in the custom event handler
  viewer.addEventHandler(new NaoEventHandler(node));

  // Set the dimensions for the window
  viewer.setUpViewInWindow(0, 0, 1280, 960);

  // Set the window name
  viewer.realize();
  osgViewer::Viewer::Windows windows;
  viewer.getWindows(windows);
  windows.front()->setWindowName("HRP-4 MPC");

  // Adjust the viewpoint of the Viewer
/*
  viewer.getCameraManipulator()->setHomePosition(
        ::osg::Vec3d( 5.14,  3.28, 6.28)*0.7,
        ::osg::Vec3d( 0.50,  -1.00, 0.00),
        ::osg::Vec3d( 0.00,  0.00, 0.1));/**/
  viewer.getCameraManipulator()->setHomePosition(
        ::osg::Vec3d( 5.14,  3.28, 6.28)*0.7,
        ::osg::Vec3d( 0.50,  -1.00, 0.00),
        ::osg::Vec3d( 0.00,  0.00, 0.1));

  // We need to re-dirty the CameraManipulator by passing it into the viewer
  // again, so that the viewer knows to update its HomePosition setting
  viewer.setCameraManipulator(viewer.getCameraManipulator());

  // Begin running the application loop
  viewer.run();
}
