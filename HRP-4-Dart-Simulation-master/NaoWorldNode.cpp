#include "NaoWorldNode.hpp"

//==============================================================================
NaoWorldNode::NaoWorldNode(
    const dart::simulation::WorldPtr world,
    const dart::dynamics::SkeletonPtr nao)
  : dart::gui::osg::WorldNode(world),
    mExternalForce(Eigen::Vector3d::Zero()),
    mForceDuration(0.0)
{
  assert(world);
  assert(nao);

  mWorld = world;
  mRobot = nao;

  mController.reset(new Controller(nao, world));
  mController->setInitialConfiguration();
}

//==============================================================================
void NaoWorldNode::customPreStep()
{
  mController->update();
}

//==============================================================================
void NaoWorldNode::reset()
{
  //mExternalForce.setZero();
  mController.reset(new Controller(mRobot, mWorld));
  mController->setInitialConfiguration();

}

//==============================================================================
void NaoWorldNode::pushForwardAtlas(double force, int frames)
{
  mExternalForce.x() = force;
  mForceDuration = frames;/**/
  //mRobot->setForce(Eigen::Vector3d(100,0,0));
}

//==============================================================================
void NaoWorldNode::pushBackwardAtlas(double force, int frames)
{
  mExternalForce.x() = -force;
  mForceDuration = frames;
}

//==============================================================================
void NaoWorldNode::pushLeftAtlas(double force, int frames)
{
  mExternalForce.z() = force;
  mForceDuration = frames;
}

//==============================================================================
void NaoWorldNode::pushRightAtlas(double force, int frames)
{
  mExternalForce.z() = -force;
  mForceDuration = frames;
}

//==============================================================================
void NaoWorldNode::switchToNormalStrideWalking()
{/*
  mController->changeStateMachine("walking", mWorld->getTime());*/
}

//==============================================================================
void NaoWorldNode::switchToShortStrideWalking()
{/*
  mController->changeStateMachine("running", mWorld->getTime());*/
}

//==============================================================================
void NaoWorldNode::switchToNoControl()
{/*
  mController->changeStateMachine("standing", mWorld->getTime());*/
}

dart::dynamics::SkeletonPtr NaoWorldNode::getRobot() {
	return mRobot;
}

dart::simulation::WorldPtr NaoWorldNode::getWorld() {
	return mWorld;
}

std::shared_ptr<Controller> NaoWorldNode::getController() {
	return mController;
}

