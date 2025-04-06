#ifndef DART_EXAMPLE_OSG_OSGATLASSIMBICON_ATLASSIMBICONEVENTHANDLER_HPP_
#define DART_EXAMPLE_OSG_OSGATLASSIMBICON_ATLASSIMBICONEVENTHANDLER_HPP_

#include <dart/dart.hpp>
#include <dart/utils/utils.hpp>
#include <dart/gui/osg/osg.hpp>

#include "NaoWorldNode.hpp"

class NaoEventHandler : public osgGA::GUIEventHandler
{
public:

  NaoEventHandler(NaoWorldNode* node);

  bool handle(const osgGA::GUIEventAdapter& ea,
              osgGA::GUIActionAdapter&) override;

protected:

  NaoWorldNode* mNode;

};

#endif // DART_EXAMPLE_OSG_OSGATLASSIMBICON_ATLASSIMBICONEVENTHANDLER_HPP_
