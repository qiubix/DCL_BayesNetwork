/*!
 * \file MockMultiplicityMapper.cpp
 * \brief
 */

#include <memory>
#include <string>
#include <sstream>
#include <iostream>

#include "MockMultiplicityMapper.hpp"

#include "Logger.hpp"
#include "Common/Timer.hpp"

#include <boost/thread.hpp>
#include <boost/date_time/posix_time/posix_time.hpp>

namespace Processors {
namespace Network {

MockMultiplicityMapper::MockMultiplicityMapper(const std::string & name) : Base::Component(name)
{
  LOG(LTRACE)<<"Hello MockMultiplicityMapper\n";
}

MockMultiplicityMapper::~MockMultiplicityMapper()
{
  LOG(LTRACE)<<"Good bye MockMultiplicityMapper\n";
}

void MockMultiplicityMapper::prepareInterface()
{
  LOG(LTRACE) << "MockMultiplicityMapper::prepareInterface\n";

  h_onJointMultiplicity.setup(this, &MockMultiplicityMapper::onJointMultiplicity);
  registerHandler("onJointMultiplicity", &h_onJointMultiplicity);

  registerStream("in_jointCloud", &in_jointCloud);
  addDependency("onJointMultiplicity", &in_jointCloud);

  registerStream("out_jointMultiplicity", &out_jointMultiplicity);
}

bool MockMultiplicityMapper::onInit()
{
  LOG(LTRACE) << "MockMultiplicityMapper::initialize\n";
  return true;
}

bool MockMultiplicityMapper::onFinish()
{
  LOG(LTRACE) << "MockMultiplicityMapper::finish\n";
  return true;
}

bool MockMultiplicityMapper::onStop()
{
  LOG(LTRACE) << "MockMultiplicityMapper::onStop\n";
  return true;
}

bool MockMultiplicityMapper::onStart()
{
  LOG(LTRACE) << "MockMultiplicityMapper::onStart\n";
  return true;
}

void MockMultiplicityMapper::onJointMultiplicity()
{
  LOG(LDEBUG) << "Extracting multiplicity of the joint cloud";
  pcl::PointCloud<PointXYZSIFT>::Ptr jointCloud = in_jointCloud.read();
  std::vector <int> jointMultiplicity;
  int featureMultiplicity;
  pcl::PointCloud<PointXYZSIFT>::iterator it = jointCloud->begin();
  while (it != jointCloud->end()) {
    featureMultiplicity = it->multiplicity;
    jointMultiplicity.push_back(featureMultiplicity);
    ++it;
  }
  LOG(LINFO) << "Number of joint model's features: " << jointMultiplicity.size();
  out_jointMultiplicity.write(jointMultiplicity);
}

}//: namespace Network
}//: namespace Processors
