#pragma once
#ifndef BAYESNETWORK_ABSTRACTOCTREE_HPP
#define BAYESNETWORK_ABSTRACTOCTREE_HPP

#include "DepthFirstIterator.hpp"

namespace Processors {
namespace Network {

class AbstractOctree {
public:
  virtual ~AbstractOctree() {};
  virtual bool empty() = 0;
  virtual int getNumberOfPoints() = 0;
  virtual DepthFirstIterator depthBegin() = 0;
  virtual DepthFirstIterator depthEnd() = 0;
  virtual PointXYZSIFT getPoint(unsigned int id) = 0;

};

}
}

#endif //BAYESNETWORK_ABSTRACTOCTREE_HPP
