#pragma once
#ifndef OCTREE_ITERATOR_HPP
#define OCTREE_ITERATOR_HPP

#include "OctreeNode.hpp"

namespace Processors {
namespace Network {

class AbstractOctreeIterator
{
public:
  virtual ~AbstractOctreeIterator(){};

  virtual OctreeNode getCurrentNode() = 0;

private:
  /* data */
};

}
}

#endif //OCTREE_ITERATOR_HPP
