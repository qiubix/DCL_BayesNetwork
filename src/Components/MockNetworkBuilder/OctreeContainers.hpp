#ifndef OCTREE_CONTAINERS_HPP
#define OCTREE_CONTAINERS_HPP

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/octree/octree.h>
#include <pcl/octree/octree_impl.h>

using namespace pcl::octree;
namespace Processors {
namespace Network {
  
class OctreeContainerEmptyWithId : public OctreeContainerEmpty 
{
public:
    OctreeContainerEmptyWithId() : OctreeContainerEmpty()
    {
        this->nodeId = -1;
        this->parentId = -1;
    }
    virtual ~OctreeContainerEmptyWithId() {}

    int getNodeId()
    {
        return nodeId;
    }
    int getParentId()
    {
       	return parentId;
    }
    void setNodeId(int nodeId)
    {
        this->nodeId = nodeId;
    }
    void setParentId(int parentId) 
    {
        this->parentId = parentId;
    }

private:
    int nodeId;
    int parentId;
};

class OctreeContainerPointIndicesWithId : public OctreeContainerPointIndices 
{
public:
    OctreeContainerPointIndicesWithId() : OctreeContainerPointIndices() 
    {
        this->nodeId = -1;
        this->parentId = -1;
    }
    virtual ~OctreeContainerPointIndicesWithId() {}

    int getNodeId()
    {
        return nodeId;
    }
    int getParentId()
    {
       	return parentId;
    }
    void setNodeId(int nodeId)
    {
        this->nodeId = nodeId;
    }
    void setParentId(int parentId) 
    {
        this->parentId = parentId;
    }

private:
    int nodeId;
    int parentId;
};

}//: namespace Network   
}//: namespace Processors


#endif //OCTREE_CONTAINERS_HPP
