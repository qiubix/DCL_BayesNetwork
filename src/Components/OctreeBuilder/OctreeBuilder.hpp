/*!
 * \file OctreeBuilder.hpp
 * \brief Component for building octree from point cloud
 */

#ifndef OCTREE_BUILDER_HPP_
#define OCTREE_BUILDER_HPP_

#define CV_NO_BACKWARD_COMPATIBILITY


#include "Component_Aux.hpp"
#include "Component.hpp"
#include "DataStream.hpp"
#include "Property.hpp"

#include <Types/PointXYZSIFT.hpp>

namespace Processors {
namespace Network {

/*!
 * \class OctreeBuilder
 * \brief Component for building octree from point cloud
 * \author Karol Katerżawa
 */
class OctreeBuilder: public Base::Component
{
public:
    /*!
     * Constructor.
     */
    OctreeBuilder(const std::string & name = "OctreeBuilder");

    /*!
     * Destructor
     */
    virtual ~OctreeBuilder();

    /*!
     * Prepare data streams and handlers
     */
    void prepareInterface();

protected:

    /// Input data stream
    Base::DataStreamIn< pcl::PointCloud<PointXYZSIFT>::Ptr > in_cloud;

    /// Output data stream
    //Base::DataStreamOut< std::vector<int> > out_jointMultiplicity;

    /*!
     * Connects source to given device.
     */
    bool onInit();

    /*!
     * Disconnect source from device, closes streams, etc.
     */
    bool onFinish();

    /*!
     * Start component
     */
    bool onStart();

    /*!
     * Stop component
     */
    bool onStop();

    /*!
     * Event handler function.
     */
};

}//: namespace Network
}//: namespace Processors


/*
 * Register processor component.
 */
REGISTER_COMPONENT("OctreeBuilder", Processors::Network::OctreeBuilder)

#endif /* OCTREE_BUILDER_HPP_ */

