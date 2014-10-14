/*!
 * \file MultipleModels.hpp
 * \brief
 */

#ifndef MULTIPLE_MODELS_HPP_
#define MULTIPLE_MODELS_HPP_

#define CV_NO_BACKWARD_COMPATIBILITY


#include "Component_Aux.hpp"
#include "Component.hpp"
#include "DataStream.hpp"
#include "Property.hpp"
#include "EventHandler2.hpp"
#include "OctreeContainers.hpp"

#include "../../../lib/SMILE/smile.h"
#include <opencv2/core/core.hpp>

#include <Types/PointXYZSIFT.hpp>


namespace Processors {
namespace Network {

/*!
 * \class CreateNetwork
 * \brief Class used to build Bayes network
 * \author Karol Katerżawa
 */
class MultipleModels: public Base::Component
{
public:
  /*!
   * Constructor.
   */
  MultipleModels(const std::string & name = "CreateNetwork");

  /*!
   * Destructor
   */
  virtual ~MultpleModels();

  /*!
   * Prepare data streams and handlers
   */
  void prepareInterface();

protected:

  /// Input data stream
  Base::DataStreamIn< std::vector<DSL_network> > in_networks;

  /// Output data stream
  Base::DataStreamOut< std::vector<double> > out_probabilities;

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

  /// Event handlers
  Base::EventHandler2 h_onNetworks;

  /*!
   * Event handler function.
   */
  void createGrid();

private:

};

}//: namespace Network
}//: namespace Processors


/*
 * Register processor component.
 */
REGISTER_COMPONENT("MultpleModels", Processors::Network::MultpleModels)

#endif /* MULTIPLE_MODELS_HPP_ */

