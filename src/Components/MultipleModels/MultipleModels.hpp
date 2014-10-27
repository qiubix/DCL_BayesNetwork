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
  virtual ~MultipleModels();

  /*!
   * Prepare data streams and handlers
   */
  void prepareInterface();

protected:

  /// Input data stream
  Base::DataStreamIn< DSL_network > in_network;

  /// Output data stream
  Base::DataStreamOut< std::vector<DSL_network> > out_networks;

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
  Base::EventHandler2 h_onNetwork;

  /*!
   * Event handler function.
   */
  void createGrid();
  void addNewNetwork();

private:
  std::vector<DSL_network> networks;
};

}//: namespace Network
}//: namespace Processors


/*
 * Register processor component.
 */
REGISTER_COMPONENT("MultipleModels", Processors::Network::MultipleModels)

#endif /* MULTIPLE_MODELS_HPP_ */

