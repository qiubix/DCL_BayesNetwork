/*!
 * \file NetworkTester.hpp
 * \brief
 */

#ifndef NETWORK_TESTER_HPP_
#define NETWORK_TESTER_HPP_

#define CV_NO_BACKWARD_COMPATIBILITY


#include "Component_Aux.hpp"
#include "Component.hpp"
#include "DataStream.hpp"
#include "Property.hpp"
#include "EventHandler2.hpp"
#include "../../../lib/SMILE/smile.h"
#include <opencv2/core/core.hpp>

namespace Processors {
namespace Network {

/*!
 * \class NetworkTester
 * \brief Component for testing purposes
 * \author Karol Katerżawa
 */
class NetworkTester: public Base::Component
{
public:
  /*!
   * Constructor.
   */
  NetworkTester(const std::string & name = "NetworkTester");

  /*!
   * Destructor
   */
  virtual ~NetworkTester();

  /*!
   * Prepare data streams and handlers
   */
  void prepareInterface();

protected:
  ///Input data streams
  Base::DataStreamIn<DSL_network> in_network;

  //Output data streams
  Base::DataStreamOut<int> out_result;

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
  Base::EventHandler <NetworkTester> h_onNetwork;

  /*!
   * Event handler function.
   */
  void testNetwork();

private:
  DSL_network network;
  int result;
  
  std::string getNodeName(int nodeId);
  int findFeatureNode(int nodeId);
  void displayProbability(std::string nodeName);
  void observeNode(int nodeId);
};

}//: namespace Network
}//: namespace Processors


/*
 * Register processor component.
 */
REGISTER_COMPONENT("NetworkTester", Processors::Network::NetworkTester)

#endif /* NETWORK_TESTER_HPP_ */

