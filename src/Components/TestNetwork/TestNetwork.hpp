/*!
 * \file TestNetwork.hpp
 * \brief
 */

#ifndef TEST_NETWORK_HPP_
#define TEST_NETWORK_HPP_

#define CV_NO_BACKWARD_COMPATIBILITY


#include "Component_Aux.hpp"
#include "Component.hpp"
#include "DataStream.hpp"
#include "Property.hpp"

namespace Processors {
namespace Network {

/*!
 * \class TestNetwork
 * \brief Component for testing purposes
 * \author Karol Katerżawa
 */
class TestNetwork: public Base::Component
{
public:
  /*!
   * Constructor.
   */
  TestNetwork(const std::string & name = "TestNetwork");

  /*!
   * Destructor
   */
  virtual ~TestNetwork();

  /*!
   * Prepare data streams and handlers
   */
  void prepareInterface();

protected:
  ///Input data streams

  //Output data streams

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

  /*!
   * Event handler function.
   */

private:
};

}//: namespace Network
}//: namespace Processors


/*
 * Register processor component.
 */
REGISTER_COMPONENT("TestNetwork", Processors::Network::TestNetwork)

#endif /* TEST_NETWORK_HPP_ */

