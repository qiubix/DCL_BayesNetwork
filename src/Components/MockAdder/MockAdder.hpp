/*!
 * \file
 * \brief
 * \author Karol Katerżawa
 */

#ifndef MOCK_ADDER_HPP_
#define MOCK_ADDER_HPP_

#include "Component_Aux.hpp"
#include "Component.hpp"
#include "DataStream.hpp"
#include "Property.hpp"
#include "EventHandler2.hpp"

#include <opencv2/opencv.hpp>
#include <vector>
#include <Types/PointXYZSIFT.hpp>
#include <Types/SIFTObjectModel.hpp>
//#include "Types/Features.hpp"

#include <pcl/registration/correspondence_estimation.h>
#include "pcl/registration/correspondence_rejection_sample_consensus.h"

namespace Processors {
namespace Network {

/*!
 * \class MockAdder
 * \brief MockAdder processor class.
 *
 * MockAdder processor.
 */
class MockAdder: public Base::Component {
public:
  /*!
   * Constructor.
   */
  MockAdder(const std::string & name = "MockAdder");

  /*!
   * Destructor
   */
  virtual ~MockAdder();

  /*!
   * Prepare components interface (register streams and handlers).
   * At this point, all properties are already initialized and loaded to
   * values set in config file.
   */
  void prepareInterface();

protected:

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

  bool matIsEqual(const cv::Mat mat1, const cv::Mat mat2);


  // Input data streams
//		Base::DataStreamIn<pcl::PointCloud<PointXYZSIFT>::Ptr> in_cloud;
  Base::DataStreamIn<std::vector<AbstractObject*> > in_models;

  // Output data streams
  Base::DataStreamOut<pcl::PointCloud<PointXYZSIFT>::Ptr> out_cloud;
  Base::DataStreamOut< std::vector<std::map<int,int> > > out_multiplicityOfModels;
  //Base::DataStreamOut<vector<vector<int> > > out_descriptors;
  //Base::DataStreamOut<vector<int> > out_multiplicity;

  // Handlers
  Base::EventHandler2 h_add;

  // Handlers
  void add();

  pcl::PointCloud<PointXYZSIFT>::Ptr jointCloud;
  std::vector <pcl::PointCloud<PointXYZSIFT>::Ptr> cloudModels;
  std::vector <AbstractObject*> models;

  //vector<vector<int> > descriptors;
  //vector<int> multiplicity;
private:

  unsigned nextId;
};

} //: namespace Network
} //: namespace Processors

/*
 * Register processor component.
 */
REGISTER_COMPONENT("MockAdder", Processors::Network::MockAdder)

#endif /* MOCK_ADDER_HPP_ */
