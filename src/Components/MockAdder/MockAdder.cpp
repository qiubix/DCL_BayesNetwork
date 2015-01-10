/*!
 * \file
 * \brief
 * \author Karol Katerżawa
 */

#include <memory>
#include <string>

#include "MockAdder.hpp"
#include "Common/Logger.hpp"

#include <boost/bind.hpp>

//#include <fstream>
#include <pcl/point_representation.h>

#include "pcl/impl/instantiate.hpp"
#include "pcl/search/kdtree.h"
#include "pcl/search/impl/kdtree.hpp"

namespace Processors {
namespace Network {

MockAdder::MockAdder(const std::string & name) :
		Base::Component(name)  {
  nextId = 0;
}

MockAdder::~MockAdder() {
}

void MockAdder::prepareInterface() {
  // Register data streams, events and event handlers HERE!
  //registerStream("in_descriptors", &in_descriptors);
  //registerStream("out_descriptors", &out_descriptors);
  //registerStream("in_cloud", &in_cloud);
  registerStream("in_models", &in_models);
  registerStream("out_cloud", &out_cloud);
  registerStream("out_multiplicityOfModels", &out_multiplicityOfModels);
  // Register handlers
  h_add.setup(boost::bind(&MockAdder::add, this));
  registerHandler("add", &h_add);
  //	addDependency("add", &in_cloud);
  addDependency("add", &in_models);
}

bool MockAdder::onInit() {
  jointCloud = pcl::PointCloud<PointXYZSIFT>::Ptr (new pcl::PointCloud<PointXYZSIFT>());
  return true;
}

bool MockAdder::onFinish() {
  return true;
}

bool MockAdder::onStop() {
  //std::fstream plik;
  //plik.open( "/home/mlaszkow/test.txt", std::ios::out );
  //plik<<"Deskryptory:"<<endl;
  //for (int i = 0; i< descriptors.size(); i++)
  //plik<<descriptors[i]<<endl;
  //plik.close();
  return true;
}

bool MockAdder::onStart() {
  return true;
}

void MockAdder::add() {
  LOG(LDEBUG) << "================= MockAdder: adding models to joint cloud =================";

  models = in_models.read();
  std::vector <std::map<int,int> > modelsMultiplicity;
  LOG(LDEBUG) << "Number of models: " << models.size();
  for (unsigned n=0; n<models.size(); ++n) {

    std::map<int,int> modelMultiplicity;
    pcl::PointCloud<PointXYZSIFT>::Ptr modelCloud = dynamic_cast<SIFTObjectModel*>(models.at(n))->cloud_xyzsift;
    LOG(LDEBUG) << "Model no " << n << ": model's cloud size = " << modelCloud->size();

    if (jointCloud->empty()){
      LOG(LDEBUG) << "Writing new cloud to empty joint cloud. Size: " << modelCloud->size();
      jointCloud = modelCloud;
      for (unsigned k=0; k<modelCloud->size(); ++k) {
        std::pair<int,int> nextMultiplicity = std::make_pair<int,int>(k, modelCloud->at(k).multiplicity);
        modelMultiplicity.insert(nextMultiplicity);
        jointCloud->at(k).pointId = k;
        modelCloud->at(k).pointId = k;
      }
      nextId = modelCloud->size();
      LOG(LDEBUG) << "number of model's features: " << modelMultiplicity.size();
      modelsMultiplicity.push_back(modelMultiplicity);
      cloudModels.push_back(modelCloud);
      out_cloud.write(jointCloud);

      LOG(LWARNING) << "---- Model number: " << n;
      LOG(LWARNING) << "---- Model size: " << modelCloud->size();
      for (unsigned k=0; k<modelCloud->size(); ++k) {
        LOG(LWARNING) << "Feature id: " << modelCloud->at(k).pointId;
      }

      continue;
    } //: empty jointCloud

  } //: for models

  LOG(LDEBUG) << "Added all models to joint cloud. Joint cloud size: " << jointCloud->size();
  out_cloud.write(jointCloud);

  LOG(LDEBUG) << "Writing multiplicity vectors of models merged to cloud. Number of vectors: " << modelsMultiplicity.size();
  out_multiplicityOfModels.write(modelsMultiplicity);

} //:add()

} //: namespace Network
} //: namespace Processors
