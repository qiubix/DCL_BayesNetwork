/*!
 * \file
 * \brief
 * \author Michał Laszkowski, Karol Katerżawa
 */

#include <memory>
#include <string>

#include "SIFTAdder.hpp"
#include "Common/Logger.hpp"

#include <boost/bind.hpp>

//#include <fstream>
#include <pcl/point_representation.h>

#include "pcl/impl/instantiate.hpp"
#include "pcl/search/kdtree.h"
#include "pcl/search/impl/kdtree.hpp"

namespace Processors {
namespace Network {

class SIFTFeatureRepresentation: public pcl::DefaultFeatureRepresentation <PointXYZSIFT> //could possibly be pcl::PointRepresentation<...> ??
{
  using pcl::PointRepresentation<PointXYZSIFT>::nr_dimensions_;
  public:
  SIFTFeatureRepresentation ()
  {
    // Define the number of dimensions
    nr_dimensions_ = 128 ;
    trivial_ = false ;
  }

  // Override the copyToFloatArray method to define our feature vector
  virtual void copyToFloatArray (const PointXYZSIFT &p, float * out) const
  {
    //This representation is only for determining correspondences (not for use in Kd-tree for example - so use only SIFT part of the point
    for (register int i = 0; i < 128 ; i++)
      out[i] = p.descriptor[i];//p.descriptor.at<float>(0, i) ;
    //std::cout << "SIFTFeatureRepresentation:copyToFloatArray()" << std::endl ;
  }
};

SIFTAdder::SIFTAdder(const std::string & name) :
		Base::Component(name)  {
  nextId = 0;
}

SIFTAdder::~SIFTAdder() {
}

void SIFTAdder::prepareInterface() {
  // Register data streams, events and event handlers HERE!
  //registerStream("in_descriptors", &in_descriptors);
  //registerStream("out_descriptors", &out_descriptors);
  //registerStream("in_cloud", &in_cloud);
  registerStream("in_models", &in_models);
  registerStream("out_cloud", &out_cloud);
  registerStream("out_multiplicityOfModels", &out_multiplicityOfModels);
  // Register handlers
  h_add.setup(boost::bind(&SIFTAdder::add, this));
  registerHandler("add", &h_add);
  //	addDependency("add", &in_cloud);
  addDependency("add", &in_models);
}

bool SIFTAdder::onInit() {
  jointCloud = pcl::PointCloud<PointXYZSIFT>::Ptr (new pcl::PointCloud<PointXYZSIFT>());
  return true;
}

bool SIFTAdder::onFinish() {
  return true;
}

bool SIFTAdder::onStop() {
  //std::fstream plik;
  //plik.open( "/home/mlaszkow/test.txt", std::ios::out );
  //plik<<"Deskryptory:"<<endl;
  //for (int i = 0; i< descriptors.size(); i++)
  //plik<<descriptors[i]<<endl;
  //plik.close();
  return true;
}

bool SIFTAdder::onStart() {
  return true;
}

//TODO: refactor
void SIFTAdder::add() {
  LOG(LDEBUG) << "================= SIFTAdder: adding models to joint cloud =================";

  models = in_models.read();
  std::vector <std::map<int,int> > modelsMultiplicity;
  LOG(LDEBUG) << "Number of models: " << models.size();
  for (unsigned n=0; n<models.size(); ++n) {

    //TODO: handle errors
    LOG(LDEBUG) << "inside for loop " << n << "model name: " << models.at(n)->name;
    std::map<int,int> modelMultiplicity;
    pcl::PointCloud<PointXYZSIFT>::Ptr modelCloud;
    LOG(LDEBUG) << "before cast";
    SIFTObjectModel* som = NULL;
    som = dynamic_cast<SIFTObjectModel*>(models.at(n));
    LOG(LDEBUG) << "after cast";
    if(som == NULL)
      LOG(LERROR) << "som jest nullem";
    modelCloud = som->cloud_xyzsift;
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

    LOG(LDEBUG) << "Joint cloud size before merge: " << jointCloud->size();

    pcl::CorrespondencesPtr correspondences(new pcl::Correspondences()) ;
    pcl::registration::CorrespondenceEstimation<PointXYZSIFT, PointXYZSIFT> correst ;

    //SIFTFeatureRepresentation point_representation ;
    //correst.setPointRepresentation (point_representation.makeShared()); //NEVER do like this, makeShared will return DefaultFeatureRepresentation<PointDefault>!
    SIFTFeatureRepresentation::Ptr point_representation(new SIFTFeatureRepresentation()) ;
    correst.setPointRepresentation(point_representation) ;
    correst.setInputSource(modelCloud) ;
    correst.setInputTarget(jointCloud) ;
    correst.determineReciprocalCorrespondences(*correspondences) ;

    LOG(LDEBUG) << "Correspondences determined " << correspondences -> size();

    //TODO: FIXME: Something's wrong with removing wrong correspondences. It matches the same exact points
    if ( correspondences -> size() > 12 ) {
      //ransac znalezienie blednych dopasowan
      pcl::Correspondences inliers ;
      pcl::registration::CorrespondenceRejectorSampleConsensus<PointXYZSIFT> sac ;
      sac.setInputSource(modelCloud) ;
      sac.setInputTarget(jointCloud) ;
      sac.setInlierThreshold(0.001f) ;
      sac.setMaximumIterations(2000) ;
      sac.setInputCorrespondences(correspondences) ;
      sac.getCorrespondences(inliers);

      LOG(LDEBUG) << "Wrong matches found";

      //usuniecie blednych dopasowan
      pcl::Correspondences::iterator iter_inliers = inliers.begin();
      while(iter_inliers!=inliers.end()){
        pcl::Correspondences::iterator iter_correspondences = correspondences->begin();
        while(iter_correspondences!=correspondences->end()){
          if(iter_correspondences->index_query == iter_inliers->index_query){
            iter_correspondences= correspondences->erase(iter_correspondences);
            break;
          }
          else
            ++iter_correspondences;
        }
        ++iter_inliers;
      }
    } //: if more than 4 correspondences

    LOG(LINFO) << "Number of reciprocal correspondences: " << correspondences->size() << " out of " << modelCloud->size() << " keypoints";// << std::endl ;

    pcl::PointCloud<PointXYZSIFT>::Ptr cloudPartToJoin = modelCloud;
    cloudModels.push_back(modelCloud);

    //zliczanie krotnosci
    if(!appendMultiplicity(correspondences, modelCloud, cloudPartToJoin))
      continue;

    //usuniecie punktow
    pcl::PointCloud<PointXYZSIFT>::iterator pt_iter = cloudPartToJoin->begin();
    while(pt_iter!=cloudPartToJoin->end()){
      if(pt_iter->multiplicity==-1){
        pt_iter = cloudPartToJoin->erase(pt_iter);
      }
      else{
        ++pt_iter;
      }
    }

    LOG(LDEBUG) << "Reduced next cloud size: " << cloudPartToJoin->size();
    if (cloudPartToJoin->empty()) {
      LOG(LDEBUG) << "number of model's features: " << modelMultiplicity.size();
      modelsMultiplicity.push_back(modelMultiplicity);
      continue;
    }
    for (unsigned k=0; k<cloudPartToJoin->size(); ++k) {
      std::pair<int,int> nextMultiplicity = std::make_pair<int,int>(jointCloud->size()+k, cloudPartToJoin->at(k).multiplicity);
      modelMultiplicity.insert(nextMultiplicity);
      cloudPartToJoin->at(k).pointId = nextId;
      ++nextId;
    }

    *jointCloud = *jointCloud + *cloudPartToJoin;
    LOG(LDEBUG) << "New joint cloud size: " << jointCloud->size();
    LOG(LDEBUG) << "number of model's features: " << modelMultiplicity.size();
    modelsMultiplicity.push_back(modelMultiplicity);
    //		modelMultiplicity.clear();
    LOG(LWARNING) << "---- Model number: " << n;
    LOG(LWARNING) << "---- Model size: " << modelCloud->size();
    for (unsigned k=0; k<modelCloud->size(); ++k) {
      LOG(LWARNING) << "Feature id: " << modelCloud->at(k).pointId;
    }

  } //: for models

  LOG(LDEBUG) << "Added all models to joint cloud. Joint cloud size: " << jointCloud->size();
  out_cloud.write(jointCloud);

  LOG(LWARNING) << "------ Joint cloud feature ids: ------";
  for (unsigned l=0; l<jointCloud->size(); ++l) {
    LOG(LWARNING) << "Feature id: " << jointCloud->at(l).pointId;
  }
  LOG(LDEBUG) << "Writing multiplicity vectors of models merged to cloud. Number of vectors: " << modelsMultiplicity.size();
  out_multiplicityOfModels.write(modelsMultiplicity);

} //:add()

bool SIFTAdder::appendMultiplicity(pcl::CorrespondencesPtr correspondences, pcl::PointCloud<PointXYZSIFT>::Ptr modelCloud, pcl::PointCloud<PointXYZSIFT>::Ptr cloudPartToJoin) {
  for(int i = 0; i< correspondences->size();i++){
    if (correspondences->at(i).index_query >=modelCloud->size() || correspondences->at(i).index_match >=jointCloud->size()){
      return false;
    }
    jointCloud->at(correspondences->at(i).index_match).multiplicity += modelCloud->at(correspondences->at(i).index_query).multiplicity;
    //modelMultiplicity.insert(std::make_pair<int,int>(correspondences->at(i).index_match, modelCloud->at(correspondences->at(i).index_query).multiplicity));
    cloudPartToJoin->at(correspondences->at(i).index_query).multiplicity=-1; //do usuniecia punkt w nowej chmurze, ktory juz jest zarejestrowany w polaczonej chmurze
    unsigned nextPointId = jointCloud->at(correspondences->at(i).index_match).pointId;
    modelCloud->at(correspondences->at(i).index_query).pointId = nextPointId;
  }
  return true;
}

} //: namespace Network
} //: namespace Processors
