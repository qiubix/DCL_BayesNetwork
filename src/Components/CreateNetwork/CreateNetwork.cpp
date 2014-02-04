/*!
 * \file CreateNetwork.cpp
 * \brief
 */

#include <memory>
#include <string>
#include <sstream>
#include <iostream>

#include "CreateNetwork.hpp"

#include "Logger.hpp"
#include "Common/Timer.hpp"

#include <boost/thread.hpp>
#include <boost/date_time/posix_time/posix_time.hpp>

namespace Processors {
namespace Network {

CreateNetwork::CreateNetwork(const std::string & name) : Base::Component(name)
{
    LOG(LTRACE)<<"Hello CreateNetwork\n";
}

CreateNetwork::~CreateNetwork()
{
    LOG(LTRACE)<<"Good bye CreateNetwork\n";
}

void CreateNetwork::prepareInterface()
{
    LOG(LTRACE) << "CreateNetwork::prepareInterface\n";

    h_onModels.setup(this, &CreateNetwork::onModels);
    registerHandler("onModels", &h_onModels);
    h_onJointMultiplicity.setup(this, &CreateNetwork::onJointMultiplicity);
    registerHandler("onJointMultiplicity", &h_onJointMultiplicity);

    registerStream("in_models", &in_models);
    addDependency("onModels", &in_models);
    registerStream("in_jointMultiplicity", &in_jointMultiplicity);
    addDependency("onJointMultiplicity", &in_jointMultiplicity);

    registerStream("out_network", &out_network);
}

DSL_network CreateNetwork::getNetwork()
{
    return this->theNet;
}

string CreateNetwork::getNodeName(int nodeHandle)
{
    return features[nodeHandle];
}

bool CreateNetwork::onInit()
{
    LOG(LTRACE) << "CreateNetwork::initialize\n";

    return true;
}

bool CreateNetwork::onFinish()
{
    LOG(LTRACE) << "CreateNetwork::finish\n";

    return true;
}

bool CreateNetwork::onStop()
{
    LOG(LTRACE) << "CreateNetwork::onStop\n";

    return true;
}

bool CreateNetwork::onStart()
{
    LOG(LTRACE) << "CreateNetwork::onStart\n";

    return true;
}

void CreateNetwork::onModels()
{
    LOG(LDEBUG) << "CreateNetwork::onModels";
//    map<int,int> newModel = in_models.read();
//    models.push_back(newModel);
    models = in_models.read();
    LOG(LDEBUG) << "CreateNetwork: Number of models: " << models.size();
}

void CreateNetwork::onJointMultiplicity()
{
    LOG(LDEBUG) << "CreateNetwork::onJointMultiplicity";
	jointMultiplicityVector = in_jointMultiplicity.read();
    //TODO: FIXME: think of better way of synchronizing input dataports
    if (theNet.GetNumberOfNodes() == 0 && !models.empty()) {
		mapFeaturesNames();
		buildNetwork();
		exportNetwork();
    }
}

void CreateNetwork::buildNetwork()
{
    LOG(LDEBUG) << "=========== BUILDING NETWORK ==============";
    vector<string> outcomesNames;
    vector<string> parentsNames;
    outcomesNames.push_back("YES");
    outcomesNames.push_back("NO");
    for (unsigned i=0; i<jointMultiplicityVector.size(); ++i) {
        addNode(features[i], outcomesNames, parentsNames);
        LOG(LTRACE) << "Added feature no " << i;
    }
    LOG(LDEBUG) << "Number of feature nodes added: " << jointMultiplicityVector.size();
    for (unsigned j=0; j<models.size(); ++j) {
        stringstream name;
        name << "H" << j;
        string hypothesisName(name.str());
        map<int,int> modelFeatures = models[j];
        map<int,int>::iterator it = modelFeatures.begin();
        while (it != modelFeatures.end()) {
            string parentName = features[it->first];
            parentsNames.push_back(parentName);
            ++it;
        }
        addNode(hypothesisName, outcomesNames, parentsNames);
        LOG(LTRACE) << "Added hypotheses no " << j;
        parentsNames.clear();
    }
    LOG(LDEBUG) << "Added " << models.size() << " hypotheses nodes";
    setBaseNetworkCPTs();
}

void CreateNetwork::loadNetwork()
{
    int result = -1;
    //result = theNet.ReadFile("/home/qiubix/DCL/BayesNetwork/in_network.xdsl", DSL_XDSL_FORMAT);
    //result = theNet.ReadFile("/home/kkaterza/DCL/BayesNetwork/in_network.xdsl", DSL_XDSL_FORMAT);
    LOG(LWARNING) << "Reading network file: " << result;
}

void CreateNetwork::mapFeaturesNames()
{
    for (unsigned i=0; i<jointMultiplicityVector.size(); ++i) {
        std::stringstream name;
        name << "F" << i;
        string featureName(name.str());
        features.insert(std::make_pair<int,string>(i,featureName));
    }
}

void CreateNetwork::setBaseNetworkCPTs()
{
    LOG(LDEBUG) << "Set base network CPTs";
    setBaseFeaturesCPTs();
//    setBaseHypothesesCPTs();
}

void CreateNetwork::setBaseFeaturesCPTs()
{
    LOG(LDEBUG) << "------- Set base features CPTs -------";
    int sum = 0;
    for (unsigned i=0; i<jointMultiplicityVector.size(); ++i) {
        LOG(LDEBUG) << "feature " << i << " multiplicity: " << jointMultiplicityVector[i];
        sum += jointMultiplicityVector[i];
    }
    LOG(LDEBUG) << "summed multiplicity of all features: " << sum;

    std::map<int,string>::iterator it = features.begin();
    for( ; it!=features.end(); ++it) {
        LOG(LDEBUG) << "feature no: " << it->first;
        int multiplicity = jointMultiplicityVector[it->first];
        LOG(LDEBUG) << "feature multiplicity in model: " << multiplicity;
        double baseProbability = (double) multiplicity/sum;
		std::vector <double> probabilities;
        probabilities.push_back(baseProbability);
        probabilities.push_back(1 - baseProbability);
        LOG(LDEBUG) << "calculated base probability " << baseProbability;
        setNodeCPT(it->second, probabilities);
    }
}

void CreateNetwork::setBaseHypothesesCPTs()
{
    LOG(LDEBUG) << "------ Set base hypotheses CPTs ------";
    int jointMultiplicitySum = 0;
    for (unsigned i=0; i<jointMultiplicityVector.size(); ++i) {
        jointMultiplicitySum += jointMultiplicityVector[i];
    }

    for (unsigned i=0; i<models.size(); ++i) {
        LOG(LDEBUG) << "hypothesis no :" << i;
        double P_Hi = (double) 1/models.size();
        map<int,int> modelFeatures = models[i];
        map<int,int>::iterator k = modelFeatures.begin();

        int modelMultiplicitySum = 0;
        while (k != modelFeatures.end()) {
            modelMultiplicitySum += k->second;
            ++k;
        }

        k = modelFeatures.begin();
        double baseProbability = 1;
        while (k != modelFeatures.end()) {
            double P_Fk_given_Hi = (double) (k->second)/modelMultiplicitySum;
            double P_Fk = (double) jointMultiplicityVector[k->first]/jointMultiplicitySum;
            double P_Hi_given_Fk = (double) P_Fk_given_Hi * P_Hi / P_Fk;
            baseProbability *= P_Hi_given_Fk;
            ++k;
        }
        LOG(LDEBUG) << "Calculated probability: " << baseProbability;
		std::vector<double> probabilities;
        probabilities.push_back(baseProbability);
        probabilities.push_back(1 - baseProbability);
        stringstream name;
        name << "H" << i;
        string hypothesisName(name.str());
        setNodeCPT(hypothesisName, probabilities);
    }
}

void CreateNetwork::addNode(const std::string name, const std::vector<string> outcomesNames, const std::vector<string> parentsNames)
{
    LOG(LDEBUG) << "Add node to network: " << name;
    int newNode = theNet.AddNode(DSL_CPT, name.c_str());
    DSL_idArray outcomes;
    for (int i=0; i<outcomesNames.size(); i++) {
        outcomes.Add(outcomesNames[i].c_str());
    }
    theNet.GetNode(newNode)->Definition()->SetNumberOfOutcomes(outcomes);

    int nextParent;
    for (int i=0; i<parentsNames.size(); i++) {
        nextParent = theNet.FindNode(parentsNames[i].c_str());
        theNet.AddArc(nextParent, newNode);
    }
//    features.insert(std::make_pair<int,string>(newNode, name));
}

void CreateNetwork::setNodeCPT(const string name, vector<double> probabilities)
{
    LOG(LDEBUG) << "Set node CPT: " << name;
    int node = theNet.FindNode(name.c_str());
    DSL_sysCoordinates theCoordinates(*theNet.GetNode(node)->Definition());

    std::vector<double>::iterator it = probabilities.begin();
    do {
        theCoordinates.UncheckedValue() = *it;
        ++it;
    } while(theCoordinates.Next() != DSL_OUT_OF_RANGE || it != probabilities.end());
}

void CreateNetwork::exportNetwork()
{
    theNet.WriteFile("out_network.xdsl", DSL_XDSL_FORMAT);
    out_network.write(theNet);
}

}//: namespace Network
}//: namespace Processors
