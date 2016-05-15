#include "BayesNetworkGenerator.hpp"

using Processors::Network::BayesNetwork;

BayesNetwork getDefaultBayesNetwork() {
  BayesNetwork network;
  network.addVoxelNode(0);
  network.addVoxelNode(1);
  network.addVoxelNode(2);
  network.addFeatureNode(0);
  network.addFeatureNode(1);
  network.connectNodes("V_1", "V_0");
  network.connectNodes("V_2", "V_0");
  network.connectNodes("F_0", "V_1");
  network.connectNodes("F_1", "V_2");
  network.setCPTofAllVoxelNodes(3);
  return network;
}
