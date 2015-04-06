# Test cases

1. BayesNetwork class
- should initialize empty network
- should add one node to empty network
- should add node to network with only one node
- should connect two nodes
- should add node to the end of the chain
- should add node in the middle of the chain
- should return child of a node
- should return next root node
- should get number of children
- should get number of nodes

2. CPTManager class
+ should display whole CPT
+ should set CPT of the node
+ should throw exception when CPT size and size of given probabilities vector not matching
- should throw exception when probabilities vector incorrect
  - values not from <0,1> range
  - values don't add up to 1
- should display node probability
- should modify only one cell
