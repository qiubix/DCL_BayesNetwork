# Test cases

0. NetworkBuilder
+ should build network with only one feature node
+ should have the same number of feature nodes as points in cloud
+ should build network with multiple feature nodes
+ should set default probability values for feature nodes
+ should have only one child node
+ should not have cycles
- should have unique ids
- should fill CPTs acording to number of parents
- should have nodes with unique names
- should build network on new cloud
- should not build network if one is already being build
- should wait for joint multiplicity before building network

1. BayesNetwork class
+ should initialize empty network
+ should get number of nodes
+ should check if network has node
+ should add one node to empty network
+ should add node to network with nodes
+ should throw exception when adding already existing node
+ should connect two nodes
  + should throw exception when nodes can't be connected
    ?- should add node to the end of the chain
    ?- should add node in the middle of the chain
+ should get number of children
+ should throw exception when trying to create incorrect node name
- should throw exception when trying to get not existing node

New interface for alternative CPT:
- should return child of a node
- should return next root node
  - should return first root node
  - should return next not visited node
  - should return the same node if last root node still wasn't visited
  - should stop at the last node

BayesNetworkNode class
- should init node
- should copy nodes properly
- should change node state to visited

2. CPTManager class
+ should display whole CPT
+ should set CPT of the node
+ should throw exception when CPT size and size of given probabilities vector not matching
- should throw exception when probabilities vector incorrect
  + values not from <0,1> range
  - values don't add up to 1
+ should display node probability
- should modify only one cell

3. OctreeNode
+ should init node
+ should copy node
+ should get number of children
+ should set id

-> leaf

  + should return point indices
  + should set id

-> branch

  + should set id
  + should check whether node has only one child
  + should check whether next node is also branch node

4. Octree
+ should init octree with point cloud
+ should get first octree node
+ should get next octree node in depth-search
+ should get last octree node in depth-search

5. SOMEvaluation
- should deactivate all features
- should set node to observed
- should propagate probabilities
- should read hypothesis probability
- should activate all features of instance

Integration tests
- should read SIFT point cloud from input port
- should write network to output port
- should read network from input port
- should display proper probability for sample model and instance
