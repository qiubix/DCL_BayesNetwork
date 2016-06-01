SET(CMAKE_INCLUDE_CURRENT_DIR ON)

# If DCL provides any additional libraries - add them here

# Get soource files of library
# FILE(GLOB lib_src *.cpp)
SET(lib_src BayesNetwork.cpp BayesNetworkNode.cpp CPTManager.cpp)
ADD_LIBRARY(BayesNetworkTypes SHARED ${lib_src} BayesNetworkExceptions.hpp AbstractNetwork.hpp)
# Link with other libraries
TARGET_LINK_LIBRARIES(BayesNetworkTypes ${SMILE_LIBS} ${DisCODe_LIBRARIES})

INSTALL(
  TARGETS BayesNetworkTypes
  RUNTIME DESTINATION bin COMPONENT applications
  LIBRARY DESTINATION lib COMPONENT applications
  ARCHIVE DESTINATION lib COMPONENT sdk
)

# Get soource files of library
SET(octree_types_src Octree.cpp OctreeNode.cpp OctreeBranchNode.cpp OctreeLeafNode.cpp DepthFirstIterator.cpp)
ADD_LIBRARY(OctreeTypes SHARED ${octree_types_src} OctreeContainers.hpp)
# Link with other libraries
TARGET_LINK_LIBRARIES(OctreeTypes ${DisCODe_LIBRARIES} ${PCL_OCTREE_LIBRARIES} ${PCL_IO_LIBRARIES})

# Install library
INSTALL(
  TARGETS OctreeTypes
  RUNTIME DESTINATION bin COMPONENT applications
  LIBRARY DESTINATION lib COMPONENT applications
  ARCHIVE DESTINATION lib COMPONENT sdk
)

# If DCL provides any additional headers to be used from outside of it, add them

# Get list of header files
FILE(GLOB headers *.hpp)

# Install them to include subdirectory
install(
    FILES ${headers}
    DESTINATION include/Types
    COMPONENT sdk
)
