# Add all components here using ADD_COMPONENT(<COMPONENT_DIRECTORY>)
ADD_COMPONENT(CreateNetwork)

ADD_COMPONENT(NetworkBuilder)

ADD_COMPONENT(MapMultiplicity)

ADD_COMPONENT(NetworkTester)

ADD_COMPONENT(MultipleModels)

ADD_COMPONENT(SIFTAdder)

ADD_COMPONENT(SIFTFeatureMatcher)

ADD_COMPONENT(SOMEvaluation)

ADD_COMPONENT(SOMSimpleEvaluation)
ADD_COMPONENT(SOMJSONReader)
ADD_COMPONENT(PCDReader)

add_subdirectory(tests)
