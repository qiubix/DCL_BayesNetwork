# Add all components here using ADD_COMPONENT(<COMPONENT_DIRECTORY>)
ADD_COMPONENT(CreateNetwork)

ADD_COMPONENT(CreateNetworkWithSpacialDependencies)

ADD_COMPONENT(MapMultiplicity)

ADD_COMPONENT(NetworkTester)

ADD_COMPONENT(MultipleModels)

ADD_COMPONENT(SIFTAdder)

ADD_COMPONENT(SIFTFeatureMatcher)

ADD_COMPONENT(SOMEvaluation)

ADD_COMPONENT(SOMSimpleEvaluation)

ADD_COMPONENT(MockNetworkBuilder)

add_subdirectory(tests)
