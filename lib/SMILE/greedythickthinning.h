#ifndef DSL_GREEDYTHICKTHINNING_H
#define DSL_GREEDYTHICKTHINNING_H

// {{SMILE_PUBLIC_HEADER}}

#include "bkgndknowledge.h"

class DSL_network;
class DSL_dataset;

class DSL_greedyThickThinning
{
public:
	DSL_greedyThickThinning()
	{
		maxParents = 5;
		priors = K2;
		netWeight = 1.0;
	}

	int Learn(const DSL_dataset &data, DSL_network &net);

	enum PriorsType { K2, BDeu };
	PriorsType priors;
	int maxParents;
	double netWeight;

	DSL_bkgndKnowledge bkk;
};

#endif
