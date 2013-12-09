#ifndef DSL_nb_old_H
#define DSL_nb_old_H

#include <vector>
#include <string>
// {{SMILE_PUBLIC_HEADER}}
#include "network.h"
#include "dataset.h"

class DSL_dataset;
class DSL_progress;
class DSL_network;
class DSL_stringArray;

using namespace std;

class DSL_nb_old
{
public:
	DSL_nb_old(){}
    int maxSearchTime;
	string classvar;
	int Learn(DSL_dataset ds, DSL_network &net, DSL_progress *progress = NULL) const;
};

#endif
