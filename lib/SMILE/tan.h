#ifndef DSL_TAN_H
#define DSL_TAN_H

// {{SMILE_PUBLIC_HEADER}}

#include <string>
#include <vector>

class DSL_dataset;
class DSL_network;
class DSL_progress;

class DSL_tan
{
public:
	DSL_tan();
	int Learn(DSL_dataset &ds, DSL_network &net, DSL_progress *progress = NULL) const;
	std::string classvar;
    int maxSearchTime;
	unsigned int seed;
private:
	int PreChecks(const DSL_dataset &ds_, int &cvar) const;
};

#endif
