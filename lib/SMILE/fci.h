#ifndef DSL_FCI_H
#define DSL_FCI_H

// {{SMILE_PUBLIC_HEADER}}

#include <vector>

class DSL_dataset;
class DSL_progress;
class DSL_network;
class DSL_pattern;

class DSL_fci
{
public:
    DSL_fci()
    {
        maxAdjacency = 8;
        maxSearchTime = 0;
        significance = 0.05;
    }
    
    int maxAdjacency;
    int maxSearchTime;
    double significance;
    
    int Learn(const DSL_dataset &ds, DSL_pattern &pat, DSL_progress *progress = 0) const;
};

#endif
