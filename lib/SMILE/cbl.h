#ifndef DSL_CBL_H
#define DSL_CBL_H

// {{SMILE_PUBLIC_HEADER}}

#include "bkgndknowledge.h"
#include <set>
#include <ctime>

class DSL_dataset;
class DSL_progress;
class DSL_network;
class DSL_pattern;

class DSL_cormat;
class DSL_lazyadtree;

class DSL_cbl
{
public:
    DSL_cbl()
    {
        maxAdjacency = 8;
        maxSearchTime = 0;
        significance = 0.05;
        cm = NULL;
        ad = NULL;

        timing = false;
        timeout = false;
    }
    
    int maxAdjacency;
    int maxSearchTime;
    double significance;

	DSL_bkgndKnowledge bkk;

    int Learn(const DSL_dataset &ds, DSL_pattern &pat, DSL_progress *progress = NULL) const;

private:
    int Step1(const DSL_dataset &ds, DSL_pattern &pat, DSL_progress *progress = NULL) const;
    int Step2(const DSL_dataset &ds, DSL_pattern &pat, DSL_progress *progress = NULL) const;
    int Step3(const DSL_dataset &ds, DSL_pattern &pat, DSL_progress *progress = NULL) const;
    int Step4PC(const DSL_dataset &ds, DSL_pattern &pat, DSL_progress *progress = NULL) const;
    int Step4FCI(const DSL_dataset &ds, DSL_pattern &pat, DSL_progress *progress = NULL) const;
    int Step5FCI(const DSL_dataset &ds, DSL_pattern &pat, DSL_progress *progress = NULL) const;

    void PossibleDSep(const DSL_pattern& pat, const int from, const int to, std::set<int>& sepset) const;
    bool PossibleDSep(const DSL_pattern& pat, const int from, const int current, const int to, std::vector<int>& visited, std::set<int>& sepset) const;
    void MakePowerSet(const std::set<int>& sepset, std::vector<std::set<std::set<int> > >& PowerSet) const;
    void AddToPowerSet(const std::set<int>& sepset, std::vector<std::set<std::set<int> > >& PowerSet) const;
    bool isTriangle(const DSL_pattern& pat, const int a, const int b, const int c) const;
    bool isDefDiscrPath(const DSL_pattern& pat, const int from, const int to, const int target, std::vector<int>& path) const;
    bool isDefDiscrPath(const DSL_pattern& pat, const int from, const int current, const int to, const int target, std::vector<int>& visited) const;

    mutable time_t t0;
    mutable bool timing;
    mutable bool timeout;
    mutable int nvar;
    mutable int n;
    mutable bool disc;
    mutable bool cont;
    mutable DSL_cormat *cm;
    mutable DSL_lazyadtree *ad;
    mutable std::vector<std::vector<std::vector<int> > > sepsets;
    mutable std::vector<std::vector<std::set<int> > > pos_sepsets;
};

#endif
