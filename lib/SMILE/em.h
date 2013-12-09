#ifndef DSL_EM_H
#define DSL_EM_H

// {{SMILE_PUBLIC_HEADER}}

/* This class implements the EM algorithm
 * 
 * IMPORTANT: THE STATE INDICES IN THE DATA SET SHOULD MATCH THE STATE INDICES IN THE NETWORK
 *            IT DOES *NOT* AUTOMATICALLY MATCH THE STATE NAMES
 *            THIS IMPLIES THAT YOU PROBABLY HAVE TO REORDER THE STATES IN THE DATA SET
 *            (DSL_dataset::MatchNetwork can help in reordering)   
 * 
 * The equivalent sample size determines the weight of the network
 * against the data. An equivalent sample size of 100, means that
 * the current parameters in the network are based on 100 cases.
 * A typical equivalent sample size is 0, this means that the current
 * parameters have no weight and will be overwritten.
 * 
 * The variables in the network and data set should be all discrete or all continuous.
 */

#include "dataset.h"

class DSL_network;
class DSL_progress;

class DSL_em
{
public:
	DSL_em() : eqSampleSize(1), randParams(true), randSeed(0), makeParamsUniform(false), relevance(false) {}
    
	int Learn(const DSL_dataset& ds, DSL_network& orig, const std::vector<DSL_datasetMatch> &matches, double *loglik = NULL, DSL_progress *progress = 0);
    int Learn(const DSL_dataset& ds, DSL_network& orig, const std::vector<DSL_datasetMatch> &matches, const std::vector<int> &fixedNodes, double *loglik = NULL, DSL_progress *progress = 0);
	
	void SetEquivalentSampleSize(float eqs) { eqSampleSize = eqs; }
	float GetEquivalentSampleSize() const { return eqSampleSize; }
	void SetRandomizeParameters(bool r) { randParams = r; }
	bool GetRandomizeParameters() const { return randParams; }
	int GetSeed() const { return randSeed; }
	void SetSeed(int seed) { randSeed = seed; }
	void SetRelevance(bool r) { relevance = r; }
	bool GetRelevance() { return relevance; }
	void SetUniformizeParameters(bool u) { makeParamsUniform = u; }
	bool GetUniformizeParameters() const { return makeParamsUniform; }

private:
	int LearnDisc(const DSL_dataset& ds, DSL_network& orig, const std::vector<DSL_datasetMatch> &matches, const std::vector<int> &fixedNodes, double *loglik, DSL_progress *progress = 0);
	int LearnCont(const DSL_dataset& ds, DSL_network& orig, const std::vector<DSL_datasetMatch> &matches, const std::vector<int> &fixedNodes, double *loglik, DSL_progress *progress = 0);
    
	float eqSampleSize;
	bool randParams;
	int randSeed;
	bool makeParamsUniform;
	bool relevance;
};

#endif
