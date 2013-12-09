#ifndef DSL_VALEQUATIONEVALUATION_H
#define DSL_VALEQUATIONEVALUATION_H

// {{SMILE_PUBLIC_HEADER}}

#include "nodeval.h"
#include <vector>

class DSL_valEqEvaluation : public DSL_nodeValue
{
public:
    DSL_valEqEvaluation(int myHandle, DSL_network *net);
	DSL_valEqEvaluation(DSL_nodeValue &that);

	int Clone(DSL_nodeValue &that);

    int GetType() const { return DSL_EQUATIONEVALUATION; }

    int AddIndexingParent(int parent) { return DSL_WRONG_NODE_TYPE; }
    int GetSize() { return DSL_WRONG_NODE_TYPE; }

    const std::vector<std::pair<double, double> > &GetSamples() const { return samples; }
    double GetSample(int index) const { return samples[index].first; }
    
	double GetSampleMean() const;
    double GetSampleStdDev() const;
	void GetStats(double &mean, double &stddev, double &vmin, double &vmax) const;

	bool HasSamplesOutOfBounds() const;

	void AddSample(double sample) { samples.push_back(std::make_pair(sample, 1.0)); }
    void SamplingStart(int samplesToReserve = 0);
    void SamplingEnd();

	// used only for exact evaluation
	void SetSampleMean(double mean) { sampleMean = sampleMin = sampleMax = mean; sampleStdDev = 0.0; statsAvailable = true; }
    void SetSamples(const std::vector<std::pair<double, double> > &samples);

	int SetDiscBeliefs(const std::vector<double> &beliefs);
	const std::vector<double>& GetDiscBeliefs() const { return discBeliefs; }

    int SetEvidence(double evidence);
    int SetPropagatedEvidence(double evidence);
    int GetEvidence(double &e) const { e = evidence; return DSL_OKAY; }
    int ClearEvidence();
    int ClearPropagatedEvidence();

private:
	void CalcStats() const;

    double evidence;

    std::vector<std::pair<double, double> > samples; 
    
	// if equation is determinisitc (i.e., all its ancestor is deterministic), 
    // sampleMean is its value and sampleStdDev is zero
    mutable double sampleMean;
    mutable double sampleStdDev;
	mutable double sampleMin;
	mutable double sampleMax;

	mutable bool statsAvailable;

	std::vector<double> discBeliefs;
};

#endif // DSL_VALEQUATIONEVALUATION_H
