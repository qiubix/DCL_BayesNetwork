#ifndef DSL_DEFEQUATION_H
#define DSL_DEFEQUATION_H

// {{SMILE_PUBLIC_HEADER}}

#include "nodedef.h"
#include "generalequation.h"
#include <string>

// represents an equation node which can only have
// DSL_equation as its parents.

class DSL_equation : public DSL_nodeDefinition
{
public:
	typedef std::vector<std::pair<std::string, double> > IntervalVector;

    DSL_equation(int myHandle, DSL_network *theNetwork);
	DSL_equation(DSL_nodeDefinition &that);
	~DSL_equation();

    int GetType() const { return DSL_EQUATION; }
    const char* GetTypeName() const { return "EQUATION"; }

    int AddParent(int theParent);
    int RemoveParent(int theParent);

    DSL_generalEquation& GetEquation() { return equation; }
    const DSL_generalEquation& GetEquation() const { return equation; }
	DSL_expression* GetSolution() { return solution ? solution : Solve(); }
	const DSL_expression* GetSolution() const { return solution ? solution : Solve(); }
    
    int SetEquation(const std::string &thisOne, int *errPos = NULL, std::string *errMsg = NULL);
    bool ValidateEquation(const std::string &eq, std::vector<std::string> &vars, std::string &errMsg, int *errPos = NULL) const;

    void SetBounds(double low, double high);
    void GetBounds(double &low, double &high) const { low = lowBound; high = highBound; }

	bool IsDiscretized() const { return !discIntervals.empty(); }
	const DSL_Dmatrix* GetDiscProbs() const { return discProbs; }
	void InvalidateDiscProbs();
	int UpdateDiscProbs(int randSeed = 0, std::vector<std::vector<float> > *samples = NULL);
	int SetDiscProbs(const DSL_doubleArray& probs);
	int SetDiscIntervals(const IntervalVector &intervals);
	int ClearDiscIntervals();
	const IntervalVector& GetDiscIntervals() const { return discIntervals; }
	int GetDiscInterval(int intervalIndex, double &lo, double &hi) const;

	int Clone(DSL_nodeDefinition &that);

private:
	void Construct();
	void ParentIdChanged(int parentHandle, const char *oldId, const char *newId);
    void InvalidateDescendants();
	void InvalidateWithDescendants();
	DSL_expression* Solve() const;

    DSL_generalEquation equation;
    double lowBound;
    double highBound;
	mutable DSL_expression* solution;
	
	IntervalVector discIntervals;
	DSL_Dmatrix *discProbs;
};

#endif
