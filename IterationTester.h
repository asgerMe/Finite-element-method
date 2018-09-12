#include <UT/UT_Vector.h>
#include <PRM/PRM_Include.h>
#include "SIM_FEMSolver.h"

#include <OP/OP_AutoLockInputs.h>
#include <OP/OP_Operator.h>
#include <OP/OP_OperatorTable.h>
#include <CH/CH_ExprLanguage.h>

class IterationTester 
{
public:
	IterationTester(int cp, float error)
	{
		constraintPasses = cp;
		errorTol = error;
	}
	bool operator()(int iteration, const UT_Vector& r) const
	{
		if (iteration > constraintPasses)
			return false;
		return (r.norm2() > errorTol);
	}
private:
	int constraintPasses;
	float errorTol;

};
