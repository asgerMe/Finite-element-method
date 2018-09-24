#include "GETFEMMatrices.h"

#ifndef __SOLVE_SYSTEM__
#define __SOLVE_SYSTEM__


class SOLVE_LINEAR
{
public:
	static SIM_SingleSolver::SIM_Result SOLVE_3D(const SIM_Object& object, GU_Detail *gdp, Material &parms, Wilson &w, unsigned int sub_steps, bool solve_explicit);
	static SIM_SingleSolver::SIM_Result SOLVE_2D(const SIM_Object& object, GU_Detail *gdp, Material &parms, Wilson &w, unsigned int sub_steps, bool solve_explicit);
	template<typename T> static void invert_sparse(GU_Detail *gdp, T &ds, const Wilson &w);
};



#endif