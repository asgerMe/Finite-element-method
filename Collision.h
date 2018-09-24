
#include <GU/GU_SDF.h>
#include <SIM/SIM_Object.h>
#include <SIM/SIM_ObjectArray.h>
#include <SIM/SIM_SingleSolver.h>
#include <SIM/SIM_GeometryCopy.h>
#include "Placeholder.h"

#ifndef __COLLISION__
#define __COLLISION__

class Collider
{
private:
	
public:
	Collider();
	//static void collidePoints(SIM_Object &object, GU_Detail *gdp, Data_struct &ds);
	static void apply_external_forces(const SIM_Object& object, const double &mass, const UT_Vector3 &pos, const UT_Vector3 &v, UT_Vector3 &result);
	static bool is_surface(const GA_Detail *gdp, const GA_Offset &ptoff, const char* surface_group_name);
	static void calculate_element_hessian(const GU_Detail *gdp, const GA_Primitive *prim, const Data_struct_2D &ds, unsigned int i, unsigned int j);
};
#endif
