#include "SIM_FEMSolver.h"
#include "Solve_System.h"
#include <UT/UT_DSOVersion.h>
#include <PRM/PRM_Include.h>
#include <SIM/SIM_PRMShared.h>
#include <SIM/SIM_DopDescription.h>
#include <SIM/SIM_SingleSolver.h>
#include <SIM/SIM_GeometryCopy.h>
#include <UT/UT_SparseMatrix.h>
#include "Preconditioner.h"
#include "IterationTester.h"
#include <GU/GU_DetailHandle.h>
#include <UT/UT_Matrix.h>
#include <UT/UT_MatrixSolver.h>
#include <SIM/SIM_Object.h>
#include <SIM/SIM_ObjectArray.h>
#include <SIM/SIM_Engine.h>
#include "GETFEMMatrices.h"
#include "Placeholder.h"
#include <GU/GU_SDF.h>
#include <PRM/PRM_ChoiceList.h>


using namespace HDK_Sample;
void
initializeSIM(void *)
{
	IMPLEMENT_DATAFACTORY(SIM_FEMSolver);
}
SIM_FEMSolver::SIM_FEMSolver(const SIM_DataFactory *factory)
	: BaseClass(factory),
	SIM_OptionsUser(this)
{
}
SIM_FEMSolver::~SIM_FEMSolver()
{
}

const SIM_DopDescription *
SIM_FEMSolver::getFEMSolverDopDescription()
{
	static PRM_Name      theConName(SIM_NAME_CONSTRAINTPASSES, "Sub Steps");
	static PRM_Name         sopOrdinalName(SIM_NAME_INTEGRATION_MODE, "Integration Mode");

	static PRM_Name         poissonName(SIM_NAME_POISSON, "Poisson Ratio");
	static PRM_Name         densityName(SIM_NAME_DENSITY, "Density");
	static PRM_Name         viscosityName(SIM_NAME_VISCOSITY, "Viscosity");
	static PRM_Name         youngName(SIM_NAME_YOUNG, "Young's Modulus");
	static PRM_Name			winklerName(SIM_NAME_WINKLER, "Winkler Boundaries");

	static PRM_Name theBoundName(SIM_NAME_USE2D, "Manifold FEM");

	static PRM_Default youngDefault(100);
	static PRM_Default poissonDefault(0.25);
	static PRM_Default densityDefault(1);
	static PRM_Default viscosityDefault(0);
	static PRM_Default substepDefault(1);

	static PRM_Default winklerDefault(0);

	static PRM_Range viscosityRange(PRM_RANGE_RESTRICTED, 0);
	static PRM_Range densityRange(PRM_RANGE_RESTRICTED, 0.01);
	static PRM_Range poissonRange(PRM_RANGE_RESTRICTED, 0.001, PRM_RANGE_RESTRICTED, 0.499);
	static PRM_Range youngsRange(PRM_RANGE_RESTRICTED, 0.01);

	static PRM_Name         sopOrdChoices[] =
	{
		PRM_Name("choice2", "Implicit"),
		PRM_Name("choice3", "Explicit"),
		PRM_Name("choice4", "Static"),
		PRM_Name(0)
	};
	static PRM_ChoiceList   sopOrdinalMenu(PRM_CHOICELIST_SINGLE, sopOrdChoices);

	// define a template for parameters to control the force's behavior
	static PRM_Template          theTemplates[] = {
		PRM_Template(PRM_TOGGLE, 1, &theBoundName, PRMzeroDefaults),
		PRM_Template(PRM_TOGGLE, 1, &winklerName, PRMzeroDefaults),
		PRM_Template(PRM_INT,1,&theConName, &substepDefault),
		PRM_Template(PRM_ORD, 1, &sopOrdinalName, 0, &sopOrdinalMenu),
		PRM_Template(PRM_FLT, 1, &youngName,  &youngDefault, 0, &youngsRange),
		PRM_Template(PRM_FLT, 1, &poissonName, &poissonDefault, 0, &poissonRange),
		PRM_Template(PRM_FLT, 1, &densityName, &densityDefault, 0, &densityRange),
		PRM_Template(PRM_FLT, 1, &viscosityName, &viscosityDefault, 0, &viscosityRange),
		PRM_Template(PRM_TOGGLE, 1, &winklerName, PRMzeroDefaults),
		PRM_Template()
	};

	// define a template for parameters to control the guide geometry
	
	static SIM_DopDescription    theDopDescription(true,
		"visco_elastic_fem",
		"Visco Elastic FEM",
		SIM_SOLVER_DATANAME "/Visco Elastic FEM",
		classname(),
		theTemplates);
	theDopDescription.setDefaultUniqueDataName(1);
	return &theDopDescription;
}

SIM_Solver::SIM_Result SIM_FEMSolver::solveSingleObjectSubclass(
	SIM_Engine& engine, SIM_Object& object,
	SIM_ObjectArray& feedback_to_objects,
	const SIM_Time& time_step,
	bool object_is_new
)
{
	SIM_GeometryCopy *state = 0;
	bool is_2D = getbd();

	if (object_is_new)
	{
		
		if (!SIM_DATA_GET(object, SIM_GEOMETRY_DATANAME, SIM_GeometryCopy))
		{
			state = SIM_DATA_CREATE(object, SIM_GEOMETRY_DATANAME, SIM_GeometryCopy, SIM_DATA_RETURN_EXISTING | SIM_DATA_ADOPT_EXISTING_ON_DELETE);
			
			GU_DetailHandleAutoWriteLock lock(state->lockGeometry());
			GU_Detail *gdp = lock.getGdp();
			if (!gdp->getNumPoints())
				return SIM_SOLVER_FAIL;
		
			GA_RWHandleV3D a0(gdp->findAttribute(GA_ATTRIB_POINT, "P0"));
			GA_ROHandleV3D a1(gdp->findAttribute(GA_ATTRIB_POINT, "v"));
			GA_ROHandleV3D a2(gdp->findAttribute(GA_ATTRIB_POINT, "acc"));
			GA_RWHandleM3D a3(gdp->findAttribute(GA_ATTRIB_PRIMITIVE, "cauchy_stress"));

			if (a0.isInvalid() || a1.isInvalid() || a2.isInvalid() || a3.isInvalid())
				return SIM_SOLVER_FAIL;

			GA_ROHandleV3D pp(gdp->findAttribute(GA_ATTRIB_POINT, "P"));
			GA_Iterator primit(gdp->getPrimitiveRange());
			unsigned int points = gdp->getPointRange().getEntries();
			
			for (primit; !primit.atEnd(); ++primit)
			{
				GA_Offset primoff = *primit;
				const GA_Primitive *prim = gdp->getPrimitive(primoff);
			
				UT_Matrix3 init_stress(0.0f);
				if (a3.isValid())
				{
					a3.set(primoff, init_stress);
				}
				
				for (GA_Iterator pointit(prim->getPointRange()); !pointit.atEnd(); ++pointit)
				{
					GA_Offset ptoff = *pointit;
					UT_Vector3D position = pp.get(ptoff);
					a0.set(ptoff, position);
				}	
			}
		}
	}
	
	else
	{
		SIM_GeometryCopy *state = SIM_DATA_GET(object, SIM_GEOMETRY_DATANAME, SIM_GeometryCopy);
		if (state)
		{
			unsigned int sub_steps = getConstraintPasses();
			GU_DetailHandleAutoWriteLock lock(state->lockGeometry());
			GU_Detail *gdp = lock.getGdp();
			
			Material parms;
			parms.density = getdensity();
			parms.viscosity = getviscosity();
			parms.youngs_modulus = getyoung();
			parms.poisson_ratio = getpoisson();

			parms.dt = time_step;
			parms.dt = parms.dt / sub_steps;

			Wilson w(parms.dt);
			bool __explicit__ = false;
			if (getintMode() == 0)
				__explicit__ = false;
			if (getintMode() == 1)
				__explicit__ = true;
			if (getintMode() == 2)
			{
				__explicit__ = false;
				w.theta = 1.0;
			}
			if(!is_2D)
				SOLVE_LINEAR::SOLVE_3D(object, gdp, parms, w,  sub_steps, __explicit__);
			else if(is_2D)
				SOLVE_LINEAR::SOLVE_2D(object, gdp, parms, w, sub_steps, __explicit__);
		
			gdp->bumpAllDataIds();
		}
	}

	return state ? SIM_SOLVER_SUCCESS : SIM_SOLVER_FAIL;
}

