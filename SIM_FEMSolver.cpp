#include "SIM_FEMSolver.h"
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

	static PRM_Name theBoundName(SIM_NAME_USE2D, "Manifold FEM");
	static PRM_Default errorDefault(0.1);
	static PRM_Name         sopOrdChoices[] =
	{
		PRM_Name("choice1", "Mixed Mode"),
		PRM_Name("choice2", "Implicit"),
		PRM_Name("choice3", "Explicit"),
		PRM_Name("choice4", "Static"),
		PRM_Name(0)
	};
	static PRM_ChoiceList   sopOrdinalMenu(PRM_CHOICELIST_SINGLE, sopOrdChoices);

	// define a template for parameters to control the force's behavior
	static PRM_Template          theTemplates[] = {
		PRM_Template(PRM_INT,1,&theConName, PRM100Defaults),
		PRM_Template(PRM_ORD, 1, &sopOrdinalName, 0, &sopOrdinalMenu),
		PRM_Template(PRM_TOGGLE, 1, &theBoundName, PRMzeroDefaults),
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

			const UT_StringHolder v_name("P0");
			GA_RWHandleV3D v_handle(gdp->addFloatTuple(GA_ATTRIB_POINT, v_name, 3, GA_Defaults(0.0)));
		
			const UT_StringHolder vp_name("v");
			GA_RWHandleV3D vv_handle(gdp->addFloatTuple(GA_ATTRIB_POINT, vp_name, 3, GA_Defaults(0.0)));

			const UT_StringHolder accp_name("acc");
			GA_RWHandleV3D acc_handle(gdp->addFloatTuple(GA_ATTRIB_POINT, accp_name, 3, GA_Defaults(0.0)));

			const UT_StringHolder stress_name("cauchy_stress");
			GA_RWHandleM3D stress_handle(gdp->addFloatTuple(GA_ATTRIB_PRIMITIVE, stress_name, 9, GA_Defaults(1.0)));

			const UT_StringHolder test_name("test");
			GA_RWHandleF test_handle(gdp->addFloatTuple(GA_ATTRIB_PRIMITIVE, test_name, 1, GA_Defaults(1.0)));

			GA_ROHandleV3D pp(gdp->findAttribute(GA_ATTRIB_POINT, "P"));

			GA_Iterator primit(gdp->getPrimitiveRange());

			Data_struct ds;
			
			unsigned int points = gdp->getPointRange().getEntries();
			ds.init(points);
			
			for (primit; !primit.atEnd(); ++primit)
			{
				GA_Offset primoff = *primit;
				const GA_Primitive *prim = gdp->getPrimitive(primoff);
			
				UT_Matrix3 init_stress(0.0f);
				if (stress_handle.isValid())
				{
					stress_handle.set(primoff, init_stress);
				}
				
				for (GA_Iterator pointit(prim->getPointRange()); !pointit.atEnd(); ++pointit)
				{
					GA_Offset ptoff = *pointit;
					UT_Vector3D position = pp.get(ptoff);
					v_handle.set(ptoff, position);
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
			parms.dt = time_step;
			parms.dt = parms.dt / sub_steps;

			Wilson w(parms.dt);
			bool __explicit__ = false;
			if (getintMode() == 0)
				__explicit__ = false;
			if (getintMode() == 1)
				__explicit__ = false;
			if (getintMode() == 2)
				__explicit__ = true;
			if (getintMode() == 3)
			{
				__explicit__ = false;
				w.theta = 1;
			}

				
			Data_struct ds;
			unsigned int points = gdp->getPointRange().getEntries();
			ds.init(points);

			gridUpdate::assemble_E(ds, parms);
			gridUpdate::assemble_m2(ds, parms);
			gridUpdate::assemble_m1(ds, parms);

			for (unsigned int i = 0; i < sub_steps; i++)
			{
				const SIM_Geometry *reference_geo = object.getGeometry();
				SIM_GeometryAutoReadLock  const_lock(reference_geo);
				const GU_Detail *reference_gdp = const_lock.getGdp();
				GA_Iterator primit(gdp->getPrimitiveRange());
				ds.clear_globals();
				ds.i = i;
				bool use_vic = false;

				for (primit; !primit.atEnd(); ++primit)
				{
					GA_Offset primoff = *primit;
					const GA_Primitive *prim = gdp->getPrimitive(primoff);
					if (prim->getPointRange().getEntries() != 4)
						continue;

					bool attribute_present = false;

					gridUpdate::update_material_parms(gdp, prim, ds, parms, attribute_present, use_vic);

					if (attribute_present)
					{
						gridUpdate::assemble_E(ds, parms);
						gridUpdate::assemble_m2(ds, parms);
						gridUpdate::assemble_m1(ds, parms);
					}

					gridUpdate::update_tetrapoints(gdp, prim, ds);
					gridUpdate::global_indices(gdp, ds);
					gridUpdate::update_jacobians(gdp, prim, ds);
					gridUpdate::assemble_B(ds);

					if (!__explicit__)  //Assemble for implicit update
					{
						gridUpdate::assemble_BNL(ds);
						gridUpdate::assemble_global_implicit(object, gdp, primoff, ds, parms, w, use_vic);
					}
					if (__explicit__)
					{
						gridUpdate::assemble_global(object, gdp, primoff, ds, parms, use_vic);
					}
				}

				if (__explicit__)
				{
					gridUpdate::step_solution_exp(gdp, ds, parms, w);
				}

				if (!__explicit__)
				{
					//solve systems of equations
					gridUpdate::apply_inertia(ds, gdp, w);
					DiagonalPreconditioner diagonal_preconditioner(ds.global_stiffness);
					UT_Functor2<void, const UT_Vector&, UT_Vector&> multiply_full(
						&ds.global_stiffness, &UT_SparseMatrix::multVec
					);

					int cp = 200;
					double error = 0.000000000000001;
					IterationTester iteration_tester(cp, error);
					UT_Functor2<bool, int, const UT_Vector&> keep_iterating(iteration_tester);
					UT_Functor2<void, const UT_Vector&, UT_Vector&>
						preconditioner_full(diagonal_preconditioner);

					UT_MatrixIterSolver::PCG(
						ds.X, ds.global_force,
						multiply_full,
						preconditioner_full,
						keep_iterating
					);

					gridUpdate::step_solution_imp(gdp, ds, w, parms);
				}
			}
		}
	}
	return SIM_SOLVER_SUCCESS;
}

