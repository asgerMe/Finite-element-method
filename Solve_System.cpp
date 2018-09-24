#include "Solve_System.h"
#include "Preconditioner.h"
#include "IterationTester.h"
#include <UT/UT_SparseMatrix.h>
#include <UT/UT_MatrixSolver.h>
#include <SIM/SIM_Engine.h>
#include <SIM/SIM_PRMShared.h>
#include <SIM/SIM_DopDescription.h>
#include <SIM/SIM_SingleSolver.h>
#include <SIM/SIM_GeometryCopy.h>

#include "solve_linear_templates.cpp"

SIM_SingleSolver::SIM_Result SOLVE_LINEAR::SOLVE_3D(const SIM_Object &object, GU_Detail *gdp, Material &parms, Wilson &w, unsigned int sub_steps, bool __explicit__)
{
	Data_struct ds;
	unsigned int points = gdp->getPointRange().getEntries();
	ds.init(points);

	gridUpdate::assemble_E<Data_struct>(ds, parms);
	gridUpdate::assemble_m2<Data_struct>(ds, parms);
	gridUpdate::assemble_m1<Data_struct>(ds, parms);

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
				return SIM_SingleSolver::SIM_SOLVER_FAIL;

			bool attribute_present = false;

			gridUpdate::update_material_parms(gdp, prim, ds, parms, attribute_present, use_vic);

			if (attribute_present)
			{
				gridUpdate::assemble_E<Data_struct>(ds, parms);
			}
			if (use_vic && attribute_present)
			{
				gridUpdate::assemble_m2<Data_struct>(ds, parms);
				gridUpdate::assemble_m1<Data_struct>(ds, parms);
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
			gridUpdate::step_solution_exp<Data_struct>(gdp, ds, parms, w);
		}

		if (!__explicit__)
		{
			invert_sparse<Data_struct>(gdp, ds, w);
			gridUpdate::step_solution_imp<Data_struct>(gdp, ds, w, parms);
		}
	}
	return SIM_SingleSolver::SIM_SOLVER_SUCCESS;
};
SIM_SingleSolver::SIM_Result SOLVE_LINEAR::SOLVE_2D(const SIM_Object &object, GU_Detail *gdp, Material &parms, Wilson &w, unsigned int sub_steps, bool __explicit__)
{
	Data_struct_2D ds;
	unsigned int points = gdp->getPointRange().getEntries();
	ds.init(points);

	gridUpdate::assemble_E<Data_struct_2D>(ds, parms);
	gridUpdate::assemble_m2<Data_struct_2D>(ds, parms);
	gridUpdate::assemble_m1<Data_struct_2D>(ds, parms);
	
	
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
			if (prim->getPointRange().getEntries() != 3)
				return SIM_SingleSolver::SIM_SOLVER_FAIL;

			bool attribute_present = false;
			gridUpdate::update_material_parms<Data_struct_2D>(gdp, prim, ds, parms, attribute_present, use_vic);
			if (attribute_present)
			{
				gridUpdate::assemble_E<Data_struct_2D>(ds, parms);
				gridUpdate::assemble_m2<Data_struct_2D>(ds, parms);
				gridUpdate::assemble_m1<Data_struct_2D>(ds, parms);
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
			gridUpdate::step_solution_exp<Data_struct_2D>(gdp, ds, parms, w);
		}
		if (!__explicit__)
		{
			invert_sparse<Data_struct_2D>(gdp, ds, w);
			gridUpdate::step_solution_imp<Data_struct_2D>(gdp, ds, w, parms);
		}
	}

	return SIM_SingleSolver::SIM_SOLVER_SUCCESS;
};

template<typename T> void SOLVE_LINEAR::invert_sparse(GU_Detail *gdp, T &ds, const Wilson &w)
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
}


