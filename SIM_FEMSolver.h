#ifndef __SIM_FEMSOLVER_h__
#define __SIM_FEMSOLVER_h__

#include <SIM/SIM_OptionsUser.h>
#include <SIM/SIM_SingleSolver.h>
#include <OP/OP_Operator.h>
#include <OP/OP_OperatorTable.h>
#include <SIM/SIM_Utils.h>
#include "Placeholder.h"

#define SIM_NAME_CONSTRAINTPASSES     "cPasses"
#define SIM_NAME_ERRORTOLERANCE "errorTol"
#define SIM_NAME_INTEGRATION_MODE "intMode"
#define SIM_NAME_USE2D "is2D"

#define SIM_NAME_YOUNG "Y"
#define SIM_NAME_POISSON "v"
#define SIM_NAME_DENSITY "density"
#define SIM_NAME_VISCOSITY "viscosity"

#define SIM_NAME_WINKLER "winkler"



class SIM_ObjectArray;
class SIM_GeometryCopy;

namespace HDK_Sample {
	class SIM_FEMSolver : public SIM_SingleSolver,
		public SIM_OptionsUser
	{
	public:
		GETSET_DATA_FUNCS_I(SIM_NAME_CONSTRAINTPASSES, ConstraintPasses);
		GETSET_DATA_FUNCS_F(SIM_NAME_ERRORTOLERANCE, errorTolerance);
		GETSET_DATA_FUNCS_I(SIM_NAME_INTEGRATION_MODE, intMode);
		GETSET_DATA_FUNCS_I(SIM_NAME_USE2D, bd);

		GETSET_DATA_FUNCS_I(SIM_NAME_WINKLER, winkler);

		GETSET_DATA_FUNCS_F(SIM_NAME_YOUNG, young);
		GETSET_DATA_FUNCS_F(SIM_NAME_POISSON, poisson);
		GETSET_DATA_FUNCS_F(SIM_NAME_DENSITY, density);
		GETSET_DATA_FUNCS_F(SIM_NAME_VISCOSITY, viscosity);
		
	protected:

		explicit             SIM_FEMSolver(const SIM_DataFactory *factory);
		virtual             ~SIM_FEMSolver();
	
		bool SIM_FEMSolver::SIM_primIterate(SIM_Object& object, SIM_GeometryCopy *state, const SIM_Time& time_step);

		virtual SIM_Result solveSingleObjectSubclass(
			SIM_Engine& engine, SIM_Object& object,
			SIM_ObjectArray& feedback_to_objects,
			const SIM_Time& time_step,
			bool object_is_new
		);
			
	private:
		static const SIM_DopDescription     *getFEMSolverDopDescription();
		DECLARE_STANDARD_GETCASTTOTYPE();
		DECLARE_DATAFACTORY(SIM_FEMSolver,
		SIM_SingleSolver,
			"Visco Elastic FEM",
			getFEMSolverDopDescription());
	};

} // End HDK_Sample namespace
#endif


