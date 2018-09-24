
#include <UT/UT_Vector.h>
#include <UT/UT_Matrix.h>
#include <GU/GU_Detail.h>
#include "Placeholder.h"
#include "MaterialProperties.h"
#include "Collision.h"


#ifndef ASSEMBLE_M2H
#define ASSEMBLE_M2H

class gridUpdate
{
public:
	gridUpdate();
	static void update_tetrapoints(GU_Detail *gdp, const GA_Primitive *prim, Data_struct &ds);
	static void update_tetrapoints(GU_Detail *gdp, const GA_Primitive *prim, Data_struct_2D &ds);
	
	static void update_jacobians(const GU_Detail *gdp, const GA_Primitive *prim,  Data_struct &ds);
	static void update_jacobians(const GU_Detail *gdp, const GA_Primitive *prim, Data_struct_2D &ds);

	static void assemble_B(Data_struct &ds);
	static void assemble_B(Data_struct_2D &ds);

	static void assemble_BNL(Data_struct &ds);
	static void assemble_BNL(Data_struct_2D &ds);

	static void assemble_global(const SIM_Object& object, GU_Detail *gdp, const GA_Offset &primoff, Data_struct &ds, const Material &parms, const bool &use_viscosity);
	static void assemble_global(const SIM_Object& object, GU_Detail *gdp, const GA_Offset &primoff, Data_struct_2D &ds, const Material &parms, const bool &use_viscosity);

	static void assemble_global_implicit(const SIM_Object &object ,GU_Detail *gdp, const GA_Offset &primoff, Data_struct &ds, const Material &parms, const Wilson &w, const bool &use_viscosity);
	static void assemble_global_implicit(const SIM_Object &object, GU_Detail *gdp, const GA_Offset &primoff, Data_struct_2D &ds, const Material &parms, const Wilson &w, const bool &use_viscosity);

	static void get_surface_force(const GU_Detail *gdp, const GA_Offset &primoff, const UT_Vector4i &tetra_points, Data_struct &ds);
	static void element_projection(const GU_Detail *gdp, const GA_Primitive *prim, Data_struct_2D &ds);

	template<typename T> static void assemble_m1(T &ds, const Material &parms);
	template<typename T> static void assemble_m2(T &ds, const Material &parms);
	template<typename T> static void assemble_E(T &ds, const Material &parms);
	template<typename T> static void add_full_stress_vector(T &ds, const bool &use_vic);
	template<typename T> static void update_material_parms(const GU_Detail *gdp, const GA_Primitive *prim, T &ds, Material &parms, bool &attribute_present, bool &use_vic);
	template<typename T> static void form_stress_tensor(T &ds);
	template<typename T> static void assemble_linear_element_matrix(T &ds, bool use_vic);
	template<typename T> static void  assemble_non_linear_element_matrix(T &ds);
	template<typename T> static void inject_element_matrix(const GU_Detail *gdp, unsigned int i, unsigned int j, T &ds, const Wilson &wil);
	template<typename T> static void step_solution_imp(GU_Detail *gdp, T &ds, const Wilson &wil, const Material &material);
	template<typename T> static void step_solution_exp(GU_Detail *gdp, T &ds, Material &parms, const Wilson &wil);

	static double point_to_prim(const GA_Primitive *prim, GA_ROHandleD &attrib_handle, const Data_struct &ds);
	static double point_to_prim(const GA_Primitive *prim, GA_ROHandleD &attrib_handle, const Data_struct_2D &ds);

	static void global_indices(GU_Detail *gdp, Data_struct &ds);
	static void global_indices(GU_Detail *gdp, Data_struct_2D &ds);

	static void apply_inertia( Data_struct &ds, const GU_Detail *gdp, const Wilson &wil);
	static void apply_inertia(Data_struct_2D &ds, const GU_Detail *gdp, const Wilson &wil);

	static void apply_boundary_conditions(GU_Detail *gdp, GA_Primitive *prim, Data_struct &ds);
};

#endif // !ASSEMBLE_M2H
