#include "GETFEMMatrices.h"
#include "Placeholder.h"
#include "MaterialProperties.h"
#include "Collision.h"

#include <SIM/SIM_Object.h>
#include <SIM/SIM_ObjectArray.h>

#include "templates.cpp"

gridUpdate::gridUpdate()
{

}

void gridUpdate::update_tetrapoints(GU_Detail *gdp, const GA_Primitive *prim, Data_struct &ds)
{
	GA_RWHandleV3D sf(gdp->findAttribute(GA_ATTRIB_POINT, "surface_force"));
	GA_ROHandleI winklerp(gdp->findAttribute(GA_ATTRIB_POINT, "winkler"));
	GA_ROHandleI corep(gdp->findAttribute(GA_ATTRIB_POINT, "core"));
	UT_Vector3 zero(0.0);
	for (unsigned int i = 0; i < 4; i++)
	{
		GA_Index index = prim->getPointIndex(i);
		ds.tetra_points(i) = index;
		UT_Vector4i offsets;
		GA_Offset ptoff = gdp->getPointMap().offsetFromIndex(GA_Index(ds.tetra_points(i)));
		ds.tetra_points_offset(i) = ptoff;
		bool is_surface = Collider::is_surface(gdp, ptoff, "surface_points");
		

		if (is_surface)
			ds.bound(i) = 1;
		else
		{
			ds.bound(i) = 0;
			if(sf.isValid())
				sf.set(ptoff, zero);
		}
		if (winklerp.isValid())
		{
			if (winklerp.get(ptoff))
				ds.bound(i) = 2;
		}
		if (corep.isValid())
		{
			if (corep.get(ptoff))
				ds.bound(i) = 3;
		}

	}
}
void gridUpdate::update_tetrapoints(GU_Detail *gdp, const GA_Primitive *prim, Data_struct_2D &ds)
{
	GA_RWHandleV3D sf(gdp->findAttribute(GA_ATTRIB_POINT, "surface_force"));
	GA_ROHandleI winklerp(gdp->findAttribute(GA_ATTRIB_POINT, "winkler"));
	GA_ROHandleI corep(gdp->findAttribute(GA_ATTRIB_POINT, "core"));
	
	UT_Vector3 zero(0.0);
	for (unsigned int i = 0; i < 3; i++)
	{
		GA_Index index = prim->getPointIndex(i);
		ds.tetra_points(i) = index;
		UT_Vector4i offsets;
		GA_Offset ptoff = gdp->getPointMap().offsetFromIndex(GA_Index(ds.tetra_points(i)));
		ds.tetra_points_offset(i) = ptoff;
		bool is_surface = Collider::is_surface(gdp, ptoff, "surface_points");
		if (is_surface)
			ds.bound(i) = 1;
		else
		{
			ds.bound(i) = 0;
			if(sf.isValid())
				sf.set(ptoff, zero);
		}
		if (winklerp.isValid())
		{
			if (winklerp.get(ptoff))
				ds.bound(i) = 2;
		}
		if (corep.isValid())
		{
			if (corep.get(ptoff))
				ds.bound(i) = 3;
		}

	}
}

void gridUpdate::get_surface_force(const GU_Detail *gdp, const GA_Offset &primoff, const UT_Vector4i &tetra_points, Data_struct &ds)
{
	GA_ROHandleV3D force_handle(gdp->findAttribute(GA_ATTRIB_POINT, "surface_force"));
	GA_ROHandleI surface_handle(gdp->findAttribute(GA_ATTRIB_POINT, "surface"));
	unsigned int point_range = gdp->getPointRange().getEntries();
	if (force_handle.isValid() && surface_handle.isValid())
	{
		const GA_Primitive *prim = gdp->getPrimitive(primoff);
		int count = 0;
	}
}

void gridUpdate::global_indices(GU_Detail *gdp, Data_struct &ds)
{
	unsigned int point_count = gdp->getNumPoints();
	ds.global_index(0) = ds.tetra_points(0);
	ds.global_index(1) = ds.tetra_points(0) + point_count;
	ds.global_index(2) = ds.tetra_points(0) + 2*point_count;
	ds.global_index(3) = ds.tetra_points(1);
	ds.global_index(4) = ds.tetra_points(1) + point_count;
	ds.global_index(5) = ds.tetra_points(1) + 2*point_count;
	ds.global_index(6) = ds.tetra_points(2);
	ds.global_index(7) = ds.tetra_points(2) + point_count;
	ds.global_index(8) = ds.tetra_points(2) + 2*point_count;
	ds.global_index(9) = ds.tetra_points(3);
	ds.global_index(10) = ds.tetra_points(3) + point_count;
	ds.global_index(11) = ds.tetra_points(3) + 2*point_count;

	ds.global_bound_index(0) = ds.tetra_points(0);
	ds.global_bound_index(1) = ds.tetra_points(0);
	ds.global_bound_index(2) = ds.tetra_points(0);
	ds.global_bound_index(3) = ds.tetra_points(1);
	ds.global_bound_index(4) = ds.tetra_points(1);
	ds.global_bound_index(5) = ds.tetra_points(1);
	ds.global_bound_index(6) = ds.tetra_points(2);
	ds.global_bound_index(7) = ds.tetra_points(2);
	ds.global_bound_index(8) = ds.tetra_points(2);
	ds.global_bound_index(9) = ds.tetra_points(3);
	ds.global_bound_index(10) = ds.tetra_points(3);
	ds.global_bound_index(11) = ds.tetra_points(3);
}
void gridUpdate::global_indices(GU_Detail *gdp, Data_struct_2D &ds)
{
	unsigned int point_count = gdp->getNumPoints();
	ds.global_index(0) = ds.tetra_points(0);
	ds.global_index(1) = ds.tetra_points(0) + point_count;
	ds.global_index(2) = ds.tetra_points(0) + 2 * point_count;
	ds.global_index(3) = ds.tetra_points(1);
	ds.global_index(4) = ds.tetra_points(1) + point_count;
	ds.global_index(5) = ds.tetra_points(1) + 2 * point_count;
	ds.global_index(6) = ds.tetra_points(2);
	ds.global_index(7) = ds.tetra_points(2) + point_count;
	ds.global_index(8) = ds.tetra_points(2) + 2 * point_count;
	

	ds.global_bound_index(0) = ds.tetra_points_offset(0);
	ds.global_bound_index(1) = ds.tetra_points_offset(0);
	ds.global_bound_index(2) = ds.tetra_points_offset(0);
	ds.global_bound_index(3) = ds.tetra_points_offset(1);
	ds.global_bound_index(4) = ds.tetra_points_offset(1);
	ds.global_bound_index(5) = ds.tetra_points_offset(1);
	ds.global_bound_index(6) = ds.tetra_points_offset(2);
	ds.global_bound_index(7) = ds.tetra_points_offset(2);
	ds.global_bound_index(8) = ds.tetra_points_offset(2);
}

void gridUpdate::assemble_global_implicit(const SIM_Object& object, GU_Detail *gdp, const GA_Offset &primoff, Data_struct &ds, const Material &parms, const Wilson &wil, const bool &use_viscosity)
{
	GA_RWHandleM3D stress_handle(gdp->findAttribute(GA_ATTRIB_PRIMITIVE, "cauchy_stress"));

	GA_ROHandleV3D force_handle(gdp->findAttribute(GA_ATTRIB_POINT, "volume_force"));
	GA_ROHandleV3D lp_handle(gdp->findAttribute(GA_ATTRIB_POINT, "P0"));
	GA_ROHandleV3D pp_handle(gdp->findAttribute(GA_ATTRIB_POINT, "P"));
	GA_ROHandleV3D vp_handle(gdp->findAttribute(GA_ATTRIB_POINT, "v"));
	GA_ROHandleV3D sf(gdp->findAttribute(GA_ATTRIB_POINT, "surface_force"));
	GA_ROHandleI winklerp(gdp->findAttribute(GA_ATTRIB_POINT, "winkler"));


	if (stress_handle.isValid() && vp_handle.isValid() && pp_handle.isValid() && lp_handle.isValid())
	{
		unsigned int point_range = gdp->getPointRange().getEntries();

		double prim_surf_area = 0;
		double a = ds.determinant / 120.0;
		double b = ds.determinant / 24.0;
		double c = ds.surf_area / 12.0;
	
		GA_Primitive *prim = gdp->getPrimitive(primoff);
		UT_Vector point_values(0, 11);
		unsigned int i = 0;
		unsigned int bi = 0;

		for (GA_Iterator pointit(prim->getPointRange()); !pointit.atEnd(); ++pointit)
		{		
			for (unsigned int j = 0; j < 3; j++)
			{
				point_values(3 * i + j) = pp_handle(ds.tetra_points_offset(i))(j) - lp_handle(ds.tetra_points_offset(i))(j);
			}
			i += 1;
		}

		UT_Vector strain_vector(0,5);
		ds.stress = stress_handle.get(primoff);
		ds.B.postMult(point_values, strain_vector);
		ds.sigma.postMult(strain_vector, ds.stress_vector);
		
		add_full_stress_vector<Data_struct>(ds, use_viscosity);
		form_stress_tensor<Data_struct>(ds);
		stress_handle.set(primoff, ds.stress);

		point_values.zero();
		ds.BT.postMult(ds.stress_vector, point_values);

		assemble_linear_element_matrix<Data_struct>(ds, use_viscosity);
		if(wil.theta != 1)
			assemble_non_linear_element_matrix<Data_struct>(ds);

		ds.non_linear_element_3D = ds.element_stiffness_matrix_nl;
		ds.non_linear_element_3D.addScaledMatrix(ds.element_stiffness_matrix, 1);
	
		unsigned int q = 0;
		unsigned int i_1 = 0;
		double mass = 5*parms.density;
		
		for (GA_Iterator pointit(prim->getPointRange()); !pointit.atEnd(); ++pointit)
		{
			GA_Offset ptoff = *pointit;
			GA_Index index = pointit.getIndex();
			UT_Vector3D force_vector(0.0);
			UT_Vector3D surface_force_vector(0.0);

						
			ds.lumped_mass(index) += a*mass;

			if (sf.isValid())
			{
				surface_force_vector = ds.bound(i_1)*(sf(ds.tetra_points_offset(i_1)) + sf(ds.tetra_points_offset(0)) + sf(ds.tetra_points_offset(1)) + sf(ds.tetra_points_offset(2)) + sf(ds.tetra_points_offset(3)));
			}

			if (force_handle.isValid())
			{
				force_vector = force_handle(ds.tetra_points_offset(i_1)) + force_handle(ds.tetra_points_offset(0)) + force_handle(ds.tetra_points_offset(1)) + force_handle(ds.tetra_points_offset(2)) + force_handle(ds.tetra_points_offset(3));
			}
			UT_Vector3 external_force(0.0);
			Collider::apply_external_forces(object, mass, pp_handle(ds.tetra_points_offset(i_1)), vp_handle(ds.tetra_points_offset(i_1)), external_force);

			
			
			for (unsigned int j = 0; j < 3; j++)
			{
				ds.global_force(ds.global_index(q)) -= c*(surface_force_vector(j));
				ds.global_force(ds.global_index(q)) += a*(force_vector(j) + external_force(j));
				ds.global_force(ds.global_index(q)) -= b*point_values(3 * i_1 + j);

				for (unsigned int w = 0; w < 12; w++)
				{
					inject_element_matrix<Data_struct>(gdp, w, q, ds, wil);
				}
				
				q += 1;
			}
			i_1 += 1;
		}
	}
}
void gridUpdate::assemble_global_implicit(const SIM_Object& object, GU_Detail *gdp, const GA_Offset &primoff, Data_struct_2D &ds, const Material &parms, const Wilson &wil, const bool &use_viscosity)
{
	GA_RWHandleM3D stress_handle(gdp->findAttribute(GA_ATTRIB_PRIMITIVE, "cauchy_stress"));

	GA_ROHandleV3D force_handle(gdp->findAttribute(GA_ATTRIB_POINT, "volume_force"));
	GA_ROHandleV3D lp_handle(gdp->findAttribute(GA_ATTRIB_POINT, "P0"));
	GA_ROHandleV3D pp_handle(gdp->findAttribute(GA_ATTRIB_POINT, "P"));
	GA_ROHandleV3D vp_handle(gdp->findAttribute(GA_ATTRIB_POINT, "v"));
	GA_ROHandleV3D sf(gdp->findAttribute(GA_ATTRIB_POINT, "surface_force"));

	if (stress_handle.isValid() && vp_handle.isValid() && pp_handle.isValid() && lp_handle.isValid())
	{
		unsigned int point_range = gdp->getPointRange().getEntries();

		double prim_surf_area = 0;
		double a = ds.determinant / 24.0;
		double b = ds.determinant / 6.0;
		double c = ds.surf_area / 6.0;
		
		GA_Primitive *prim = gdp->getPrimitive(primoff);
		UT_Vector point_values(0,8);

		unsigned int i = 0;
		unsigned int bi = 0;
	
		for (GA_Iterator pointit(prim->getPointRange()); !pointit.atEnd(); ++pointit)
		{
			
			for (unsigned int j = 0; j < 3; j++)
			{
				Collider::calculate_element_hessian(gdp, prim, ds, i, j);
				point_values(3 * i + j) = pp_handle(ds.tetra_points_offset(i))(j) - lp_handle(ds.tetra_points_offset(i))(j);
			}
			i += 1;
		}
	
		element_projection(gdp, prim, ds);
		UT_Vector strain_vector(0,3);
		ds.stress = stress_handle.get(primoff);
		UT_Vector BR(0,5);
		ds.projection.postMult(point_values, BR);
		
		ds.B.postMult(BR, strain_vector);
		ds.sigma.postMult(strain_vector, ds.stress_vector);
		
		add_full_stress_vector<Data_struct_2D>(ds, use_viscosity);
		form_stress_tensor<Data_struct_2D>(ds);
		
		stress_handle.set(primoff, ds.stress);

		point_values.zero();
		ds.BT.postMult(ds.stress_vector, BR);
		ds.projectionT.postMult(BR, point_values);

		assemble_linear_element_matrix<Data_struct_2D>(ds, use_viscosity);
		assemble_non_linear_element_matrix<Data_struct_2D>(ds);
	
		UT_Matrix P_tK_nl(0,8,0,5);
		P_tK_nl.zero();
		
		ds.element_stiffness_matrix_nl.addScaledMatrix(ds.element_stiffness_matrix, 1);
		ds.projectionT.postMult(ds.element_stiffness_matrix_nl, P_tK_nl);
		P_tK_nl.postMult(ds.projection, ds.non_linear_element_3D);
	
		unsigned int q = 0;
		unsigned int i_1 = 0;
		for (GA_Iterator pointit(prim->getPointRange()); !pointit.atEnd(); ++pointit)
		{
			GA_Offset ptoff = *pointit;
			GA_Index index = pointit.getIndex();
			UT_Vector3D force_vector(0.0);
			UT_Vector3D surface_force_vector(0.0);

			double mass = (4 * parms.density);
			ds.lumped_mass(index) += a*mass;

			if (sf.isValid())
			{
				surface_force_vector = ds.bound(i_1)*(sf(ds.tetra_points_offset(i_1)) + sf(ds.tetra_points_offset(0)) + sf(ds.tetra_points_offset(1)));
			}
			
			
			if (force_handle.isValid())
			{
				force_vector = force_handle(ds.tetra_points_offset(i_1)) + force_handle(ds.tetra_points_offset(0)) + force_handle(ds.tetra_points_offset(1)) + force_handle(ds.tetra_points_offset(2));
			}

			UT_Vector3 external_force(0.0);
			Collider::apply_external_forces(object, mass, pp_handle(ds.tetra_points_offset(i_1)), vp_handle(ds.tetra_points_offset(i_1)), external_force);

			
			for (unsigned int j = 0; j < 3; j++)
			{
				ds.global_force(ds.global_index(q)) += c*(surface_force_vector(j));
				ds.global_force(ds.global_index(q)) += a*(force_vector(j) + external_force(j));
				ds.global_force(ds.global_index(q)) -= b*point_values(3 * i_1 + j);

				for (unsigned int w = 0; w < 9; w++)
				{
					inject_element_matrix<Data_struct_2D>(gdp, w, q, ds, wil);
				}
				q += 1;
			}
			i_1 += 1;
		}
		
	}
}

void gridUpdate::assemble_global(const SIM_Object& object, GU_Detail *gdp, const GA_Offset &primoff,  Data_struct &ds, const Material &parms, const bool &use_viscosity)
{
	GA_RWHandleM3D stress_handle(gdp->findAttribute(GA_ATTRIB_PRIMITIVE, "cauchy_stress"));
	GA_ROHandleV3D force_handle(gdp->findAttribute(GA_ATTRIB_POINT, "volume_force"));
	GA_ROHandleV3D sf(gdp->findAttribute(GA_ATTRIB_POINT, "surface_force"));
	GA_ROHandleV3D lp_handle(gdp->findAttribute(GA_ATTRIB_POINT, "P0"));
	GA_ROHandleV3D pp_handle(gdp->findAttribute(GA_ATTRIB_POINT, "P"));
	GA_ROHandleV3D vp_handle(gdp->findAttribute(GA_ATTRIB_POINT, "v"));
	if (stress_handle.isValid() && vp_handle.isValid() && pp_handle.isValid() && lp_handle.isValid())
	{
		unsigned int point_range = gdp->getPointRange().getEntries();
		
		double prim_surf_area = 0;
		double a = ds.determinant / 120.0;
		double b = ds.determinant / 24.0;
		double c = ds.surf_area / 12.0;
		GA_Primitive *prim = gdp->getPrimitive(primoff);
		unsigned int i = 0;
		UT_Vector3i boundary_points;
		unsigned int boundary_check = 0;
		UT_Vector point_values(0, 11);


		for (GA_Iterator pointit(prim->getPointRange()); !pointit.atEnd(); ++pointit)
		{
			GA_Offset ptoff = *pointit;
			bool is_b = Collider::is_surface(gdp, ptoff, "surface_points");
			if (is_b && i < 3)
			{
				boundary_check += 1;
				boundary_points(i) = ptoff;
			}
			
			for (unsigned int j = 0; j < 3; j++)
			{
				point_values(3 * i + j) = pp_handle(ds.tetra_points_offset(i))(j) - lp_handle(ds.tetra_points_offset(i))(j);
			}
			i += 1;
		}
		UT_Vector strain_vector(0,5);
		ds.stress = stress_handle.get(primoff);
		ds.B.postMult(point_values, strain_vector);
		ds.sigma.postMult(strain_vector, ds.stress_vector);
		add_full_stress_vector<Data_struct>(ds, use_viscosity);
		form_stress_tensor<Data_struct>(ds);
		stress_handle.set(primoff, ds.stress);
		
		point_values.zero();
		ds.BT.postMult(ds.stress_vector, point_values);

		unsigned int i_1 = 0;		
		for (GA_Iterator pointit(prim->getPointRange()); !pointit.atEnd(); ++pointit)
		{
			GA_Offset ptoff = *pointit;
			GA_Index ptindex = pointit.getIndex();
			
			double mass = (5 * parms.density);
			ds.lumped_mass(ptindex) += a*mass;

			UT_Vector3D force_vector(0.0);
			UT_Vector3D surface_force_vector(0.0);

			if (sf.isValid())
			{
				surface_force_vector = ds.bound(i_1)*(sf(ds.tetra_points_offset(i_1)) + sf(ds.tetra_points_offset(0)) + sf(ds.tetra_points_offset(1)) + sf(ds.tetra_points_offset(2)) + sf(ds.tetra_points_offset(3)));
			}
			if (force_handle.isValid())
			{
				force_vector = force_handle(ds.tetra_points_offset(i_1)) + force_handle(ds.tetra_points_offset(0)) + force_handle(ds.tetra_points_offset(1)) + force_handle(ds.tetra_points_offset(2)) + force_handle(ds.tetra_points_offset(3));
			}

			

			UT_Vector3 external_force(0.0);
			Collider::apply_external_forces(object, mass, pp_handle(ds.tetra_points_offset(i_1)), vp_handle(ds.tetra_points_offset(i_1)), external_force);

			for (unsigned int j = 0; j < 3; j++) 
			{
				ds.global_force(ptindex + j*(point_range)) += a*(force_vector(j) + external_force(j));
				ds.global_force(ptindex + j*(point_range)) -= b*point_values(3 * i_1 + j);
				ds.global_force(ptindex + j*(point_range)) -= c*(surface_force_vector(j));
				
			}
			i_1 += 1;	
		}
	}
}
void gridUpdate::assemble_global(const SIM_Object& object, GU_Detail *gdp, const GA_Offset &primoff, Data_struct_2D &ds, const Material &parms, const bool &use_viscosity)
{
	GA_RWHandleM3D stress_handle(gdp->findAttribute(GA_ATTRIB_PRIMITIVE, "cauchy_stress"));
	GA_ROHandleV3D force_handle(gdp->findAttribute(GA_ATTRIB_POINT, "volume_force"));
	GA_ROHandleV3D sf(gdp->findAttribute(GA_ATTRIB_POINT, "surface_force"));
	GA_ROHandleV3D lp_handle(gdp->findAttribute(GA_ATTRIB_POINT, "P0"));
	GA_ROHandleV3D pp_handle(gdp->findAttribute(GA_ATTRIB_POINT, "P"));
	GA_ROHandleV3D vp_handle(gdp->findAttribute(GA_ATTRIB_POINT, "v"));

	GA_RWHandleV3 TT(gdp->findAttribute(GA_ATTRIB_PRIMITIVE, "TEST"));

	if (stress_handle.isValid() && vp_handle.isValid() && pp_handle.isValid() && lp_handle.isValid())
	{
		unsigned int point_range = gdp->getPointRange().getEntries();
		double prim_surf_area = 0;
		double a = ds.determinant / 24.0;
		double b = ds.determinant / 6.0;
		double c = ds.surf_area / 6.0;


		GA_Primitive *prim = gdp->getPrimitive(primoff);
		UT_Vector point_values(0, 8);
		point_values.zero();

		unsigned int i = 0;
	
		for (GA_Iterator pointit(prim->getPointRange()); !pointit.atEnd(); ++pointit)
		{
			GA_Offset ptoff = *pointit;
			
			for (unsigned int j = 0; j < 3; j++)
			{
				point_values(3 * i + j) = pp_handle(ds.tetra_points_offset(i))(j) - lp_handle(ds.tetra_points_offset(i))(j);
			}
			i += 1;
		}
		
		UT_Matrix BR(0,3,0,8);
		BR.zero();

		UT_Vector point_values_2D(0,5);
		UT_Vector point_values_linear(0,8);
		UT_Vector point_values_non_linear(0,8);
		UT_Vector strain_vector(0, 3);

		element_projection(gdp, prim, ds);
		ds.stress = stress_handle.get(primoff);
		
		ds.B.postMult(ds.projection, BR);
		BR.postMult(point_values, strain_vector);
		
		ds.sigma.postMult(strain_vector, ds.stress_vector);
		add_full_stress_vector<Data_struct_2D>(ds, use_viscosity);
		form_stress_tensor<Data_struct_2D>(ds);
		stress_handle.set(primoff, ds.stress);
		ds.BT.postMult(ds.stress_vector, point_values_2D);
		ds.projectionT.postMult(point_values_2D, point_values_linear);
		
		unsigned int i_1 = 0;
		for (GA_Iterator pointit(prim->getPointRange()); !pointit.atEnd(); ++pointit)
		{
			GA_Offset ptoff = *pointit;
			GA_Index ptindex = pointit.getIndex();

			double mass = (4 * parms.density);
			ds.lumped_mass(ptindex) += a*mass;

			UT_Vector3D force_vector(0.0);
			if (force_handle.isValid())
			{
				force_vector = force_handle(ds.tetra_points_offset(i_1)) + force_handle(ds.tetra_points_offset(0)) + force_handle(ds.tetra_points_offset(1)) + force_handle(ds.tetra_points_offset(2));
			}
			UT_Vector3D surface_force_vector(0.0);

			if (sf.isValid())
			{
				surface_force_vector = ds.bound(i_1)*(sf(ds.tetra_points_offset(i_1)) + sf(ds.tetra_points_offset(0)) + sf(ds.tetra_points_offset(1)));
			}

			UT_Vector3 external_force(0.0);
			Collider::apply_external_forces(object, mass, pp_handle(ds.tetra_points_offset(i_1)), vp_handle(ds.tetra_points_offset(i_1)), external_force);

			for (unsigned int j = 0; j < 3; j++)
			{
				ds.global_force(ptindex + j*(point_range)) += a*(force_vector(j) + external_force(j));
				ds.global_force(ptindex + j*(point_range)) -= b*point_values_linear(3 * i_1 + j);
				ds.global_force(ptindex + j*(point_range)) += c*surface_force_vector(j);
			}
			i_1 += 1;
		}
	}
}

void gridUpdate::update_jacobians(const GU_Detail *gdp, const GA_Primitive *prim, Data_struct &ds)
{	
	GA_ROHandleV3D pp(gdp->findAttribute(GA_ATTRIB_POINT, "P"));
	if (pp.isValid())
	{	
		UT_Vector3D p0 = pp.get(ds.tetra_points_offset(0));
		UT_Vector3D p1 = pp.get(ds.tetra_points_offset(1));
		UT_Vector3D p2 = pp.get(ds.tetra_points_offset(2));
		UT_Vector3D p3 = pp.get(ds.tetra_points_offset(3));
		
		ds.jacobian(0, 0) = 1;
		ds.jacobian(1, 0) = p0(0);
		ds.jacobian(2, 0) = p0(1);
		ds.jacobian(3, 0) = p0(2);

		ds.jacobian(0, 1) = 1;
		ds.jacobian(1, 1) = p1(0);
		ds.jacobian(2, 1) = p1(1);
		ds.jacobian(3, 1) = p1(2);

		ds.jacobian(0, 2) = 1;
		ds.jacobian(1, 2) = p2(0);
		ds.jacobian(2, 2) = p2(1);
		ds.jacobian(3, 2) = p2(2);

		ds.jacobian(0, 3) = 1;
		ds.jacobian(1, 3) = p3(0);
		ds.jacobian(2, 3) = p3(1);
		ds.jacobian(3, 3) = p3(2);

		ds.determinant = (ds.jacobian.determinant());
		ds.inverse_jacobian = ds.jacobian;
		ds.inverse_jacobian.invert();

		if(!ds.bound(0))
			ds.surf_area = 0.5*cross(p3 - p1, p2 - p1).length();
		else if(!ds.bound(1))
			ds.surf_area = 0.5*cross(p3 - p0, p2 - p0).length();
		else if (!ds.bound(2))
			ds.surf_area = 0.5*cross(p3 - p0, p1 - p0).length();
		else if (!ds.bound(3))
			ds.surf_area = 0.5*cross(p1 - p0, p2 - p0).length();

	}
}
void gridUpdate::update_jacobians(const GU_Detail *gdp, const GA_Primitive *prim, Data_struct_2D &ds)
{
	GA_ROHandleV3D pp(gdp->findAttribute(GA_ATTRIB_POINT, "P"));
	if (pp.isValid())
	{
		UT_Vector3D p0 = pp.get(ds.tetra_points_offset(0));
		UT_Vector3D p1 = pp.get(ds.tetra_points_offset(1));
		UT_Vector3D p2 = pp.get(ds.tetra_points_offset(2));

		fpreal a10 = p1(0) - p0(0);
		fpreal a11 = p1(1) - p0(1);
		fpreal a12 = p1(2) - p0(2);

		fpreal a20 = p2(0) - p0(0);
		fpreal a21 = p2(1) - p0(1);
		fpreal a22 = p2(2) - p0(2);
		
		ds.jacobian(0, 0) = 1;
		ds.jacobian(0, 1) = 1;
		ds.jacobian(0, 2) = 1;

		ds.jacobian(1, 0) = a10;
		ds.jacobian(1, 1) = a11;
		ds.jacobian(1, 2) = a12;

		ds.jacobian(2, 0) = a20;
		ds.jacobian(2, 1) = a21;
		ds.jacobian(2, 2) = a22;		

		ds.determinant = (ds.jacobian.determinant());
		ds.inverse_jacobian = ds.jacobian;
		ds.inverse_jacobian.invert();

		if (!ds.bound(0))
			ds.surf_area = (p2 - p1).length();
		else if (!ds.bound(1))
			ds.surf_area = (p2 - p0).length();
		else if (!ds.bound(2))
			ds.surf_area = (p1 - p0).length();
		
	}
}

void gridUpdate::element_projection(const GU_Detail *gdp, const GA_Primitive *prim, Data_struct_2D &ds)
{
	ds.projection.zero();
	ds.projectionT.zero();
	GA_ROHandleV3D pp(gdp->findAttribute(GA_ATTRIB_POINT, "P"));
	if (pp.isValid())
	{
		UT_Vector3D p0 = pp.get(ds.tetra_points_offset(0));
		UT_Vector3D p1 = pp.get(ds.tetra_points_offset(1));
		UT_Vector3D p2 = pp.get(ds.tetra_points_offset(2));
		
		UT_Vector3D projection_x = (p1 - p0);
		UT_Vector3D projection_w = (p2 - p0);
		projection_x.normalize();
		projection_w.normalize();
		UT_Vector3D projection_n = cross(projection_x, projection_w);
		UT_Vector3D projection_y = cross(projection_n, projection_x);

		projection_x.normalize();
		projection_y.normalize();

		for (unsigned int i = 0; i < 3; i++)
		{
			ds.projection(0, i) = projection_x(i);
			ds.projection(1, i) = projection_y(i);

			ds.projection(2, 3+i) = projection_x(i);
			ds.projection(3, 3+i) = projection_y(i);

			ds.projection(4, 6+i) = projection_x(i);
			ds.projection(5, 6+i) = projection_y(i);
		}
		ds.projection.transpose(ds.projectionT);
	}
}

 template<typename T> void gridUpdate::assemble_m1(T &ds, const Material &parms)
{
	ds.M1.zero();
	unsigned int size = ds.sigma.columns();
	unsigned int loop_max = 6;
	if (size < 6)
		loop_max = 4;

	unsigned int bound = loop_max / 2;
	float tau = 2 * parms.viscosity*(1 + parms.poisson_ratio) / parms.youngs_modulus;
	
	float aa = (1 - exp(-parms.dt / tau))  * (2.0 / 3.0);
	float ab = (1 - exp(-parms.dt / tau)) * (-1.0 / 3.0);
	float ac = (1 - exp(-parms.dt / tau)) * (1.0);

	for (unsigned int i = 0; i < loop_max; i++)
	{
		for (unsigned int j = 0; j < loop_max; j++)
		{
			if (i == j && i < bound)
				ds.M1(i, j) = 1.0 - aa;
			if (i == j && i >= bound)
				ds.M1(i, j) = 1.0 - ac;
			if (i != j && i < bound && j < bound)
				ds.M1(i,j) = -ab;
		}
	}
};
 template<typename T> void gridUpdate::assemble_m2(T &ds, const Material &parms)
{
	ds.M2.zero();
	unsigned int size = ds.sigma.columns();
	unsigned int loop_max = 6;
	if (size < 6)
		loop_max = 4;
	unsigned int bound = loop_max / 2;
	float ah = parms.youngs_modulus / (3.0 * (1.0 - 2.0 * parms.poisson_ratio));
	float bh = parms.youngs_modulus / (3.0 * (1.0 - 2.0 * parms.poisson_ratio));

	double tau = 2.0 * parms.viscosity*(1 + parms.poisson_ratio) / parms.youngs_modulus;

	float af = 4.0 * parms.youngs_modulus / (6.0 * (1.0 + parms.poisson_ratio)) * (tau / parms.dt)*(1.0 - exp(-parms.dt / tau));
	float bf = -2.0 * parms.youngs_modulus / (6.0 * (1.0 + parms.poisson_ratio)) * (tau / parms.dt)*(1.0 - exp(-parms.dt / tau));
	float cf = 3.0 * parms.youngs_modulus / (6.0 * (1.0 + parms.poisson_ratio)) * (tau / parms.dt)*(1.0 - exp(-parms.dt / tau));
	
	for(unsigned int i = 0; i < loop_max; i++)
	{
		for(unsigned int j = 0; j < loop_max; j++)
		{
			if (i == j && i < bound)
				ds.M2(i, j) = af + ah;
			if (i == j && i >= bound)
				ds.M2(i, j) = cf + bh;
			if (i != j && i < bound && j < bound)
				ds.M2(i, j) = bf + bh;
		}
	}
};
 template<typename T> void gridUpdate::assemble_E(T &ds, const Material &parms)
{
	ds.sigma.zero();
	unsigned int size = ds.sigma.columns();
	unsigned int loop_max = 6;
	if (size < 6)
		loop_max = 4;
	
	unsigned int bound = loop_max / 2;

	float diagonal1 = 2 * (1 - parms.poisson_ratio) / (1 - 2 * parms.poisson_ratio);
	float diagonal2 = 1.0;
	float diagonal3 = 2 * parms.poisson_ratio / (1 - 2 * parms.poisson_ratio);
	float multiplier = parms.youngs_modulus / (2 * (1 + parms.poisson_ratio));

	float diag1 = diagonal1 * multiplier;
	float diag2 = diagonal2 * multiplier;
	float diag3 = diagonal3 * multiplier;

	for (unsigned int i = 0; i < loop_max; i++)
	{
		for (unsigned int j = 0; j < loop_max; j++)
		{
			if (i == j && i < bound)
				ds.sigma(i, j) = diag1;
			if (i == j && i >= bound)
				ds.sigma(i, j) = diag2;
			if (i != j && i < bound && j < bound)
				ds.sigma(i, j) = diag3;
		}
	}
};
 template<typename T> void gridUpdate::update_material_parms(const GU_Detail *gdp, const GA_Primitive *prim, T &ds, Material &parms, bool &attribute_present, bool &use_vic)
 {

	 GA_ROHandleD densityp(gdp->findAttribute(GA_ATTRIB_PRIMITIVE, "density"));
	 GA_ROHandleD viscosityp(gdp->findAttribute(GA_ATTRIB_PRIMITIVE, "viscosity"));
	 GA_ROHandleD poissonp(gdp->findAttribute(GA_ATTRIB_PRIMITIVE, "poisson_ratio"));
	 GA_ROHandleD youngp(gdp->findAttribute(GA_ATTRIB_PRIMITIVE, "Y"));
	 GA_Offset primoff = prim->getMapOffset();

	 if (parms.viscosity < 0.0000001)
		 use_vic = false;
	 else
		use_vic = true;

	 if (viscosityp.isValid())
		 parms.viscosity = abs(viscosityp(primoff)), attribute_present = true;

	 if (poissonp.isValid())
		 parms.poisson_ratio = abs(poissonp(primoff)), attribute_present = true;

	 if (youngp.isValid())
		 parms.youngs_modulus = abs(youngp(primoff)), attribute_present = true;

	 if (densityp.isValid())
		 parms.density = abs(densityp(primoff)), attribute_present = true;

	 if (parms.poisson_ratio >= 0.5)
		 parms.poisson_ratio = 0.499;

	 if (parms.poisson_ratio <= 0)
		 parms.poisson_ratio = 0.001;
 }
 template<typename T> void gridUpdate::add_full_stress_vector(T &ds, const bool &use_vic)
 {
	 if (ds.stress_vector.length() == 6)
	 {
		 ds.stress_vector(0) += ds.stress(0, 0);
		 ds.stress_vector(1) += ds.stress(1, 1);
		 ds.stress_vector(2) += ds.stress(2, 2);

		 ds.stress_vector(3) += ds.stress(1, 2);
		 ds.stress_vector(4) += ds.stress(2, 0);
		 ds.stress_vector(5) += ds.stress(0, 1);
	 }
	 if (ds.stress_vector.length() == 4)
	 {
		 ds.stress_vector(0) += ds.stress(0, 0);
		 ds.stress_vector(1) += ds.stress(1, 1);
		 ds.stress_vector(2) += ds.stress(0, 1);
		 ds.stress_vector(3) += ds.stress(2, 2);
	 }

	 if (use_vic)
	 {
		 ds.M1.postMult(ds.stress_vector, ds.m1_holder);
		 ds.stress_vector = ds.m1_holder;
	 }
 }
 template<typename T> void gridUpdate::form_stress_tensor(T &ds)
 {
	 if (ds.stress_vector.length() == 6)
	 {
		 ds.stress(0, 0) = ds.stress_vector(0);
		 ds.stress(1, 1) = ds.stress_vector(1);
		 ds.stress(2, 2) = ds.stress_vector(2);

		 ds.stress(0, 2) = ds.stress_vector(4);
		 ds.stress(2, 0) = ds.stress_vector(4);

		 ds.stress(1, 0) = ds.stress_vector(5);
		 ds.stress(0, 1) = ds.stress_vector(5);

		 ds.stress(2, 1) = ds.stress_vector(3);
		 ds.stress(1, 2) = ds.stress_vector(3);
	 }

	 if (ds.stress_vector.length() == 4)
	 {
		 ds.stress(0, 0) = ds.stress_vector(0);
		 ds.stress(1, 1) = ds.stress_vector(1);
		 ds.stress(0, 1) = ds.stress_vector(2);
		 ds.stress(1, 0) = ds.stress_vector(2);
		 ds.stress(2, 2) = ds.stress_vector(3);
	 }

 }
 template<typename T> void gridUpdate::assemble_linear_element_matrix(T &ds, bool use_vic)
 {
	 UT_MatrixD B_temp;
	 if (ds.stress_vector.length() == 4)
		 B_temp.init(0, 3, 0, 5);
	 else
		 B_temp.init(0, 5, 0, 11);

	 if (!use_vic)
		 ds.sigma.postMult(ds.B, B_temp);
	 else
		 ds.M2.postMult(ds.B, B_temp);

	 ds.element_stiffness_matrix.zero();
	 ds.BT.postMult(B_temp, ds.element_stiffness_matrix);
 }
 template<typename T> void gridUpdate::assemble_non_linear_element_matrix(T &ds)
 {
	 if(ds.stress_vector.length() == 6)
	 {
		 for (unsigned int q = 0; q < 9; q += 3)
		 {
			 for (unsigned int i = 0; i < 3; i++)
			 {
				 for (unsigned int j = 0; j < 3; j++)
				 {
					 ds.stress_strain_matrix(q + i, q + j) = ds.stress(i, j);
				 }
			 }
		 }

		 UT_MatrixD BNL_temp(0, 8, 0, 11);
		 BNL_temp.zero();

		 ds.stress_strain_matrix.postMult(ds.BNL, BNL_temp);
		 ds.BNLT.postMult(BNL_temp, ds.element_stiffness_matrix_nl);
	 }


	 if(ds.stress_vector.length() == 4)
	 {
		 for (unsigned int q = 0; q < 4; q += 2)
		 {
			 for (unsigned int i = 0; i < 2; i++)
			 {
				 for (unsigned int j = 0; j < 2; j++)
				 {
					 ds.stress_strain_matrix(q + i, q + j) = ds.stress(i, j);
				 }
			 }
		 }
		 ds.stress_strain_matrix(4, 4) = ds.stress(2, 2);

		 UT_MatrixD BNL_temp(0, 4, 0, 5);
		 BNL_temp.zero();

		 ds.stress_strain_matrix.postMult(ds.BNL, BNL_temp);
		 ds.BNLT.postMult(BNL_temp, ds.element_stiffness_matrix_nl);
	 }
	
 }
 template<typename T> void gridUpdate::inject_element_matrix(const GU_Detail *gdp, unsigned int i, unsigned int j, T &ds, const Wilson &wil)
 {

	 GA_ROHandleV3 pin_p(gdp->findAttribute(GA_ATTRIB_POINT, "pintoanimation"));

	 unsigned int point_count = gdp->getPointRange().getEntries();

	 fpreal value = 0;
	 if(ds.stress_vector.length() == 4)
		value = ds.determinant*0.5*(ds.non_linear_element_3D(i, j));
	 else
		 value = ds.determinant*1/6.0*(ds.non_linear_element_3D(i, j));

	 unsigned int x = ds.global_index(i);
	 unsigned int y = ds.global_index(j);

	 unsigned int offset1 = ds.global_bound_index(i);
	 unsigned int offset2 = ds.global_bound_index(j);
	 
	 if (pin_p.isValid())
	 {
		 UT_Vector3 boundsx = pin_p.get(offset1);
		 UT_Vector3 boundsy = pin_p.get(offset2);
		 
		 if (boundsx(0))
		 {
			 if (x < point_count)
			 {
				 if (x != y)
				 {
					 ds.global_force(x) -= value;
				 }
				 value = 0;
			 }
		 }
		 
		 if (boundsy(0) == 1 && y < point_count)
		 {
			 value = 0;
		 }
		 if (boundsx(1) == 1)
		 {
			 if (point_count <= x && x < 2 * point_count)
			 {
				 if (x != y)
				 {
					 ds.global_force(x) -= value;
				 }
				 value = 0;
			 }
		 }
		 if (boundsy(1) == 1 && point_count <= y && y < 2 * point_count)
		 {
			 value = 0;
		 }
		 if (boundsx(2) == 1)
		 {
			 if (x >= 2 * point_count)
			 {
				 if (x != y)
				 {
					 ds.global_force(x) -= value;
				 }

				 value = 0;
			 }
		 }
		 if (boundsy(2) == 1 && y >= 2 * point_count)
		 {
			 value = 0;
		 }

	 }
	 ds.global_stiffness.addToElement(x, y, value);

 }

double gridUpdate::point_to_prim(const GA_Primitive *prim, GA_ROHandleD &attrib_handle, const Data_struct &ds)
 {
	 fpreal prim_attrib = 0;
	
	 for (GA_Iterator pointit(prim->getPointRange()); !pointit.atEnd(); ++pointit)
	 {
		 GA_Offset ptoff = *pointit;
		 prim_attrib += attrib_handle.get(ptoff);
	 }
	 prim_attrib *= 0.25;
	 return prim_attrib;
 }
double gridUpdate::point_to_prim(const GA_Primitive *prim, GA_ROHandleD &attrib_handle, const Data_struct_2D &ds)
{
	fpreal prim_attrib = 0;

	for (GA_Iterator pointit(prim->getPointRange()); !pointit.atEnd(); ++pointit)
	{
		GA_Offset ptoff = *pointit;
		prim_attrib += attrib_handle.get(ptoff);
	}
	prim_attrib *= 0.25;
	return prim_attrib;
}

void gridUpdate::assemble_B(Data_struct &ds)
{
	ds.B.zero();

	ds.B(0, 0) = ds.inverse_jacobian(0, 1);
	ds.B(0, 3) = ds.inverse_jacobian(1, 1);
	ds.B(0, 6) = ds.inverse_jacobian(2, 1);
	ds.B(0, 9) = ds.inverse_jacobian(3, 1);

	ds.B(1, 1 + 0) = ds.inverse_jacobian(0, 2);
	ds.B(1, 1 + 3) = ds.inverse_jacobian(1, 2);
	ds.B(1, 1 + 6) = ds.inverse_jacobian(2, 2);
	ds.B(1, 1 + 9) = ds.inverse_jacobian(3, 2);

	ds.B(2, 2 + 0) = ds.inverse_jacobian(0, 3);
	ds.B(2, 2 + 3) = ds.inverse_jacobian(1, 3);
	ds.B(2, 2 + 6) = ds.inverse_jacobian(2, 3);
	ds.B(2, 2 + 9) = ds.inverse_jacobian(3, 3);
	ds.B(3, 1 + 0) = ds.inverse_jacobian(0, 1);
	ds.B(3, 1 + 3) = ds.inverse_jacobian(1, 1);
	ds.B(3, 1 + 6) = ds.inverse_jacobian(2, 1);
	ds.B(3, 1 + 9) = ds.inverse_jacobian(3, 1);

	ds.B(3, 0) = ds.inverse_jacobian(0, 2);
	ds.B(3, 3) = ds.inverse_jacobian(1, 2);
	ds.B(3, 6) = ds.inverse_jacobian(2, 2);
	ds.B(3, 9) = ds.inverse_jacobian(3, 2);

	ds.B(4, 1 + 0) = ds.inverse_jacobian(0, 3);
	ds.B(4, 1 + 3) = ds.inverse_jacobian(1, 3);
	ds.B(4, 1 + 6) = ds.inverse_jacobian(2, 3);
	ds.B(4, 1 + 9) = ds.inverse_jacobian(3, 3);

	ds.B(4, 2 + 0) = ds.inverse_jacobian(0, 2);
	ds.B(4, 2 + 3) = ds.inverse_jacobian(1, 2);
	ds.B(4, 2 + 6) = ds.inverse_jacobian(2, 2);
	ds.B(4, 2 + 9) = ds.inverse_jacobian(3, 2);

	ds.B(5, 2 + 0) = ds.inverse_jacobian(0, 1);
	ds.B(5, 2 + 3) = ds.inverse_jacobian(1, 1);
	ds.B(5, 2 + 6) = ds.inverse_jacobian(2, 1);
	ds.B(5, 2 + 9) = ds.inverse_jacobian(3, 1);

	ds.B(5, 0) = ds.inverse_jacobian(0, 3);
	ds.B(5, 3) = ds.inverse_jacobian(1, 3);
	ds.B(5, 6) = ds.inverse_jacobian(2, 3);
	ds.B(5, 9) = ds.inverse_jacobian(3, 3);

	ds.B.transpose(ds.BT);
};
void gridUpdate::assemble_B(Data_struct_2D &ds)
{
	ds.B.zero();

	ds.B(0, 0) = ds.inverse_jacobian(0, 1);
	ds.B(0, 2) = ds.inverse_jacobian(1, 1);
	ds.B(0, 4) = ds.inverse_jacobian(2, 1);
	
	ds.B(1, 1 + 0) = ds.inverse_jacobian(0, 2);
	ds.B(1, 1 + 2) = ds.inverse_jacobian(1, 2);
	ds.B(1, 1 + 4) = ds.inverse_jacobian(2, 2);

	ds.B(2, 0) = ds.inverse_jacobian(0, 2);
	ds.B(2, 1) = ds.inverse_jacobian(0, 1);

	ds.B(2, 2) = ds.inverse_jacobian(1, 2);
	ds.B(2, 3) = ds.inverse_jacobian(1, 1);

	ds.B(2, 4) = ds.inverse_jacobian(2, 2);
	ds.B(2, 5) = ds.inverse_jacobian(2, 1);

	fpreal plane = 1 / 3.0;
	ds.B(3, 0) = plane;
	ds.B(3, 2) = plane;
	ds.B(3, 4) = plane;

	ds.B.transpose(ds.BT);
};

void gridUpdate::assemble_BNL(Data_struct &ds)
{
	ds.BNL.zero();

	ds.BNL(0, 0) = ds.inverse_jacobian(0, 1);
	ds.BNL(0, 3) = ds.inverse_jacobian(1, 1);
	ds.BNL(0, 6) = ds.inverse_jacobian(2, 1);
	ds.BNL(0, 9) = ds.inverse_jacobian(3, 1);

	ds.BNL(1, 0) = ds.inverse_jacobian(0, 2);
	ds.BNL(1, 3) = ds.inverse_jacobian(1, 2);
	ds.BNL(1, 6) = ds.inverse_jacobian(2, 2);
	ds.BNL(1, 9) = ds.inverse_jacobian(3, 2);

	ds.BNL(2, 0) = ds.inverse_jacobian(0, 3);
	ds.BNL(2, 3) = ds.inverse_jacobian(1, 3);
	ds.BNL(2, 6) = ds.inverse_jacobian(2, 3);
	ds.BNL(2, 9) = ds.inverse_jacobian(3, 3);

	ds.BNL(3, 1 + 0) = ds.inverse_jacobian(0, 1);
	ds.BNL(3, 1 + 3) = ds.inverse_jacobian(1, 1);
	ds.BNL(3, 1 + 6) = ds.inverse_jacobian(2, 1);
	ds.BNL(3, 1 + 9) = ds.inverse_jacobian(3, 1);

	ds.BNL(4, 1 + 0) = ds.inverse_jacobian(0, 2);
	ds.BNL(4, 1 + 3) = ds.inverse_jacobian(1, 2);
	ds.BNL(4, 1 + 6) = ds.inverse_jacobian(2, 2);
	ds.BNL(4, 1 + 9) = ds.inverse_jacobian(3, 2);

	ds.BNL(5, 1 + 0) = ds.inverse_jacobian(0, 3);
	ds.BNL(5, 1 + 3) = ds.inverse_jacobian(1, 3);
	ds.BNL(5, 1 + 6) = ds.inverse_jacobian(2, 3);
	ds.BNL(5, 1 + 9) = ds.inverse_jacobian(3, 3);

	ds.BNL(6, 2 + 0) = ds.inverse_jacobian(0, 1);
	ds.BNL(6, 2 + 3) = ds.inverse_jacobian(1, 1);
	ds.BNL(6, 2 + 6) = ds.inverse_jacobian(2, 1);
	ds.BNL(6, 2 + 9) = ds.inverse_jacobian(3, 1);

	ds.BNL(7, 2 + 0) = ds.inverse_jacobian(0, 2);
	ds.BNL(7, 2 + 3) = ds.inverse_jacobian(1, 2);
	ds.BNL(7, 2 + 6) = ds.inverse_jacobian(2, 2);
	ds.BNL(7, 2 + 9) = ds.inverse_jacobian(3, 2);

	ds.BNL(8, 2 + 0) = ds.inverse_jacobian(0, 3);
	ds.BNL(8, 2 + 3) = ds.inverse_jacobian(1, 3);
	ds.BNL(8, 2 + 6) = ds.inverse_jacobian(2, 3);
	ds.BNL(8, 2 + 9) = ds.inverse_jacobian(3, 3);

	

	ds.BNL.transpose(ds.BNLT);
};
void gridUpdate::assemble_BNL(Data_struct_2D &ds)
{
	ds.BNL.zero();

	ds.BNL(0, 0) = ds.inverse_jacobian(0, 1);
	ds.BNL(0, 2) = ds.inverse_jacobian(1, 1);
	ds.BNL(0, 4) = ds.inverse_jacobian(2, 1);

	ds.BNL(1, 0) = ds.inverse_jacobian(0, 2);
	ds.BNL(1, 2) = ds.inverse_jacobian(1, 2);
	ds.BNL(1, 4) = ds.inverse_jacobian(2, 2);

	ds.BNL(2, 1 + 0) = ds.inverse_jacobian(0, 1);
	ds.BNL(2, 1 + 2) = ds.inverse_jacobian(1, 1);
	ds.BNL(2, 1 + 4) = ds.inverse_jacobian(2, 1);

	ds.BNL(3, 1 + 0) = ds.inverse_jacobian(0, 2);
	ds.BNL(3, 1 + 2) = ds.inverse_jacobian(1, 2);
	ds.BNL(3, 1 + 4) = ds.inverse_jacobian(2, 2);

	double plane = 1 / 3.0;
	ds.BNL(4, 0) = plane;
	ds.BNL(4, 2) = plane;
	ds.BNL(4, 4) = plane;

	ds.BNL.transpose(ds.BNLT);
};

void gridUpdate::apply_inertia(Data_struct &ds, const GU_Detail *gdp, const Wilson &wil)
{
	GA_ROHandleV3 disp(gdp->findAttribute(GA_ATTRIB_POINT, "pintoanimation"));
	GA_ROHandleV3D vp_handle(gdp->findAttribute(GA_ATTRIB_POINT, "v"));
	GA_ROHandleV3D accp_handle(gdp->findAttribute(GA_ATTRIB_POINT, "acc"));
	unsigned int pointCount = gdp->getNumPoints();

	for (GA_Iterator point_iter(gdp->getPointRange()); !point_iter.atEnd(); ++point_iter)
	{
		const GA_Offset offset = *point_iter;
		const GA_Index index = point_iter.getIndex();
		UT_Vector3D acc = accp_handle(offset);
		UT_Vector3D v = vp_handle(offset);
		
		fpreal mass_contrib = 0;

		if(wil.theta != 1)
			mass_contrib = wil.a0*ds.lumped_mass(index);

		UT_Vector3D inertia;
	
		inertia = ds.lumped_mass(index)*(wil.a1*v + wil.a2*acc);
	
		if (disp.isInvalid())
		{
			ds.global_stiffness.addToElement(index, index, mass_contrib);
			ds.global_stiffness.addToElement(index + pointCount, index + pointCount, mass_contrib);
			ds.global_stiffness.addToElement(index + 2 * pointCount, index + 2 * pointCount, mass_contrib);
			
			ds.global_force(index) += inertia(0);
			ds.global_force(index + pointCount) += inertia(1);
			ds.global_force(index + 2 * pointCount) += inertia(2);
			
		}
		if (disp.isValid())
		{
			UT_Vector3 di = disp(offset);
			if (di(0) != 1)
			{
				ds.global_stiffness.addToElement(index, index, mass_contrib);
				ds.global_force(index) += inertia(0);
			}
			if (di(1) != 1)
			{
				ds.global_stiffness.addToElement(index + pointCount, index + pointCount, mass_contrib);
				ds.global_force(index + pointCount) += inertia(1);
			}
			if (di(2) != 1)
			{
				ds.global_stiffness.addToElement(index + 2 * pointCount, index + 2 * pointCount, mass_contrib);
				ds.global_force(index + 2 * pointCount) += inertia(2);
			}

			if (di(0) == 1)
			{
				ds.global_stiffness.addToElement(index, index, 1);
				ds.global_force(index) = 0;
			}

			if (di(1) == 1)
			{
				ds.global_stiffness.addToElement(index + pointCount, index + pointCount, 1);
				ds.global_force(index + pointCount) = 0;
			}

			if (di(2) == 1)
			{
				ds.global_stiffness.addToElement(index + 2 * pointCount, index + 2 * pointCount, 1);
				ds.global_force(index + 2 * pointCount) = 0;
			}
		}
	}
}
void gridUpdate::apply_inertia(Data_struct_2D &ds, const GU_Detail *gdp, const Wilson &wil)
{
	GA_ROHandleV3 disp(gdp->findAttribute(GA_ATTRIB_POINT, "pintoanimation"));
	GA_ROHandleV3D vp_handle(gdp->findAttribute(GA_ATTRIB_POINT, "v"));
	GA_ROHandleV3D accp_handle(gdp->findAttribute(GA_ATTRIB_POINT, "acc"));
	unsigned int pointCount = gdp->getNumPoints();

	for (GA_Iterator point_iter(gdp->getPointRange()); !point_iter.atEnd(); ++point_iter)
	{
		const GA_Offset offset = *point_iter;
		const GA_Index index = point_iter.getIndex();
		UT_Vector3D acc = accp_handle(offset);
		UT_Vector3D v = vp_handle(offset);

		fpreal mass_contrib = 0;
		
		if (wil.theta != 1)
			mass_contrib = wil.a0*ds.lumped_mass(index);

		UT_Vector3D inertia;

		inertia = ds.lumped_mass(index)*(wil.a1*v + wil.a2*acc);
		
		if (disp.isInvalid())
		{
			ds.global_stiffness.addToElement(index, index, mass_contrib);
			ds.global_stiffness.addToElement(index + pointCount, index + pointCount, mass_contrib);
			ds.global_stiffness.addToElement(index + 2 * pointCount, index + 2 * pointCount, mass_contrib);

			ds.global_force(index) += inertia(0);
			ds.global_force(index + pointCount) += inertia(1);
			ds.global_force(index + 2 * pointCount) += inertia(2);

		}
		
		if (disp.isValid())
		{
			UT_Vector3 di = disp(offset);
			if (di(0) != 1)
			{
				ds.global_stiffness.addToElement(index, index, mass_contrib);
				ds.global_force(index) += inertia(0);
			}
			if (di(1) != 1)
			{
				ds.global_stiffness.addToElement(index + pointCount, index + pointCount, mass_contrib);
				ds.global_force(index + pointCount) += inertia(1);
			}
			if (di(2) != 1)
			{
				ds.global_stiffness.addToElement(index + 2 * pointCount, index + 2 * pointCount, mass_contrib);
				ds.global_force(index + 2 * pointCount) += inertia(2);
			}
		
			
			if (di(0) == 1)
			{
				ds.global_stiffness.addToElement(index, index, 1);
				ds.global_force(index) = 0;
			}

			if (di(1) == 1)
			{
				ds.global_stiffness.addToElement(index + pointCount, index + pointCount, 1);
				ds.global_force(index + pointCount) = 0;
			}

			if (di(2) == 1)
			{
				ds.global_stiffness.addToElement(index + 2 * pointCount, index + 2 * pointCount, 1);
				ds.global_force(index + 2 * pointCount) = 0;
			}
		}
	}
}

template<typename T> void gridUpdate::step_solution_exp(GU_Detail *gdp, T &ds, Material &parms, const Wilson &wil)
{
	fpreal t_1 = pow(parms.dt, 2.0);
	unsigned int points = gdp->getNumPoints();
	GA_RWHandleV3D pp0(gdp->findAttribute(GA_ATTRIB_POINT, "P0"));
	GA_RWHandleV3D pp(gdp->findAttribute(GA_ATTRIB_POINT, "P"));
	GA_RWHandleV3D vp(gdp->findAttribute(GA_ATTRIB_POINT, "v"));
	GA_RWHandleV3D accp(gdp->findAttribute(GA_ATTRIB_POINT, "acc"));

	GA_RWHandleI pin(gdp->findAttribute(GA_ATTRIB_POINT, "pintoanimation"));
	if (pp0.isValid() && pp.isValid())
	{
		for (GA_Iterator pointit(gdp->getPointRange()); !pointit.atEnd(); ++pointit)
		{
			GA_Offset ptoff = *pointit;
			GA_Index index = pointit.getIndex();

			UT_Vector3D point_force(ds.global_force(index), ds.global_force(index + points), ds.global_force(index + 2 * points));
			UT_Vector3D p_next = 1.0 / ds.lumped_mass(index)*t_1*point_force +  2 * pp(ptoff) - pp0(ptoff);
			
			UT_Vector3D new_acc = (wil.a3*(p_next - pp(ptoff))  + wil.a4*vp(ptoff) + wil.a5*accp(ptoff));
			UT_Vector3D new_v = vp(ptoff) + wil.a6*(new_acc + accp(ptoff));
			
			vp.set(ptoff, new_v);
			accp.set(ptoff, new_acc);
			pp0.set(ptoff, pp(ptoff));
			if (pin.isValid())
			{
				if (!pin(ptoff))
					pp.set(ptoff, p_next);
			}
			else
				pp.set(ptoff, p_next);
		}
	}
}
template<typename T> void gridUpdate::step_solution_imp(GU_Detail *gdp, T &ds, const Wilson &wil, const Material &material)
{
	GA_RWHandleV3D accp(gdp->findAttribute(GA_ATTRIB_POINT, "acc"));
	GA_RWHandleV3D vp(gdp->findAttribute(GA_ATTRIB_POINT, "v"));
	GA_RWHandleV3D pp(gdp->findAttribute(GA_ATTRIB_POINT, "P"));
	GA_RWHandleV3D pp0(gdp->findAttribute(GA_ATTRIB_POINT, "P0"));

	unsigned int point_count = gdp->getNumPoints();

	if (vp.isValid() && accp.isValid())
	{
		for (GA_Iterator pointit(gdp->getPointRange()); !pointit.atEnd(); ++pointit)
		{
			GA_Offset ptoff = *pointit;
			GA_Index ptindex = pointit.getIndex();
			pp0.set(ptoff, pp(ptoff));
			if (wil.theta != 1)
			{
				
				fpreal u0 = ds.X(ptindex);
				fpreal u1 = ds.X(ptindex + point_count);
				fpreal u2 = ds.X(ptindex + 2 * point_count);
				UT_Vector3D u(u0, u1, u2);

				UT_Vector3D old_v = vp.get(ptoff);
				UT_Vector3D old_acc = accp.get(ptoff);
				UT_Vector3D old_u = pp.get(ptoff);

				UT_Vector3D new_acc = (wil.a3*u + wil.a4*old_v + wil.a5*old_acc);
				UT_Vector3D new_v = old_v + wil.a6*(new_acc + old_acc);
				UT_Vector3D new_u = old_u + material.dt*old_v + wil.a7*(new_acc + 2 * old_acc);

				pp.set(ptoff, new_u);
				vp.set(ptoff, new_v);
				accp.set(ptoff, new_acc);
			}
			else
			{
				fpreal u0 = ds.X(ptindex);
				fpreal u1 = ds.X(ptindex + point_count);
				fpreal u2 = ds.X(ptindex + 2 * point_count);
				UT_Vector3D u(u0, u1, u2);
				pp.add(ptoff, u);
			}
		}
	}
}



