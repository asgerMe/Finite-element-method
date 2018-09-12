#include "GETFEMMatrices.h"
#include "Placeholder.h"
#include "MaterialProperties.h"
#include "Collision.h"

#include <SIM/SIM_Object.h>
#include <SIM/SIM_ObjectArray.h>

gridUpdate::gridUpdate()
{

}

void gridUpdate::update_tetrapoints(const GU_Detail *gdp, const GA_Primitive *prim, Data_struct &ds)
{
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
			ds.bound(i) = 0;

	}
}
void gridUpdate::update_tetrapoints(const GU_Detail *gdp, const GA_Primitive *prim, Data_struct_2D &ds)
{
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
			ds.bound(i) = 0;

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

void gridUpdate::inject_element_matrix(const GU_Detail *gdp, unsigned int i, unsigned int j, Data_struct &ds, const Wilson &wil)
{

	GA_ROHandleV3 pin_p(gdp->findAttribute(GA_ATTRIB_POINT, "pintoanimation"));

	unsigned int point_count = gdp->getPointRange().getEntries();

	fpreal value = ds.determinant / 6.0*(ds.element_stiffness_matrix(i, j) + ds.element_stiffness_matrix_nl(i,j));
	
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

void gridUpdate::assemble_global_implicit(const SIM_Object& object, GU_Detail *gdp, const GA_Offset &primoff, Data_struct &ds, const Material &parms, const Wilson &wil, const bool &use_viscosity)
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
		double a = ds.determinant / 120.0;
		double b = ds.determinant / 24.0;
		double c = ds.surf_area / 6.0;

		GA_Primitive *prim = gdp->getPrimitive(primoff);
	
		unsigned int i = 0;
		unsigned int bi = 0;

		for (GA_Iterator pointit(prim->getPointRange()); !pointit.atEnd(); ++pointit)
		{
			GA_Offset ptoff = *pointit;
			GA_Index ptindex = pointit.getIndex();
			unsigned int index = ds.tetra_points(i);
			
			for (unsigned int j = 0; j < 3; j++)
			{
				ds.point_values(3 * i + j) = pp_handle(ds.tetra_points(i))(j) - lp_handle(ds.tetra_points(i))(j);
			}
			i += 1;
		}

		ds.stress = stress_handle.get(primoff);
		ds.B.postMult(ds.point_values, ds.strain_vector);
		ds.sigma.postMult(ds.strain_vector, ds.stress_vector);

		add_full_stress_vector(ds, use_viscosity);
		form_stress_tensor(ds);
		stress_handle.set(primoff, ds.stress);

		ds.point_values.zero();
		ds.BT.postMult(ds.stress_vector, ds.point_values);

		assemble_linear_element_matrix(ds, use_viscosity);
		if(wil.theta != 1)
			assemble_non_linear_element_matrix(ds);

		unsigned int q = 0;
		unsigned int i_1 = 0;
		for (GA_Iterator pointit(prim->getPointRange()); !pointit.atEnd(); ++pointit)
		{
			GA_Offset ptoff = *pointit;
			UT_Vector3D force_vector(0.0);
			UT_Vector3D surface_force_vector(0.0);
			fpreal s_area = 0;
			if (sf.isValid())
			{
				surface_force_vector = ds.bound(i_1)*(sf(ds.tetra_points_offset(i_1)) + sf(ds.tetra_points_offset(0)) + sf(ds.tetra_points_offset(1)) + sf(ds.tetra_points_offset(2)) + sf(ds.tetra_points_offset(3)));
			}

			if (force_handle.isValid())
			{
				if(ds.bound(0) + ds.bound(1) + ds.bound(2) + ds.bound(3) > 2.5)
					force_vector = force_handle(ds.tetra_points_offset(i_1)) + force_handle(ds.tetra_points_offset(0)) + force_handle(ds.tetra_points_offset(1)) + force_handle(ds.tetra_points_offset(2)) + force_handle(ds.tetra_points_offset(3));
			}
			UT_Vector3 external_force(0.0);
			Collider::apply_external_forces(object, pp_handle(ds.tetra_points_offset(i_1)), vp_handle(ds.tetra_points_offset(i_1)), external_force);

			ds.lumped_mass(ds.global_bound_index(q)) += a*(5 * parms.density);
			
			for (unsigned int j = 0; j < 3; j++)
			{
				ds.global_force(ds.global_index(q)) += c*(surface_force_vector(j));
				ds.global_force(ds.global_index(q)) += a*(force_vector(j) + external_force(j));
				ds.global_force(ds.global_index(q)) -= b*ds.point_values(3 * i_1 + j);

				for (unsigned int w = 0; w < 12; w++)
				{
					inject_element_matrix(gdp, w, q, ds, wil);
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
	GA_ROHandleV3D surf_force_handle(gdp->findAttribute(GA_ATTRIB_POINT, "surface_force"));
	GA_ROHandleV3D lp_handle(gdp->findAttribute(GA_ATTRIB_POINT, "P0"));
	GA_ROHandleV3D pp_handle(gdp->findAttribute(GA_ATTRIB_POINT, "P"));
	GA_ROHandleV3D vp_handle(gdp->findAttribute(GA_ATTRIB_POINT, "v"));
	if (stress_handle.isValid() && vp_handle.isValid() && pp_handle.isValid() && lp_handle.isValid())
	{
		unsigned int point_range = gdp->getPointRange().getEntries();
		
		double prim_surf_area = 0;
		double a = ds.determinant / 120.0;
		double b = ds.determinant / 24.0;
		GA_Primitive *prim = gdp->getPrimitive(primoff);
		unsigned int i = 0;
		UT_Vector3i boundary_points;
		unsigned int boundary_check = 0;

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
				ds.point_values(3 * i + j) = pp_handle(ds.tetra_points_offset(i))(j) - lp_handle(ds.tetra_points_offset(i))(j);
			}
			i += 1;
		}
		fpreal c = 0;
		if (boundary_check == 3 && surf_force_handle.isValid())
		{
			const UT_Vector3D p0 = pp_handle(boundary_points(0));
			const UT_Vector3D p1 = pp_handle(boundary_points(1));
			const UT_Vector3D p2 = pp_handle(boundary_points(2));

			c = 1.0/6.0*cross((p1 - p0), (p2 - p0)).length();
			std::cout << boundary_points << std::endl;
		}

		ds.stress = stress_handle.get(primoff);
		ds.B.postMult(ds.point_values, ds.strain_vector);
		ds.sigma.postMult(ds.strain_vector, ds.stress_vector);
		add_full_stress_vector(ds, use_viscosity);
		form_stress_tensor(ds);
		stress_handle.set(primoff, ds.stress);
		
		ds.point_values.zero();
		ds.BT.postMult(ds.stress_vector, ds.point_values);

		unsigned int i_1 = 0;		
		for (GA_Iterator pointit(prim->getPointRange()); !pointit.atEnd(); ++pointit)
		{
			GA_Offset ptoff = *pointit;
			GA_Index ptindex = pointit.getIndex();
			
			ds.lumped_mass(ptindex) += a*(5 * parms.density);

			UT_Vector3D force_vector(0.0);
			UT_Vector3D surface_force(0.0);

			if (force_handle.isValid())
			{
				force_vector = force_handle(ds.tetra_points_offset(i_1)) + force_handle(ds.tetra_points_offset(0)) + force_handle(ds.tetra_points_offset(1)) + force_handle(ds.tetra_points_offset(2)) + force_handle(ds.tetra_points_offset(3));
			}

			UT_Vector3D point_surf_force(0.0);
			//GA_Index ptsurfindex = gdp->getPointMap().offsetFromIndex(GA_Index(boundary_points(i_1)));
			if (boundary_check == 3 && surf_force_handle.isValid() && i_1 < 3)
			{
				point_surf_force = surf_force_handle(boundary_points(i_1)) + surf_force_handle(boundary_points(0)) + surf_force_handle(boundary_points(1)) + surf_force_handle(boundary_points(2));
			}

			UT_Vector3 external_force(0.0);
			Collider::apply_external_forces(object, pp_handle(ds.tetra_points_offset(i_1)), vp_handle(ds.tetra_points_offset(i_1)), external_force);

			for (unsigned int j = 0; j < 3; j++) 
			{
				ds.global_force(ptindex + j*(point_range)) += a*(force_vector(j) + external_force(j));
				ds.global_force(ptindex + j*(point_range)) -= b*ds.point_values(3 * i_1 + j);
				//ds.global_force(ptsurfindex + j*point_range) += c*point_surf_force(j);
			}
			i_1 += 1;	
		}
	}
}

float gridUpdate::point_to_prim(const GA_Primitive *prim, GA_ROHandleD &attrib_handle, const Data_struct &ds)
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

void gridUpdate::add_full_stress_vector(Data_struct &ds, const bool &use_vic)
{

	ds.stress_vector(0) += ds.stress(0, 0);
	ds.stress_vector(1) += ds.stress(1, 1);
	ds.stress_vector(2) += ds.stress(2, 2);

	ds.stress_vector(3) += ds.stress(1, 2);
	ds.stress_vector(4) += ds.stress(2, 0);
	ds.stress_vector(5) += ds.stress(0, 1);

	if (use_vic)
	{	
		ds.M1.postMult(ds.stress_vector, ds.m1_holder);
		ds.stress_vector = ds.m1_holder;
	}
}

void gridUpdate::form_stress_tensor(Data_struct &ds)
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

void gridUpdate::update_jacobians(const GU_Detail *gdp, const GA_Primitive *prim, Data_struct &ds)
{	
	GA_ROHandleV3D pp(gdp->findAttribute(GA_ATTRIB_POINT, "P"));
	if (pp.isValid())
	{	
		UT_Vector3D p0 = pp.get(ds.tetra_points_offset(0));
		UT_Vector3D p1 = pp.get(ds.tetra_points_offset(1));
		UT_Vector3D p2 = pp.get(ds.tetra_points_offset(2));
		UT_Vector3D p3 = pp.get(ds.tetra_points_offset(3));
		
		double v1 = (p1(0)*(p2(1)*p3(2) - p3(1)*p2(2)) + p2(0)*(p3(1)*p1(2) - p1(1)*p3(2)) + p3(0)*(p1(1)*p2(2) - p2(1)*p1(2)));
		double v2 = (p0(0)*(p3(1)*p2(2) - p2(1)*p3(2)) + p2(0)*(p0(1)*p3(2) - p3(1)*p0(2)) + p3(0)*(p2(1)*p0(2) - p0(1)*p2(2)));
		double v3 = (p0(0)*(p1(1)*p3(2) - p3(1)*p1(2)) + p1(0)*(p3(1)*p0(2) - p0(1)*p3(2)) + p3(0)*(p0(1)*p1(2) - p1(1)*p0(2)));
		double v4 = (p0(0)*(p2(1)*p1(2) - p1(1)*p2(2)) + p1(0)*(p0(1)*p2(2) - p2(1)*p0(2)) + p2(0)*(p1(1)*p0(2) - p0(1)*p1(2)));

		UT_Vector4D v(v1, v2, v3, v4);
		UT_Vector4D x(p0(0), p1(0), p2(0), p3(0));
		UT_Vector4D y(p0(1), p1(1), p2(1), p3(1));
		UT_Vector4D z(p0(2), p1(2), p2(2), p3(2));

		ds.inverse_jacobian(0, 0) = v(0);
		ds.inverse_jacobian(1, 0) = v(1);
		ds.inverse_jacobian(2, 0) = v(2);
		ds.inverse_jacobian(3, 0) = v(3);

		ds.inverse_jacobian(0, 1) = (y(3) - y(1))*(z(2) - z(1)) - (y(2) - y(1))*(z(3) - z(1));
		ds.inverse_jacobian(1, 1) = (y(2) - y(0))*(z(3) - z(2)) - (y(2) - y(3))*(z(0) - z(2));
		ds.inverse_jacobian(2, 1) = (y(1) - y(3))*(z(0) - z(3)) - (y(0) - y(3))*(z(1) - z(3));
		ds.inverse_jacobian(3, 1) = (y(0) - y(2))*(z(1) - z(0)) - (y(0) - y(1))*(z(2) - z(0));

		ds.inverse_jacobian(0, 2) = (x(2) - x(1))*(z(3) - z(1)) - (x(3) - x(1))*(z(2) - z(1));
		ds.inverse_jacobian(1, 2) = (x(3) - x(2))*(z(2) - z(0)) - (x(0) - x(2))*(z(2) - z(3));
		ds.inverse_jacobian(2, 2) = (x(0) - x(3))*(z(1) - z(3)) - (x(1) - x(3))*(z(0) - z(3));
		ds.inverse_jacobian(3, 2) = (x(1) - x(0))*(z(0) - z(2)) - (x(2) - x(0))*(z(0) - z(1));

		ds.inverse_jacobian(0, 3) = (x(3) - x(1))*(y(2) - y(1)) - (x(2) - x(1))*(y(3) - y(1));
		ds.inverse_jacobian(1, 3) = (x(2) - x(0))*(y(3) - y(2)) - (x(2) - x(3))*(y(0) - y(2));
		ds.inverse_jacobian(2, 3) = (x(1) - x(3))*(y(0) - y(3)) - (x(0) - x(3))*(y(1) - y(3));
		ds.inverse_jacobian(3, 3) = (x(0) - x(2))*(y(1) - y(0)) - (x(0) - x(1))*(y(2) - y(0));

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

		ds.determinant = ds.jacobian.determinant();
		ds.inverse_jacobian = 1.0 / ds.determinant*ds.inverse_jacobian;

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
/*void gridUpdate::update_jacobians_2D(const GU_Detail *gdp, const GA_Primitive *prim, Data_struct_2D &ds)
{
	GA_ROHandleV3D pp(gdp->findAttribute(GA_ATTRIB_POINT, "P"));
	if (pp.isValid())
	{
		UT_Vector3D p0 = pp.get(ds.tetra_points_offset(0));
		UT_Vector3D p1 = pp.get(ds.tetra_points_offset(1));
		UT_Vector3D p2 = pp.get(ds.tetra_points_offset(2));

		fpreal a10 = p1(0) - p0(0);
		fpreal a20 = p2(0) - p0(0);

		fpreal a11 = p1(1) - p0(1);
		fpreal a21 = p2(1) - p0(1);

		fpreal a12 = p1(2) - p0(2);
		fpreal a22 = p2(2) - p0(2);

		ds.jacobian(0, 0) = 1;
		ds.jacobian(1, 0) = a10;
		ds.jacobian(2, 0) = a20;

		ds.jacobian(0, 1) = 1;
		ds.jacobian(1, 1) = a11;
		ds.jacobian(2, 1) = a21;

		ds.jacobian(0, 2) = 1;
		ds.jacobian(1, 2) = a12;
		ds.jacobian(2, 2) = a22;		

		ds.jacobian *= 0.5;
		ds.determinant = ds.jacobian.determinant();

		fpreal a00_inv = a11*a22 - a12*a21;
		fpreal a01_inv = -a10*a22 + a12*a20;
		fpreal a02_inv = a10*a21 - a11*a21;

		fpreal a10_inv = a21 - a22;
		fpreal a11_inv = a22 - a20;
		fpreal a12_inv = a21 - a20;

		fpreal a20_inv = a11*a22 - a12*a21;
		fpreal a21_inv = a12 - a10;
		fpreal a22_inv = a11 - a10;

		ds.inverse_jacobian(0, 0) = a00_inv;
		ds.inverse_jacobian(1, 0) = a01_inv;
		ds.inverse_jacobian(2, 0) = a02_inv;

		ds.inverse_jacobian(0, 1) = a10_inv;
		ds.inverse_jacobian(1, 1) = a11_inv;
		ds.inverse_jacobian(2, 1) = a12_inv;

		ds.inverse_jacobian(0, 2) = a20_inv;
		ds.inverse_jacobian(1, 2) = a21_inv;
		ds.inverse_jacobian(2, 2) = a22_inv;

		ds.inverse_jacobian = 1 / ds.determinant*ds.inverse_jacobian;

		
	}
}
*/
void gridUpdate::assemble_m1(Data_struct &ds, const Material &parms)
{
	ds.M1.zero();

	float tau = 2 * parms.viscosity*(1 + parms.poisson_ratio) / parms.youngs_modulus;
	
	float aa = (1 - exp(-parms.dt / tau))  * (2.0 / 3.0);
	float ab = (1 - exp(-parms.dt / tau)) * (-1.0 / 3.0);
	float ac = (1 - exp(-parms.dt / tau)) * (1.0);

	for (unsigned int i = 0; i < 6; i++)
	{
		for (unsigned int j = 0; j < 6; j++)
		{
			if (i == j && i < 3)
				ds.M1(i, j) = 1.0 - aa;
			if (i == j && i >= 3)
				ds.M1(i, j) = 1.0 - ac;
			if (i != j && i < 3 && j < 3)
				ds.M1(i,j) = -ab;
		}
	}
};

void gridUpdate::assemble_m2(Data_struct &ds, Material parms)
{
	ds.M2.zero();

	float ah = parms.youngs_modulus / (3.0 * (1.0 - 2.0 * parms.poisson_ratio));
	float bh = parms.youngs_modulus / (3.0 * (1.0 - 2.0 * parms.poisson_ratio));

	double tau = 2.0 * parms.viscosity*(1 + parms.poisson_ratio) / parms.youngs_modulus;

	float af = 4.0 * parms.youngs_modulus / (6.0 * (1.0 + parms.poisson_ratio)) * (tau / parms.dt)*(1.0 - exp(-parms.dt / tau));
	float bf = -2.0 * parms.youngs_modulus / (6.0 * (1.0 + parms.poisson_ratio)) * (tau / parms.dt)*(1.0 - exp(-parms.dt / tau));
	float cf = 3.0 * parms.youngs_modulus / (6.0 * (1.0 + parms.poisson_ratio)) * (tau / parms.dt)*(1.0 - exp(-parms.dt / tau));
	
	for(unsigned int i = 0; i < 6; i++)
	{
		for(unsigned int j = 0; j < 6; j++)
		{
			if (i == j && i < 3)
				ds.M2(i, j) = af + ah;
			if (i == j && i >= 3)
				ds.M2(i, j) = cf + bh;
			if (i != j && i < 3 && j < 3)
				ds.M2(i, j) = bf + bh;
		}
	}
};

void gridUpdate::assemble_E(Data_struct &ds, Material parms)
{
	ds.sigma.zero();

	float diagonal1 = 2 * (1 - parms.poisson_ratio) / (1 - 2 * parms.poisson_ratio);
	float diagonal2 = 1.0;
	float diagonal3 = 2 * parms.poisson_ratio / (1 - 2 * parms.poisson_ratio);
	float multiplier = parms.youngs_modulus / (2 * (1 + parms.poisson_ratio));

	float diag1 = diagonal1 * multiplier;
	float diag2 = diagonal2 * multiplier;
	float diag3 = diagonal3 * multiplier;

	for (unsigned int i = 0; i < 6; i++)
	{
		for (unsigned int j = 0; j < 6; j++)
		{
			if (i == j && i < 3)
				ds.sigma(i, j) = diag1;
			if (i == j && i >= 3)
				ds.sigma(i, j) = diag2;
			if (i != j && i < 3 && j < 3)
				ds.sigma(i, j) = diag3;
		}
	}
};

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

void gridUpdate::assemble_non_linear_element_matrix(Data_struct &ds)
{
	for(unsigned int q = 0; q < 9; q+=3)
	{ 
		for (unsigned int i = 0; i < 3; i++)
		{
			for (unsigned int j = 0; j < 3; j++)
			{
					ds.stress_strain_matrix(q + i, q + j) = ds.stress(i, j);
			}
		}
	}

	ds.stress_strain_matrix.postMult(ds.BNL, ds.BNL_temp);
	ds.BNLT.postMult(ds.BNL_temp, ds.element_stiffness_matrix_nl);
}

void gridUpdate::assemble_linear_element_matrix(Data_struct &ds, bool use_vic)
{
	if (!use_vic)
		ds.sigma.postMult(ds.B, ds.B_temp);
	else
		ds.M2.postMult(ds.B, ds.B_temp);

	ds.element_stiffness_matrix.zero();
	ds.BT.postMult(ds.B_temp, ds.element_stiffness_matrix);
}

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
		UT_Vector3D acc = accp_handle(index);
		UT_Vector3D v = vp_handle(index);
		
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
			if (disp(index)(0) != 1)
			{
				ds.global_stiffness.addToElement(index, index, mass_contrib);
				ds.global_force(index) += inertia(0);
			}
			if (disp(index)(1) != 1)
			{
				ds.global_stiffness.addToElement(index + pointCount, index + pointCount, mass_contrib);
				ds.global_force(index + pointCount) += inertia(1);
			}
			if (disp(index)(2) != 1)
			{
				ds.global_stiffness.addToElement(index + 2 * pointCount, index + 2 * pointCount, mass_contrib);
				ds.global_force(index + 2 * pointCount) += inertia(2);
			}
		}
		
		if (disp.isValid())
		{
			const UT_Vector3 bc = disp(offset);
			if (bc(0) == 1)
			{
				ds.global_stiffness.addToElement(index, index, 1);
				ds.global_force(index) = 0;
			}

			if (bc(1) == 1)
			{
				ds.global_stiffness.addToElement(index + pointCount, index + pointCount, 1);
				ds.global_force(index + pointCount) = 0;
			}

			if (bc(2) == 1)
			{
				ds.global_stiffness.addToElement(index + 2 * pointCount, index + 2 * pointCount, 1);
				ds.global_force(index + 2 * pointCount) = 0;
			}
		}
	}
}

void gridUpdate::step_solution_exp(GU_Detail *gdp, const Data_struct &ds, Material &parms, const Wilson &wil)
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

void gridUpdate::step_solution_imp(GU_Detail *gdp, Data_struct &ds, const Wilson &wil, const Material &material)
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

void gridUpdate::update_material_parms(const GU_Detail *gdp, const GA_Primitive *prim, Data_struct &ds, Material &parms, bool &attribute_present, bool &use_vic)
{

	GA_ROHandleD densityp(gdp->findAttribute(GA_ATTRIB_POINT, "density"));
	GA_ROHandleD viscosityp(gdp->findAttribute(GA_ATTRIB_POINT, "viscosity"));
	GA_ROHandleD poissonp(gdp->findAttribute(GA_ATTRIB_POINT, "poisson_ratio"));
	GA_ROHandleD youngp(gdp->findAttribute(GA_ATTRIB_POINT, "Y"));

	if (viscosityp.isValid())
		parms.viscosity = gridUpdate::point_to_prim(prim, viscosityp, ds), attribute_present = true, use_vic = true;

	if (poissonp.isValid())
		parms.poisson_ratio = gridUpdate::point_to_prim(prim, poissonp, ds), attribute_present = true;

	if (youngp.isValid())
		parms.youngs_modulus = abs(gridUpdate::point_to_prim(prim, youngp, ds)), attribute_present = true;

	if (densityp.isValid())
		parms.density = abs(gridUpdate::point_to_prim(prim, densityp, ds)), attribute_present = true;

	if (parms.viscosity < 0)
		parms.viscosity = 0;

	if (parms.poisson_ratio >= 0.5)
		parms.poisson_ratio = 0.4999999;

	if (parms.poisson_ratio <= 0)
		parms.poisson_ratio = 0.0000001;
}


void gridUpdate::assemble_m1_2D(Data_struct_2D &ds, const Material &parms)
{
	ds.M1.zero();

	float tau = 2 * parms.viscosity*(1 + parms.poisson_ratio) / parms.youngs_modulus;

	float aa = (1 - exp(-parms.dt / tau))  * (2.0 / 3.0);
	float ab = (1 - exp(-parms.dt / tau)) * (-1.0 / 3.0);
	float ac = (1 - exp(-parms.dt / tau)) * (1.0);

	for (unsigned int i = 0; i < 4; i++)
	{
		for (unsigned int j = 0; j < 4; j++)
		{
			if (i == j && i < 2)
				ds.M1(i, j) = 1.0 - aa;
			if (i == j && i >= 2)
				ds.M1(i, j) = 1.0 - ac;
			if (i != j && i < 2 && j < 2)
				ds.M1(i, j) = -ab;
		}
	}
};

void gridUpdate::assemble_m2_2D(Data_struct_2D &ds, Material parms)
{
	ds.M2.zero();

	float ah = parms.youngs_modulus / (3.0 * (1.0 - 2.0 * parms.poisson_ratio));
	float bh = parms.youngs_modulus / (3.0 * (1.0 - 2.0 * parms.poisson_ratio));

	double tau = 2.0 * parms.viscosity*(1 + parms.poisson_ratio) / parms.youngs_modulus;

	float af = 4.0 * parms.youngs_modulus / (6.0 * (1.0 + parms.poisson_ratio)) * (tau / parms.dt)*(1.0 - exp(-parms.dt / tau));
	float bf = -2.0 * parms.youngs_modulus / (6.0 * (1.0 + parms.poisson_ratio)) * (tau / parms.dt)*(1.0 - exp(-parms.dt / tau));
	float cf = 3.0 * parms.youngs_modulus / (6.0 * (1.0 + parms.poisson_ratio)) * (tau / parms.dt)*(1.0 - exp(-parms.dt / tau));

	for (unsigned int i = 0; i < 4; i++)
	{
		for (unsigned int j = 0; j < 4; j++)
		{
			if (i == j && i < 2)
				ds.M2(i, j) = af + ah;
			if (i == j && i >= 2)
				ds.M2(i, j) = cf + bh;
			if (i != j && i < 2 && j < 2)
				ds.M2(i, j) = bf + bh;
		}
	}
};

void gridUpdate::assemble_E_2D(Data_struct_2D &ds, Material parms)
{
	ds.sigma.zero();

	float diagonal1 = 2 * (1 - parms.poisson_ratio) / (1 - 2 * parms.poisson_ratio);
	float diagonal2 = 1.0;
	float diagonal3 = 2 * parms.poisson_ratio / (1 - 2 * parms.poisson_ratio);
	float multiplier = parms.youngs_modulus / (2 * (1 + parms.poisson_ratio));

	float diag1 = diagonal1 * multiplier;
	float diag2 = diagonal2 * multiplier;
	float diag3 = diagonal3 * multiplier;

	for (unsigned int i = 0; i < 4; i++)
	{
		for (unsigned int j = 0; j < 4; j++)
		{
			if (i == j && i < 2)
				ds.sigma(i, j) = diag1;
			if (i == j && i >= 2)
				ds.sigma(i, j) = diag2;
			if (i != j && i < 2 && j < 2)
				ds.sigma(i, j) = diag3;
		}
	}
};

