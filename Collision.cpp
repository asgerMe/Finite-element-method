#include "Collision.h"
#include <GU/GU_SDF.h>
#include <SIM/SIM_Object.h>
#include <SIM/SIM_ObjectArray.h>
#include <SIM/SIM_SingleSolver.h>
#include <SIM/SIM_GeometryCopy.h>
#include <GU/GU_RayIntersect.h>
#include <SIM/SIM_FORCE.h>
#include <SIM/SIM_OptionsUser.h>
#include <SIM/SIM_Utils.h>

Collider::Collider()
{

}

void Collider::calculate_element_hessian(const GU_Detail *gdp, const GA_Primitive *prim, const Data_struct_2D &ds, unsigned int i, unsigned int j)
{
	GA_OffsetArray offset_array;
	const GA_Offset primoff = prim->getMapOffset();
	gdp->getEdgeAdjacentPolygons(offset_array, primoff);
	
	GA_Offset n_offset = offset_array(i);
	const GA_Primitive *n_prim = gdp->getPrimitive(n_offset);
	fpreal n_area = n_prim->calcArea();
	fpreal area = prim->calcArea();
	fpreal A_An = 3.0 / (n_area + area);
	GA_Iterator pointit(prim->getPointRange());

	GA_Offset ptoff1 = prim->getPointOffset(0);
	GA_Offset ptoff2 = prim->getPointOffset(1);
	GA_Offset ptoff3 = prim->getPointOffset(2);
		
}

bool Collider::is_surface(const GA_Detail *gdp, const GA_Offset &ptoff, const char* surface_group_name)
{
	const GA_PointGroup *found_group = gdp->findPointGroup(surface_group_name);
	if (found_group != nullptr)
	{
		bool at_surface = found_group->contains(ptoff);
		return at_surface;
	}

	return false;
}
void Collider::apply_external_forces(const SIM_Object& object, const double &mass, const UT_Vector3 &pos, const UT_Vector3 &v, UT_Vector3 &result)
{

	const SIM_Geometry *geometry = object.getGeometry();
	SIM_ConstDataArray sim_forces;
	object.filterConstSubData(
		sim_forces, 0,
		SIM_DataFilterByType("SIM_Force"),
		SIM_FORCES_DATANAME,
		SIM_DataFilterNone()
	);
	
	UT_Vector3 ang_vel;
	UT_Vector3 torque;

	for (int fi = 0; fi < sim_forces.entries(); ++fi)
	{
		UT_Vector3 result_force;
		const SIM_Force* force = SIM_DATA_CASTCONST(sim_forces(fi), SIM_Force);
		force->getForce(object, pos, v, ang_vel, mass, result_force, torque);
		result += result_force;
		
	}
}

/*
void Collider::collidePoints(SIM_Object &object, GU_Detail *gdp, Data_struct &ds)
{
	SIM_ColliderInfoArray colliderinfo;
	const SIM_Object *colliderobject;
	const SIM_Geometry *collidergeo;

	ds.global_force.zero();

	object.getColliderInfo(colliderinfo);
	
	int n = colliderinfo.entries() - 1;
	GU_RayIntersect *isect;
	GU_MinInfo mininfo;
	UT_Vector3 primitive_normal;
	UT_Vector4 result;
	UT_Vector3 target_pos;
	UT_Vector3 point_pos;

	if (n > 0)
	{
		for (int i = 0; i < n; i++)
		{
			colliderobject = colliderinfo(i).getAffector();
			collidergeo = colliderobject->getGeometry();
			
			SIM_GeometryAutoReadLock  lock(collidergeo);
			const GU_Detail *collisiongdp = lock.getGdp();
			GA_Offset ptoff;

			isect = new GU_RayIntersect(collisiongdp);
			unsigned int n_points = gdp->getNumPoints();
			GA_FOR_ALL_PTOFF(gdp, ptoff)
			{
				point_pos = gdp->getPos3(ptoff);
				if (isect->isInsideWinding(point_pos,0))
				{

					mininfo.init(100,0);
					isect->minimumPoint(point_pos,mininfo);
					mininfo.prim->evaluateInteriorPoint(result, mininfo.u1, mininfo.v1);
					primitive_normal = mininfo.prim->computeNormal();
					target_pos(0) = result(0);
					target_pos(1) = result(1);
					target_pos(2) = result(2);

					gdp->setPos3(ptoff, target_pos);

					ds.collision_holder(ptoff) = 1;
					ds.collision_holder(ptoff + n_points) = 1;
					ds.collision_holder(ptoff + 2 * n_points) = 1;
				}
				
			}
			delete isect;
		}
	}	
}

*/







