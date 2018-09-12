#include "ConstraintMatrix.h"
#include "GETFEMMatrices.h"
#include <GU/GU_Detail.h>
#include <iostream>
#include <SOP/SOP_Node.h>
#include <UT/UT_SparseMatrix.h>
#include "Placeholder.h"
#include <vector>
#include <UT/UT_Vector.h>
#include <UT/UT_Matrix.h>
#include <math.h>

void Global_Methods::apply_volume_force(Data_struct &ds, const UT_Matrix3D &jacobian, const UT_Vector4i &tetra_points, const GU_Detail *gdp, GA_Offset ptoff)
{
	unsigned int point_count = gdp->getNumPoints();
	GA_ROHandleV3 force_p(gdp->findAttribute(GA_ATTRIB_PRIMITIVE, "force"));
	
	if (force_p.isValid())
	{
		double det_sc = jacobian.determinant()/24.0;
		for (unsigned int i = 0; i < 3; i++)
		{
			for (unsigned int j = 0; j < 4; j++)
			{
				if (force_p.isValid())
				{
					ds.global_force(tetra_points(j) + i*point_count) += det_sc * force_p(ptoff)(i);
				}
			}
		}
	}
};


void Global_Methods::apply_boundary_conditions(Data_struct &ds, const GU_Detail *gdp)
{
	GA_ROHandleV3 disp(gdp->findAttribute(GA_ATTRIB_POINT, "pintoanimation"));
	unsigned int pointCount = gdp->getNumPoints();

	if (disp.isValid())
	{
		for (GA_Iterator point_iter(gdp->getPointRange()); !point_iter.atEnd(); ++point_iter)
		{
			const GA_Offset offset = *point_iter;
			const UT_Vector3 bc = disp(offset);	
			ds.global_stiffness.addToElement(offset, offset, 1);
			ds.global_stiffness.addToElement(offset + pointCount, offset + pointCount, 1);
			ds.global_stiffness.addToElement(offset + 2 * pointCount, offset + 2 * pointCount, 1);
			if (bc(0) == 1)
			{
				ds.global_stiffness.addToElement(offset, offset, 1);
				ds.global_force(offset) = 0;
			}

			if (bc(1) == 1)
			{
				ds.global_stiffness.addToElement(offset + pointCount, offset + pointCount, 1);
				ds.global_force(offset + pointCount) = 0;
			}

			if (bc(2) == 1)
			{
				ds.global_stiffness.addToElement(offset + 2 * pointCount, offset + 2 * pointCount, 1);
				ds.global_force(offset + 2 * pointCount) = 0;
			}

		}
	}
}

void Global_Methods::assemble_global(Data_struct &ds, UT_Vector4i &tetra_points, const GU_Detail *gdp)
{
	GA_ROHandleV3 disp(gdp->findAttribute(GA_ATTRIB_POINT, "pintoanimation"));
	unsigned int pointCount = gdp->getNumPoints();

	UT_Vector global_index1(0, 11);
	UT_Vector global_index2(0, 11);
	unsigned int index = 0;
	
	for (unsigned int q = 0; q < 4; q++)
	{
		for (unsigned int w = 0; w < 3; w++)
		{
			global_index1(index) = tetra_points(q) + pointCount*w;
			global_index2(index) = tetra_points(q);
			index += 1;
		}
	}

	for (unsigned int i = 0; i < 12; i++)
	{
		for (unsigned int j = 0; j < 12; j++)
		{
			unsigned int x = global_index1(i);
			unsigned int y = global_index1(j);
			float value = ds.local_stiffness(i, j);
			if (disp.isValid())
			{
				unsigned int offset = global_index2(i);
				unsigned int offset2 = global_index2(j);
				UT_Vector3 boundsx = disp.get(offset);
				UT_Vector3 boundsy = disp.get(offset2);
			
				if (boundsx(0))
				{
					if (x < pointCount)
					{
						if (x != y)
						{
							ds.global_force(x) -= value;
						}
						value = 0;
					}
				}
				if (boundsy(0) == 1 && y < pointCount)
				{
					value = 0;
				}
				if (boundsx(1) == 1)
				{
					if (pointCount <= x && x < 2 * pointCount)
					{
						if (x != y)
						{
							ds.global_force(x) -= value;
						}
						value = 0;
					}
				}
				if (boundsy(1) == 1 && pointCount <= y && y < 2 * pointCount)
				{
					value = 0;
				}
				if (boundsx(2) == 1)
				{
					if (x >= 2 * pointCount)
					{
						if (x != y)
						{
							ds.global_force(x) -= value;
						}

						value = 0;
					}
				}
				if (boundsy(2) == 1 && y >= 2 * pointCount)
				{
					value = 0;
				}
			}
			ds.global_stiffness.addToElement(x, y, value);
		}
	}
};



