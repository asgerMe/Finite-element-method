#include <iostream>
#include <UT/UT_SparseMatrix.h>
#include <UT/UT_MatrixSolver.h>

#include <math.h>
#include <GU/GU_Detail.h>
#include "Placeholder.h"
#include "GETFEMMatrices.h"
#include <UT/UT_Vector.h>


#ifndef CONSTRAINTMATRIX
#define CONTRAINTMATRIX

class Global_Methods
{

public:

	void assemble_global(Data_struct &ds,  UT_Vector4i &tetra_points, const GU_Detail *gdp);
	void apply_boundary_conditions(Data_struct &ds, const GU_Detail *gdp);
	void apply_volume_force(Data_struct &ds, const UT_Matrix3D &jacobian, const UT_Vector4i &tetra_points, const GU_Detail *gdp, GA_Offset ptoff);
};

#endif
