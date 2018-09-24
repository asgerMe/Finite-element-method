#include <UT/UT_Matrix.h>
#include <UT/UT_SparseMatrix.h>
#include <UT/UT_Vector.h>
#include <GU/GU_Detail.h>



#ifndef __PLACEHOLDER__
#define __PLACEHOLDER__

class Data_struct 
{
public:
	void init(unsigned int points);
	void clear_globals();

	double determinant;
	unsigned int i;

	UT_MatrixD local_stiffness;
	UT_Matrix4D jacobian;
	UT_Matrix4D inverse_jacobian;
	
	UT_MatrixD B;
	UT_MatrixD B_temp;
	UT_MatrixD BT;
	UT_MatrixD BNL;
	UT_MatrixD BNLT;
	UT_MatrixD BNL_temp;

	UT_MatrixD element_stiffness_matrix_nl;
	UT_MatrixD element_stiffness_matrix;
	UT_MatrixD non_linear_element_3D;

	UT_MatrixD stress_strain_matrix;

	UT_MatrixD sigma;

	UT_MatrixD M1;
	UT_MatrixD M2;

	UT_VectorD m1_holder;
	UT_VectorD m2_holder;

	UT_VectorD point_values;
	UT_Matrix3D stress;
	UT_VectorD stress_vector;
	UT_VectorD strain_vector;

	UT_Vector4i tetra_points;
	UT_Vector4i tetra_points_offset;

	UT_Vector4i bound;

	fpreal surf_area;
	UT_SparseMatrixD global_stiffness;

	UT_VectorD global_force;
	UT_VectorD global_index;
	UT_VectorD global_bound_index;

	UT_VectorD lumped_mass;
	UT_VectorD X;
	UT_VectorD collision_holder;
};
class Data_struct_2D
{
public:
	void init(unsigned int points);
	void clear_globals();

	double determinant;
	unsigned int i;

	UT_MatrixD local_stiffness;
	UT_Matrix3D jacobian;
	UT_Matrix3D inverse_jacobian;

	UT_Matrix projection;
	
	UT_MatrixD non_linear_element_3D;
	UT_MatrixD B;
	UT_MatrixD projectionT;
	UT_MatrixD BT;
	UT_MatrixD BNL;
	UT_MatrixD BNLT;

	UT_MatrixD element_stiffness_matrix_nl;
	UT_MatrixD element_stiffness_matrix;
	UT_MatrixD stress_strain_matrix;

	UT_Vector stress_vector;
	UT_MatrixD sigma;

	UT_MatrixD M1;
	UT_MatrixD M2;

	UT_VectorD m1_holder;
	UT_VectorD m2_holder;

	UT_Matrix3D stress;
	

	UT_Vector3i tetra_points;
	UT_Vector3i tetra_points_offset;

	UT_Vector4i bound;

	fpreal surf_area;
	UT_SparseMatrixD global_stiffness;

	UT_VectorD global_force;
	UT_VectorD global_index;
	UT_VectorD global_bound_index;

	UT_VectorD lumped_mass;
	UT_VectorD X;
	UT_VectorD collision_holder;
};

#endif

