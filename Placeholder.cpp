#include "Placeholder.h"
#include <GU/GU_Detail.h>
#include <UT/UT_SparseMatrix.h>
#include <iostream>


void Data_struct::clear_globals()
{
	element_stiffness_matrix.zero();
	element_stiffness_matrix_nl.zero();

	global_force.zero();
	global_stiffness.zero();
	jacobian.zero();
	inverse_jacobian.zero();
	lumped_mass.zero();
}
void Data_struct_2D::clear_globals()
{
	element_stiffness_matrix.zero();
	element_stiffness_matrix_nl.zero();

	global_force.zero();
	global_stiffness.zero();
	jacobian.zero();
	inverse_jacobian.zero();
	lumped_mass.zero();
}
void Data_struct::init(unsigned int points)
{
	i = 0;
	determinant = 0;
	surf_area = 0;

	B.init(0, 5, 0, 11);
	B.zero();

	B_temp.init(0, 5, 0, 11);
	B_temp.zero();

	global_index.init(0, 11);
	global_index.zero();

	global_bound_index.init(0, 11);
	global_bound_index.zero();

	BNL.init(0, 8, 0, 11);
	BNL.zero();

	BNL_temp.init(0, 8, 0, 11);
	BNL_temp.zero();

	BT.init(0, 11, 0, 5);
	BT.zero();

	BNLT.init(0, 11, 0, 8);
	BNLT.zero();

	element_stiffness_matrix_nl.init(0, 11, 0, 11);
	element_stiffness_matrix_nl.zero();

	element_stiffness_matrix.init(0, 11, 0, 11);
	element_stiffness_matrix.zero();

	sigma.init(0, 5, 0, 5);
	sigma.zero();

	M1.init(0, 5, 0, 5);
	M1.zero();

	M2.init(0, 5, 0, 5);
	M2.zero();

	m1_holder.init(0, 5);
	m1_holder.zero();

	m2_holder.init(0, 5);
	m2_holder.zero();

	jacobian(0.0f);
	inverse_jacobian(0.0f);

	global_force.init(0, 3 * (points)-1);
	global_force.zero();

	X.init(0, 3 * (points)-1);
	X.zero();

	global_stiffness.init(3 * (points), 3 * (points));
	global_stiffness.zero();

	lumped_mass.init(0, points - 1);
	lumped_mass.zero();

	point_values.init(0, 11);
	point_values.zero();

	stress_vector.init(0, 5);
	stress_vector.zero();

	strain_vector.init(0, 5);
	strain_vector.zero();

	stress_strain_matrix.init(0, 8, 0, 8);
	stress_strain_matrix.zero();

	tetra_points(0) = -1;
	tetra_points(1) = -1;
	tetra_points(2) = -1;
	tetra_points(3) = -1;

	tetra_points_offset(0) = -1;
	tetra_points_offset(1) = -1;
	tetra_points_offset(2) = -1;
	tetra_points_offset(3) = -1;

	bound(0) = 0;
	bound(1) = 0;
	bound(2) = 0;
	bound(3) = 0;

}
void Data_struct_2D::init(unsigned int points)
{
	B.init(0, 4, 0, 8);
	B.zero();

	B_temp.init(0, 4, 0, 8);
	B_temp.zero();

	global_index.init(0, 8);
	global_index.zero();

	global_bound_index.init(0, 8);
	global_bound_index.zero();

	BNL.init(0, 6, 0, 8);
	BNL.zero();

	BNL_temp.init(0, 6, 0, 8);
	BNL_temp.zero();

	BT.init(0, 8, 0, 4);
	BT.zero();

	BNLT.init(0, 8, 0, 6);
	BNLT.zero();

	element_stiffness_matrix_nl.init(0, 8, 0, 8);
	element_stiffness_matrix_nl.zero();

	element_stiffness_matrix.init(0, 8, 0, 8);
	element_stiffness_matrix.zero();

	sigma.init(0, 3, 0, 3);
	sigma.zero();

	M1.init(0, 3, 0, 3);
	M1.zero();

	M2.init(0, 3, 0, 3);
	M2.zero();

	m1_holder.init(0, 3);
	m1_holder.zero();

	m2_holder.init(0, 3);
	m2_holder.zero();

	jacobian(0.0f);
	inverse_jacobian(0.0f);

	global_force.init(0, 2 * (points)-1);
	global_force.zero();

	X.init(0, 3 * (points)-1);
	X.zero();

	global_stiffness.init(2 * (points), 2 * (points));
	global_stiffness.zero();

	lumped_mass.init(0, points - 1);
	lumped_mass.zero();

	point_values.init(0, 8);
	point_values.zero();

	stress_vector.init(0, 3);
	stress_vector.zero();

	strain_vector.init(0, 3);
	strain_vector.zero();

	stress_strain_matrix.init(0, 6, 0, 6);
	stress_strain_matrix.zero();

	tetra_points(0) = -1;
	tetra_points(1) = -1;
	tetra_points(2) = -1;
	tetra_points(3) = -1;

	tetra_points_offset(0) = -1;
	tetra_points_offset(1) = -1;
	tetra_points_offset(2) = -1;
	tetra_points_offset(3) = -1;

	bound(0) = 0;
	bound(1) = 0;
	bound(2) = 0;
	bound(3) = 0;
};


