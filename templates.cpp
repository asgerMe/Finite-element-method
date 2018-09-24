#include "GetFEMMatrices.h"
#include "Solve_System.h"

template void gridUpdate::assemble_E<Data_struct>(Data_struct &ds, const Material &parms);
template void gridUpdate::assemble_m1<Data_struct>(Data_struct &ds, const Material &parms);
template void gridUpdate::assemble_m2<Data_struct>(Data_struct &ds, const Material &parms);
template void gridUpdate::update_material_parms<Data_struct>(const GU_Detail *gdp, const GA_Primitive *prim, Data_struct &ds, Material &parms, bool &attribute_present, bool &use_vic);
template void gridUpdate::add_full_stress_vector<Data_struct>(Data_struct &ds, const bool &use_vic);
template void gridUpdate::form_stress_tensor<Data_struct>(Data_struct &ds);
template void gridUpdate::assemble_linear_element_matrix<Data_struct>(Data_struct &ds, bool use_vic);
template void gridUpdate::assemble_non_linear_element_matrix<Data_struct>(Data_struct &ds);


template void gridUpdate::assemble_E<Data_struct_2D>(Data_struct_2D &ds, const Material &parms);
template void gridUpdate::assemble_m1<Data_struct_2D>(Data_struct_2D &ds, const Material &parms);
template void gridUpdate::assemble_m2<Data_struct_2D>(Data_struct_2D &ds, const Material &parms);
template void gridUpdate::update_material_parms<Data_struct_2D>(const GU_Detail *gdp, const GA_Primitive *prim, Data_struct_2D &ds, Material &parms, bool &attribute_present, bool &use_vic);
template void gridUpdate::add_full_stress_vector<Data_struct_2D>(Data_struct_2D &ds, const bool &use_vic);
template void gridUpdate::form_stress_tensor<Data_struct_2D>(Data_struct_2D &ds);
template void gridUpdate::assemble_linear_element_matrix<Data_struct_2D>(Data_struct_2D &ds, bool use_vic);
template void gridUpdate::assemble_non_linear_element_matrix<Data_struct_2D>(Data_struct_2D &ds);

template void gridUpdate::step_solution_imp<Data_struct>(GU_Detail *gdp, Data_struct &ds, const Wilson &wil, const Material &material);
template void gridUpdate::step_solution_exp<Data_struct>(GU_Detail *gdp, Data_struct &ds, Material &parms, const Wilson &wil);

template void gridUpdate::step_solution_imp<Data_struct_2D>(GU_Detail *gdp, Data_struct_2D &ds, const Wilson &wil, const Material &material);
template void gridUpdate::step_solution_exp<Data_struct_2D>(GU_Detail *gdp, Data_struct_2D &ds, Material &parms, const Wilson &wil);

