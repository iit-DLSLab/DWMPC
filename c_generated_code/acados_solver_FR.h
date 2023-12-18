/*
 * Copyright (c) The acados authors.
 *
 * This file is part of acados.
 *
 * The 2-Clause BSD License
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 * this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 * this list of conditions and the following disclaimer in the documentation
 * and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.;
 */

#ifndef ACADOS_SOLVER_FR_H_
#define ACADOS_SOLVER_FR_H_

#include "acados/utils/types.h"

#include "acados_c/ocp_nlp_interface.h"
#include "acados_c/external_function_interface.h"

#define FR_NX     22
#define FR_NZ     0
#define FR_NU     15
#define FR_NP     53
#define FR_NBX    0
#define FR_NBX0   22
#define FR_NBU    0
#define FR_NSBX   0
#define FR_NSBU   0
#define FR_NSH    1
#define FR_NSH0   0
#define FR_NSG    0
#define FR_NSPHI  0
#define FR_NSHN   0
#define FR_NSGN   0
#define FR_NSPHIN 0
#define FR_NSPHI0 0
#define FR_NSBXN  0
#define FR_NS     1
#define FR_NS0    0
#define FR_NSN    0
#define FR_NG     0
#define FR_NBXN   0
#define FR_NGN    0
#define FR_NY0    36
#define FR_NY     36
#define FR_NYN    21
#define FR_N      50
#define FR_NH     23
#define FR_NHN    0
#define FR_NH0    0
#define FR_NPHI0  0
#define FR_NPHI   0
#define FR_NPHIN  0
#define FR_NR     0

#ifdef __cplusplus
extern "C" {
#endif


// ** capsule for solver data **
typedef struct FR_solver_capsule
{
    // acados objects
    ocp_nlp_in *nlp_in;
    ocp_nlp_out *nlp_out;
    ocp_nlp_out *sens_out;
    ocp_nlp_solver *nlp_solver;
    void *nlp_opts;
    ocp_nlp_plan_t *nlp_solver_plan;
    ocp_nlp_config *nlp_config;
    ocp_nlp_dims *nlp_dims;

    // number of expected runtime parameters
    unsigned int nlp_np;

    /* external functions */
    // dynamics

    external_function_param_casadi *discr_dyn_phi_fun;
    external_function_param_casadi *discr_dyn_phi_fun_jac_ut_xt;


    // cost

    external_function_param_casadi *cost_y_fun;
    external_function_param_casadi *cost_y_fun_jac_ut_xt;
    external_function_param_casadi *cost_y_hess;



    external_function_param_casadi cost_y_0_fun;
    external_function_param_casadi cost_y_0_fun_jac_ut_xt;
    external_function_param_casadi cost_y_0_hess;



    external_function_param_casadi cost_y_e_fun;
    external_function_param_casadi cost_y_e_fun_jac_ut_xt;
    external_function_param_casadi cost_y_e_hess;


    // constraints
    external_function_param_casadi *nl_constr_h_fun_jac;
    external_function_param_casadi *nl_constr_h_fun;







} FR_solver_capsule;

ACADOS_SYMBOL_EXPORT FR_solver_capsule * FR_acados_create_capsule(void);
ACADOS_SYMBOL_EXPORT int FR_acados_free_capsule(FR_solver_capsule *capsule);

ACADOS_SYMBOL_EXPORT int FR_acados_create(FR_solver_capsule * capsule);

ACADOS_SYMBOL_EXPORT int FR_acados_reset(FR_solver_capsule* capsule, int reset_qp_solver_mem);

/**
 * Generic version of FR_acados_create which allows to use a different number of shooting intervals than
 * the number used for code generation. If new_time_steps=NULL and n_time_steps matches the number used for code
 * generation, the time-steps from code generation is used.
 */
ACADOS_SYMBOL_EXPORT int FR_acados_create_with_discretization(FR_solver_capsule * capsule, int n_time_steps, double* new_time_steps);
/**
 * Update the time step vector. Number N must be identical to the currently set number of shooting nodes in the
 * nlp_solver_plan. Returns 0 if no error occurred and a otherwise a value other than 0.
 */
ACADOS_SYMBOL_EXPORT int FR_acados_update_time_steps(FR_solver_capsule * capsule, int N, double* new_time_steps);
/**
 * This function is used for updating an already initialized solver with a different number of qp_cond_N.
 */
ACADOS_SYMBOL_EXPORT int FR_acados_update_qp_solver_cond_N(FR_solver_capsule * capsule, int qp_solver_cond_N);
ACADOS_SYMBOL_EXPORT int FR_acados_update_params(FR_solver_capsule * capsule, int stage, double *value, int np);
ACADOS_SYMBOL_EXPORT int FR_acados_update_params_sparse(FR_solver_capsule * capsule, int stage, int *idx, double *p, int n_update);

ACADOS_SYMBOL_EXPORT int FR_acados_solve(FR_solver_capsule * capsule);
ACADOS_SYMBOL_EXPORT int FR_acados_free(FR_solver_capsule * capsule);
ACADOS_SYMBOL_EXPORT void FR_acados_print_stats(FR_solver_capsule * capsule);
ACADOS_SYMBOL_EXPORT int FR_acados_custom_update(FR_solver_capsule* capsule, double* data, int data_len);


ACADOS_SYMBOL_EXPORT ocp_nlp_in *FR_acados_get_nlp_in(FR_solver_capsule * capsule);
ACADOS_SYMBOL_EXPORT ocp_nlp_out *FR_acados_get_nlp_out(FR_solver_capsule * capsule);
ACADOS_SYMBOL_EXPORT ocp_nlp_out *FR_acados_get_sens_out(FR_solver_capsule * capsule);
ACADOS_SYMBOL_EXPORT ocp_nlp_solver *FR_acados_get_nlp_solver(FR_solver_capsule * capsule);
ACADOS_SYMBOL_EXPORT ocp_nlp_config *FR_acados_get_nlp_config(FR_solver_capsule * capsule);
ACADOS_SYMBOL_EXPORT void *FR_acados_get_nlp_opts(FR_solver_capsule * capsule);
ACADOS_SYMBOL_EXPORT ocp_nlp_dims *FR_acados_get_nlp_dims(FR_solver_capsule * capsule);
ACADOS_SYMBOL_EXPORT ocp_nlp_plan_t *FR_acados_get_nlp_plan(FR_solver_capsule * capsule);

#ifdef __cplusplus
} /* extern "C" */
#endif

#endif  // ACADOS_SOLVER_FR_H_
