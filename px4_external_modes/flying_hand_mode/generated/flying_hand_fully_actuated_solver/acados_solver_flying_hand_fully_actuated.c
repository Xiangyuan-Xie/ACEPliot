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

// standard
#include <stdio.h>
#include <stdlib.h>
#include <assert.h>
#include <string.h> // memcpy
// acados
// #include "acados/utils/print.h"
#include "acados_c/ocp_nlp_interface.h"
#include "acados_c/external_function_interface.h"

// example specific

#include "flying_hand_fully_actuated_model/flying_hand_fully_actuated_model.h"


#include "flying_hand_fully_actuated_constraints/flying_hand_fully_actuated_constraints.h"
#include "flying_hand_fully_actuated_cost/flying_hand_fully_actuated_cost.h"



#include "acados_solver_flying_hand_fully_actuated.h"

#define NX     FLYING_HAND_FULLY_ACTUATED_NX
#define NZ     FLYING_HAND_FULLY_ACTUATED_NZ
#define NU     FLYING_HAND_FULLY_ACTUATED_NU
#define NP     FLYING_HAND_FULLY_ACTUATED_NP
#define NP_GLOBAL     FLYING_HAND_FULLY_ACTUATED_NP_GLOBAL
#define NY0    FLYING_HAND_FULLY_ACTUATED_NY0
#define NY     FLYING_HAND_FULLY_ACTUATED_NY
#define NYN    FLYING_HAND_FULLY_ACTUATED_NYN

#define NBX    FLYING_HAND_FULLY_ACTUATED_NBX
#define NBX0   FLYING_HAND_FULLY_ACTUATED_NBX0
#define NBU    FLYING_HAND_FULLY_ACTUATED_NBU
#define NG     FLYING_HAND_FULLY_ACTUATED_NG
#define NBXN   FLYING_HAND_FULLY_ACTUATED_NBXN
#define NGN    FLYING_HAND_FULLY_ACTUATED_NGN

#define NH     FLYING_HAND_FULLY_ACTUATED_NH
#define NHN    FLYING_HAND_FULLY_ACTUATED_NHN
#define NH0    FLYING_HAND_FULLY_ACTUATED_NH0
#define NPHI   FLYING_HAND_FULLY_ACTUATED_NPHI
#define NPHIN  FLYING_HAND_FULLY_ACTUATED_NPHIN
#define NPHI0  FLYING_HAND_FULLY_ACTUATED_NPHI0
#define NR     FLYING_HAND_FULLY_ACTUATED_NR

#define NS     FLYING_HAND_FULLY_ACTUATED_NS
#define NS0    FLYING_HAND_FULLY_ACTUATED_NS0
#define NSN    FLYING_HAND_FULLY_ACTUATED_NSN

#define NSBX   FLYING_HAND_FULLY_ACTUATED_NSBX
#define NSBU   FLYING_HAND_FULLY_ACTUATED_NSBU
#define NSH0   FLYING_HAND_FULLY_ACTUATED_NSH0
#define NSH    FLYING_HAND_FULLY_ACTUATED_NSH
#define NSHN   FLYING_HAND_FULLY_ACTUATED_NSHN
#define NSG    FLYING_HAND_FULLY_ACTUATED_NSG
#define NSPHI0 FLYING_HAND_FULLY_ACTUATED_NSPHI0
#define NSPHI  FLYING_HAND_FULLY_ACTUATED_NSPHI
#define NSPHIN FLYING_HAND_FULLY_ACTUATED_NSPHIN
#define NSGN   FLYING_HAND_FULLY_ACTUATED_NSGN
#define NSBXN  FLYING_HAND_FULLY_ACTUATED_NSBXN
// initial value of stagewise parameters
static const double p_init[] = {0,0,0,1,0,0,0,5,0.1,0,0,0,0.1,0,0,0,0.15,0,0.05,0,0.076,0.363,0.441,0.007,0.2,0.1,-0.1,-1.578,0,0,0,0,0,0,0,0,1,0,0,0,0.66,0.68,0.81,0.85,0.0000000000000002937764938675783,0.8440296287459855,-0.8440296287459857,-0,0.8440296287459853,-0.8440296287459854,0.974601466721029,-0.48730073336051455,-0.4873007333605146,0.9746014667210292,-0.4873007333605146,-0.4873007333605144,-0.1773629620793188,-0.17736296207931865,-0.17736296207931868,-0.17736296207931868,-0.17736296207931868,-0.17736296207931868,0.00000000000000038256187823935575,-0.9233022122112732,-0.9233022122112734,-0.00000000000000002405084315045608,0.9233022122112732,0.9233022122112732,1.0661375615271111,0.5330687807635553,-0.5330687807635553,-1.0661375615271107,-0.5330687807635557,0.5330687807635553,1.233830334401194,-1.2338303344011936,1.2338303344011938,-1.2338303344011945,1.2338303344011945,-1.233830334401194,0,0,0,0,};





// ** solver data **

flying_hand_fully_actuated_solver_capsule * flying_hand_fully_actuated_acados_create_capsule(void)
{
    void* capsule_mem = malloc(sizeof(flying_hand_fully_actuated_solver_capsule));
    flying_hand_fully_actuated_solver_capsule *capsule = (flying_hand_fully_actuated_solver_capsule *) capsule_mem;

    return capsule;
}


int flying_hand_fully_actuated_acados_free_capsule(flying_hand_fully_actuated_solver_capsule *capsule)
{
    free(capsule);
    return 0;
}


int flying_hand_fully_actuated_acados_create(flying_hand_fully_actuated_solver_capsule* capsule)
{
    int N_shooting_intervals = FLYING_HAND_FULLY_ACTUATED_N;
    double* new_time_steps = NULL; // NULL -> don't alter the code generated time-steps
    return flying_hand_fully_actuated_acados_create_with_discretization(capsule, N_shooting_intervals, new_time_steps);
}


int flying_hand_fully_actuated_acados_update_time_steps(flying_hand_fully_actuated_solver_capsule* capsule, int N, double* new_time_steps)
{

    if (N != capsule->nlp_solver_plan->N) {
        fprintf(stderr, "flying_hand_fully_actuated_acados_update_time_steps: given number of time steps (= %d) " \
            "differs from the currently allocated number of " \
            "time steps (= %d)!\n" \
            "Please recreate with new discretization and provide a new vector of time_stamps!\n",
            N, capsule->nlp_solver_plan->N);
        return 1;
    }

    ocp_nlp_config * nlp_config = capsule->nlp_config;
    ocp_nlp_dims * nlp_dims = capsule->nlp_dims;
    ocp_nlp_in * nlp_in = capsule->nlp_in;

    for (int i = 0; i < N; i++)
    {
        ocp_nlp_in_set(nlp_config, nlp_dims, nlp_in, i, "Ts", &new_time_steps[i]);
        ocp_nlp_cost_model_set(nlp_config, nlp_dims, nlp_in, i, "scaling", &new_time_steps[i]);
    }
    return 0;

}

/**
 * Internal function for flying_hand_fully_actuated_acados_create: step 1
 */
void flying_hand_fully_actuated_acados_create_set_plan(ocp_nlp_plan_t* nlp_solver_plan, const int N)
{
    assert(N == nlp_solver_plan->N);

    /************************************************
    *  plan
    ************************************************/

    nlp_solver_plan->nlp_solver = SQP_RTI;

    nlp_solver_plan->ocp_qp_solver_plan.qp_solver = PARTIAL_CONDENSING_HPIPM;
    nlp_solver_plan->relaxed_ocp_qp_solver_plan.qp_solver = PARTIAL_CONDENSING_HPIPM;
    nlp_solver_plan->nlp_cost[0] = NONLINEAR_LS;
    for (int i = 1; i < N; i++)
        nlp_solver_plan->nlp_cost[i] = NONLINEAR_LS;

    nlp_solver_plan->nlp_cost[N] = NONLINEAR_LS;

    for (int i = 0; i < N; i++)
    {
        nlp_solver_plan->nlp_dynamics[i] = CONTINUOUS_MODEL;
        nlp_solver_plan->sim_solver_plan[i].sim_solver = ERK;
    }

    nlp_solver_plan->nlp_constraints[0] = BGH;

    for (int i = 1; i < N; i++)
    {
        nlp_solver_plan->nlp_constraints[i] = BGH;
    }
    nlp_solver_plan->nlp_constraints[N] = BGH;

    nlp_solver_plan->regularization = NO_REGULARIZE;

    nlp_solver_plan->globalization = FIXED_STEP;
}


static ocp_nlp_dims* flying_hand_fully_actuated_acados_create_setup_dimensions(flying_hand_fully_actuated_solver_capsule* capsule)
{
    ocp_nlp_plan_t* nlp_solver_plan = capsule->nlp_solver_plan;
    const int N = nlp_solver_plan->N;
    ocp_nlp_config* nlp_config = capsule->nlp_config;

    /************************************************
    *  dimensions
    ************************************************/
    #define NINTNP1MEMS 18
    int* intNp1mem = (int*)malloc( (N+1)*sizeof(int)*NINTNP1MEMS );

    int* nx    = intNp1mem + (N+1)*0;
    int* nu    = intNp1mem + (N+1)*1;
    int* nbx   = intNp1mem + (N+1)*2;
    int* nbu   = intNp1mem + (N+1)*3;
    int* nsbx  = intNp1mem + (N+1)*4;
    int* nsbu  = intNp1mem + (N+1)*5;
    int* nsg   = intNp1mem + (N+1)*6;
    int* nsh   = intNp1mem + (N+1)*7;
    int* nsphi = intNp1mem + (N+1)*8;
    int* ns    = intNp1mem + (N+1)*9;
    int* ng    = intNp1mem + (N+1)*10;
    int* nh    = intNp1mem + (N+1)*11;
    int* nphi  = intNp1mem + (N+1)*12;
    int* nz    = intNp1mem + (N+1)*13;
    int* ny    = intNp1mem + (N+1)*14;
    int* nr    = intNp1mem + (N+1)*15;
    int* nbxe  = intNp1mem + (N+1)*16;
    int* np  = intNp1mem + (N+1)*17;

    for (int i = 0; i < N+1; i++)
    {
        // common
        nx[i]     = NX;
        nu[i]     = NU;
        nz[i]     = NZ;
        ns[i]     = NS;
        // cost
        ny[i]     = NY;
        // constraints
        nbx[i]    = NBX;
        nbu[i]    = NBU;
        nsbx[i]   = NSBX;
        nsbu[i]   = NSBU;
        nsg[i]    = NSG;
        nsh[i]    = NSH;
        nsphi[i]  = NSPHI;
        ng[i]     = NG;
        nh[i]     = NH;
        nphi[i]   = NPHI;
        nr[i]     = NR;
        nbxe[i]   = 0;
        np[i]     = NP;
    }

    // for initial state
    nbx[0] = NBX0;
    nsbx[0] = 0;
    ns[0] = NS0;
    
    nbxe[0] = 17;
    
    ny[0] = NY0;
    nh[0] = NH0;
    nsh[0] = NSH0;
    nsphi[0] = NSPHI0;
    nphi[0] = NPHI0;


    // terminal - common
    nu[N]   = 0;
    nz[N]   = 0;
    ns[N]   = NSN;
    // cost
    ny[N]   = NYN;
    // constraint
    nbx[N]   = NBXN;
    nbu[N]   = 0;
    ng[N]    = NGN;
    nh[N]    = NHN;
    nphi[N]  = NPHIN;
    nr[N]    = 0;

    nsbx[N]  = NSBXN;
    nsbu[N]  = 0;
    nsg[N]   = NSGN;
    nsh[N]   = NSHN;
    nsphi[N] = NSPHIN;

    /* create and set ocp_nlp_dims */
    ocp_nlp_dims * nlp_dims = ocp_nlp_dims_create(nlp_config);

    ocp_nlp_dims_set_opt_vars(nlp_config, nlp_dims, "nx", nx);
    ocp_nlp_dims_set_opt_vars(nlp_config, nlp_dims, "nu", nu);
    ocp_nlp_dims_set_opt_vars(nlp_config, nlp_dims, "nz", nz);
    ocp_nlp_dims_set_opt_vars(nlp_config, nlp_dims, "ns", ns);
    ocp_nlp_dims_set_opt_vars(nlp_config, nlp_dims, "np", np);

    ocp_nlp_dims_set_global(nlp_config, nlp_dims, "np_global", 0);
    ocp_nlp_dims_set_global(nlp_config, nlp_dims, "n_global_data", 0);

    for (int i = 0; i <= N; i++)
    {
        ocp_nlp_dims_set_constraints(nlp_config, nlp_dims, i, "nbx", &nbx[i]);
        ocp_nlp_dims_set_constraints(nlp_config, nlp_dims, i, "nbu", &nbu[i]);
        ocp_nlp_dims_set_constraints(nlp_config, nlp_dims, i, "nsbx", &nsbx[i]);
        ocp_nlp_dims_set_constraints(nlp_config, nlp_dims, i, "nsbu", &nsbu[i]);
        ocp_nlp_dims_set_constraints(nlp_config, nlp_dims, i, "ng", &ng[i]);
        ocp_nlp_dims_set_constraints(nlp_config, nlp_dims, i, "nsg", &nsg[i]);
        ocp_nlp_dims_set_constraints(nlp_config, nlp_dims, i, "nbxe", &nbxe[i]);
    }
    ocp_nlp_dims_set_cost(nlp_config, nlp_dims, 0, "ny", &ny[0]);
    for (int i = 1; i < N; i++)
        ocp_nlp_dims_set_cost(nlp_config, nlp_dims, i, "ny", &ny[i]);
    ocp_nlp_dims_set_constraints(nlp_config, nlp_dims, 0, "nh", &nh[0]);
    ocp_nlp_dims_set_constraints(nlp_config, nlp_dims, 0, "nsh", &nsh[0]);

    for (int i = 1; i < N; i++)
    {
        ocp_nlp_dims_set_constraints(nlp_config, nlp_dims, i, "nh", &nh[i]);
        ocp_nlp_dims_set_constraints(nlp_config, nlp_dims, i, "nsh", &nsh[i]);
    }
    ocp_nlp_dims_set_constraints(nlp_config, nlp_dims, N, "nh", &nh[N]);
    ocp_nlp_dims_set_constraints(nlp_config, nlp_dims, N, "nsh", &nsh[N]);
    ocp_nlp_dims_set_cost(nlp_config, nlp_dims, N, "ny", &ny[N]);
    free(intNp1mem);

    return nlp_dims;
}


/**
 * Internal function for flying_hand_fully_actuated_acados_create: step 3
 */
void flying_hand_fully_actuated_acados_create_setup_functions(flying_hand_fully_actuated_solver_capsule* capsule)
{
    const int N = capsule->nlp_solver_plan->N;

    /************************************************
    *  external functions
    ************************************************/

#define MAP_CASADI_FNC(__CAPSULE_FNC__, __MODEL_BASE_FNC__) do{ \
        capsule->__CAPSULE_FNC__.casadi_fun = & __MODEL_BASE_FNC__ ;\
        capsule->__CAPSULE_FNC__.casadi_n_in = & __MODEL_BASE_FNC__ ## _n_in; \
        capsule->__CAPSULE_FNC__.casadi_n_out = & __MODEL_BASE_FNC__ ## _n_out; \
        capsule->__CAPSULE_FNC__.casadi_sparsity_in = & __MODEL_BASE_FNC__ ## _sparsity_in; \
        capsule->__CAPSULE_FNC__.casadi_sparsity_out = & __MODEL_BASE_FNC__ ## _sparsity_out; \
        capsule->__CAPSULE_FNC__.casadi_work = & __MODEL_BASE_FNC__ ## _work; \
        external_function_external_param_casadi_create(&capsule->__CAPSULE_FNC__, &ext_fun_opts); \
    } while(false)

    external_function_opts ext_fun_opts;
    external_function_opts_set_to_default(&ext_fun_opts);


    ext_fun_opts.external_workspace = true;
    if (N > 0)
    {
        MAP_CASADI_FNC(nl_constr_h_0_fun_jac, flying_hand_fully_actuated_constr_h_0_fun_jac_uxt_zt);
        MAP_CASADI_FNC(nl_constr_h_0_fun, flying_hand_fully_actuated_constr_h_0_fun);
        // constraints.constr_type == "BGH" and dims.nh > 0
        capsule->nl_constr_h_fun_jac = (external_function_external_param_casadi *) malloc(sizeof(external_function_external_param_casadi)*(N-1));
        for (int i = 0; i < N-1; i++) {
            MAP_CASADI_FNC(nl_constr_h_fun_jac[i], flying_hand_fully_actuated_constr_h_fun_jac_uxt_zt);
        }
        capsule->nl_constr_h_fun = (external_function_external_param_casadi *) malloc(sizeof(external_function_external_param_casadi)*(N-1));
        for (int i = 0; i < N-1; i++) {
            MAP_CASADI_FNC(nl_constr_h_fun[i], flying_hand_fully_actuated_constr_h_fun);
        }
    
        // nonlinear least squares function
        MAP_CASADI_FNC(cost_y_0_fun, flying_hand_fully_actuated_cost_y_0_fun);
        MAP_CASADI_FNC(cost_y_0_fun_jac_ut_xt, flying_hand_fully_actuated_cost_y_0_fun_jac_ut_xt);



    
        // explicit ode
        capsule->expl_vde_forw = (external_function_external_param_casadi *) malloc(sizeof(external_function_external_param_casadi)*N);
        for (int i = 0; i < N; i++) {
            MAP_CASADI_FNC(expl_vde_forw[i], flying_hand_fully_actuated_expl_vde_forw);
        }

        

        capsule->expl_ode_fun = (external_function_external_param_casadi *) malloc(sizeof(external_function_external_param_casadi)*N);
        for (int i = 0; i < N; i++) {
            MAP_CASADI_FNC(expl_ode_fun[i], flying_hand_fully_actuated_expl_ode_fun);
        }

        capsule->expl_vde_adj = (external_function_external_param_casadi *) malloc(sizeof(external_function_external_param_casadi)*N);
        for (int i = 0; i < N; i++) {
            MAP_CASADI_FNC(expl_vde_adj[i], flying_hand_fully_actuated_expl_vde_adj);
        }

    
        // nonlinear least squares cost
        capsule->cost_y_fun = (external_function_external_param_casadi *) malloc(sizeof(external_function_external_param_casadi)*(N-1));
        for (int i = 0; i < N-1; i++)
        {
            MAP_CASADI_FNC(cost_y_fun[i], flying_hand_fully_actuated_cost_y_fun);
        }

        capsule->cost_y_fun_jac_ut_xt = (external_function_external_param_casadi *) malloc(sizeof(external_function_external_param_casadi)*(N-1));
        for (int i = 0; i < N-1; i++)
        {
            MAP_CASADI_FNC(cost_y_fun_jac_ut_xt[i], flying_hand_fully_actuated_cost_y_fun_jac_ut_xt);
        }
    } // N > 0
    // nonlinear least square function
    MAP_CASADI_FNC(cost_y_e_fun, flying_hand_fully_actuated_cost_y_e_fun);
    MAP_CASADI_FNC(cost_y_e_fun_jac_ut_xt, flying_hand_fully_actuated_cost_y_e_fun_jac_ut_xt);

#undef MAP_CASADI_FNC
}


/**
 * Internal function for flying_hand_fully_actuated_acados_create: step 5
 */
void flying_hand_fully_actuated_acados_create_set_default_parameters(flying_hand_fully_actuated_solver_capsule* capsule)
{

    const int N = capsule->nlp_solver_plan->N;

    // initialize parameters to initial value
    
    double* p = malloc(NP*sizeof(double));
    memcpy(p, p_init, NP*sizeof(double));

    for (int i = 0; i <= N; i++) {
        flying_hand_fully_actuated_acados_update_params(capsule, i, p, NP);
    }
    free(p);


    // no global parameters defined
}


/**
 * Internal function for flying_hand_fully_actuated_acados_create: step 5
 */
void flying_hand_fully_actuated_acados_create_setup_nlp_in_numerical_values(flying_hand_fully_actuated_solver_capsule* capsule, const int N, double* new_time_steps)
{
    assert(N == capsule->nlp_solver_plan->N);
    ocp_nlp_config* nlp_config = capsule->nlp_config;
    ocp_nlp_dims* nlp_dims = capsule->nlp_dims;

    int tmp_int = 0;

    /************************************************
    *  nlp_in
    ************************************************/
    ocp_nlp_in * nlp_in = capsule->nlp_in;
    /************************************************
    *  nlp_out
    ************************************************/
    ocp_nlp_out * nlp_out = capsule->nlp_out;

    // set up time_steps and cost_scaling

    if (new_time_steps)
    {
        // NOTE: this sets scaling and time_steps
        flying_hand_fully_actuated_acados_update_time_steps(capsule, N, new_time_steps);
    }
    else
    {
        // set time_steps
    
        double time_step = 0.025;
        for (int i = 0; i < N; i++)
        {
            ocp_nlp_in_set(nlp_config, nlp_dims, nlp_in, i, "Ts", &time_step);
        }
        // set cost scaling
        double* cost_scaling = malloc((N+1)*sizeof(double));
        cost_scaling[0] = 0.025;
        cost_scaling[1] = 0.025;
        cost_scaling[2] = 0.025;
        cost_scaling[3] = 0.025;
        cost_scaling[4] = 0.025;
        cost_scaling[5] = 0.025;
        cost_scaling[6] = 0.025;
        cost_scaling[7] = 0.025;
        cost_scaling[8] = 0.025;
        cost_scaling[9] = 0.025;
        cost_scaling[10] = 0.025;
        cost_scaling[11] = 0.025;
        cost_scaling[12] = 0.025;
        cost_scaling[13] = 0.025;
        cost_scaling[14] = 0.025;
        cost_scaling[15] = 0.025;
        cost_scaling[16] = 0.025;
        cost_scaling[17] = 0.025;
        cost_scaling[18] = 0.025;
        cost_scaling[19] = 0.025;
        cost_scaling[20] = 0.025;
        cost_scaling[21] = 0.025;
        cost_scaling[22] = 0.025;
        cost_scaling[23] = 0.025;
        cost_scaling[24] = 0.025;
        cost_scaling[25] = 0.025;
        cost_scaling[26] = 0.025;
        cost_scaling[27] = 0.025;
        cost_scaling[28] = 0.025;
        cost_scaling[29] = 0.025;
        cost_scaling[30] = 0.025;
        cost_scaling[31] = 0.025;
        cost_scaling[32] = 0.025;
        cost_scaling[33] = 0.025;
        cost_scaling[34] = 0.025;
        cost_scaling[35] = 0.025;
        cost_scaling[36] = 0.025;
        cost_scaling[37] = 0.025;
        cost_scaling[38] = 0.025;
        cost_scaling[39] = 0.025;
        cost_scaling[40] = 0.025;
        cost_scaling[41] = 0.025;
        cost_scaling[42] = 0.025;
        cost_scaling[43] = 0.025;
        cost_scaling[44] = 0.025;
        cost_scaling[45] = 0.025;
        cost_scaling[46] = 0.025;
        cost_scaling[47] = 0.025;
        cost_scaling[48] = 0.025;
        cost_scaling[49] = 0.025;
        cost_scaling[50] = 0.025;
        cost_scaling[51] = 0.025;
        cost_scaling[52] = 0.025;
        cost_scaling[53] = 0.025;
        cost_scaling[54] = 0.025;
        cost_scaling[55] = 0.025;
        cost_scaling[56] = 0.025;
        cost_scaling[57] = 0.025;
        cost_scaling[58] = 0.025;
        cost_scaling[59] = 0.025;
        cost_scaling[60] = 0.025;
        cost_scaling[61] = 0.025;
        cost_scaling[62] = 0.025;
        cost_scaling[63] = 0.025;
        cost_scaling[64] = 0.025;
        cost_scaling[65] = 0.025;
        cost_scaling[66] = 0.025;
        cost_scaling[67] = 0.025;
        cost_scaling[68] = 0.025;
        cost_scaling[69] = 0.025;
        cost_scaling[70] = 0.025;
        cost_scaling[71] = 0.025;
        cost_scaling[72] = 0.025;
        cost_scaling[73] = 0.025;
        cost_scaling[74] = 0.025;
        cost_scaling[75] = 0.025;
        cost_scaling[76] = 0.025;
        cost_scaling[77] = 0.025;
        cost_scaling[78] = 0.025;
        cost_scaling[79] = 0.025;
        cost_scaling[80] = 0.025;
        cost_scaling[81] = 0.025;
        cost_scaling[82] = 0.025;
        cost_scaling[83] = 0.025;
        cost_scaling[84] = 0.025;
        cost_scaling[85] = 0.025;
        cost_scaling[86] = 0.025;
        cost_scaling[87] = 0.025;
        cost_scaling[88] = 0.025;
        cost_scaling[89] = 0.025;
        cost_scaling[90] = 0.025;
        cost_scaling[91] = 0.025;
        cost_scaling[92] = 0.025;
        cost_scaling[93] = 0.025;
        cost_scaling[94] = 0.025;
        cost_scaling[95] = 0.025;
        cost_scaling[96] = 0.025;
        cost_scaling[97] = 0.025;
        cost_scaling[98] = 0.025;
        cost_scaling[99] = 0.025;
        cost_scaling[100] = 1;
        for (int i = 0; i <= N; i++)
        {
            ocp_nlp_cost_model_set(nlp_config, nlp_dims, nlp_in, i, "scaling", &cost_scaling[i]);
        }
        free(cost_scaling);
    }



    /**** Cost ****/
    double* yref_0 = calloc(NY0, sizeof(double));
    // change only the non-zero elements:
    ocp_nlp_cost_model_set(nlp_config, nlp_dims, nlp_in, 0, "yref", yref_0);
    free(yref_0);

   double* W_0 = calloc(NY0*NY0, sizeof(double));
    // change only the non-zero elements:
    W_0[0+(NY0) * 0] = 12;
    W_0[1+(NY0) * 1] = 12;
    W_0[2+(NY0) * 2] = 12;
    W_0[3+(NY0) * 3] = 10;
    W_0[4+(NY0) * 4] = 10;
    W_0[5+(NY0) * 5] = 10;
    W_0[6+(NY0) * 6] = 0.1;
    W_0[7+(NY0) * 7] = 0.1;
    W_0[8+(NY0) * 8] = 0.1;
    W_0[9+(NY0) * 9] = 0.1;
    W_0[10+(NY0) * 10] = 0.1;
    W_0[11+(NY0) * 11] = 0.1;
    W_0[12+(NY0) * 12] = 0.1;
    W_0[13+(NY0) * 13] = 0.1;
    W_0[14+(NY0) * 14] = 0.1;
    W_0[15+(NY0) * 15] = 0.1;
    W_0[16+(NY0) * 16] = 0.03;
    W_0[17+(NY0) * 17] = 0.03;
    W_0[18+(NY0) * 18] = 0.03;
    W_0[19+(NY0) * 19] = 0.1;
    W_0[20+(NY0) * 20] = 0.1;
    W_0[21+(NY0) * 21] = 0.1;
    W_0[22+(NY0) * 22] = 0.03;
    W_0[23+(NY0) * 23] = 0.03;
    W_0[24+(NY0) * 24] = 0.03;
    W_0[25+(NY0) * 25] = 0.03;
    W_0[26+(NY0) * 26] = 200;
    ocp_nlp_cost_model_set(nlp_config, nlp_dims, nlp_in, 0, "W", W_0);
    free(W_0);
    double* yref = calloc(NY, sizeof(double));
    // change only the non-zero elements:

    for (int i = 1; i < N; i++)
    {
        ocp_nlp_cost_model_set(nlp_config, nlp_dims, nlp_in, i, "yref", yref);
    }
    free(yref);
    double* W = calloc(NY*NY, sizeof(double));
    // change only the non-zero elements:
    W[0+(NY) * 0] = 12;
    W[1+(NY) * 1] = 12;
    W[2+(NY) * 2] = 12;
    W[3+(NY) * 3] = 10;
    W[4+(NY) * 4] = 10;
    W[5+(NY) * 5] = 10;
    W[6+(NY) * 6] = 0.1;
    W[7+(NY) * 7] = 0.1;
    W[8+(NY) * 8] = 0.1;
    W[9+(NY) * 9] = 0.1;
    W[10+(NY) * 10] = 0.1;
    W[11+(NY) * 11] = 0.1;
    W[12+(NY) * 12] = 0.1;
    W[13+(NY) * 13] = 0.1;
    W[14+(NY) * 14] = 0.1;
    W[15+(NY) * 15] = 0.1;
    W[16+(NY) * 16] = 0.03;
    W[17+(NY) * 17] = 0.03;
    W[18+(NY) * 18] = 0.03;
    W[19+(NY) * 19] = 0.1;
    W[20+(NY) * 20] = 0.1;
    W[21+(NY) * 21] = 0.1;
    W[22+(NY) * 22] = 0.03;
    W[23+(NY) * 23] = 0.03;
    W[24+(NY) * 24] = 0.03;
    W[25+(NY) * 25] = 0.03;
    W[26+(NY) * 26] = 200;

    for (int i = 1; i < N; i++)
    {
        ocp_nlp_cost_model_set(nlp_config, nlp_dims, nlp_in, i, "W", W);
    }
    free(W);
    double* yref_e = calloc(NYN, sizeof(double));
    // change only the non-zero elements:
    ocp_nlp_cost_model_set(nlp_config, nlp_dims, nlp_in, N, "yref", yref_e);
    free(yref_e);

    double* W_e = calloc(NYN*NYN, sizeof(double));
    // change only the non-zero elements:
    W_e[0+(NYN) * 0] = 12;
    W_e[1+(NYN) * 1] = 12;
    W_e[2+(NYN) * 2] = 12;
    W_e[3+(NYN) * 3] = 10;
    W_e[4+(NYN) * 4] = 10;
    W_e[5+(NYN) * 5] = 10;
    W_e[6+(NYN) * 6] = 0.1;
    W_e[7+(NYN) * 7] = 0.1;
    W_e[8+(NYN) * 8] = 0.1;
    W_e[9+(NYN) * 9] = 0.1;
    W_e[10+(NYN) * 10] = 0.1;
    W_e[11+(NYN) * 11] = 0.1;
    W_e[12+(NYN) * 12] = 0.1;
    W_e[13+(NYN) * 13] = 0.1;
    W_e[14+(NYN) * 14] = 0.1;
    W_e[15+(NYN) * 15] = 0.1;
    W_e[16+(NYN) * 16] = 200;
    ocp_nlp_cost_model_set(nlp_config, nlp_dims, nlp_in, N, "W", W_e);
    free(W_e);






    /**** Constraints ****/

    // bounds for initial stage
    // x0
    int* idxbx0 = malloc(NBX0 * sizeof(int));
    idxbx0[0] = 0;
    idxbx0[1] = 1;
    idxbx0[2] = 2;
    idxbx0[3] = 3;
    idxbx0[4] = 4;
    idxbx0[5] = 5;
    idxbx0[6] = 6;
    idxbx0[7] = 7;
    idxbx0[8] = 8;
    idxbx0[9] = 9;
    idxbx0[10] = 10;
    idxbx0[11] = 11;
    idxbx0[12] = 12;
    idxbx0[13] = 13;
    idxbx0[14] = 14;
    idxbx0[15] = 15;
    idxbx0[16] = 16;

    double* lubx0 = calloc(2*NBX0, sizeof(double));
    double* lbx0 = lubx0;
    double* ubx0 = lubx0 + NBX0;
    // change only the non-zero elements:
    lbx0[3] = 1;
    ubx0[3] = 1;

    ocp_nlp_constraints_model_set(nlp_config, nlp_dims, nlp_in, nlp_out, 0, "idxbx", idxbx0);
    ocp_nlp_constraints_model_set(nlp_config, nlp_dims, nlp_in, nlp_out, 0, "lbx", lbx0);
    ocp_nlp_constraints_model_set(nlp_config, nlp_dims, nlp_in, nlp_out, 0, "ubx", ubx0);
    free(idxbx0);
    free(lubx0);
    // idxbxe_0
    int* idxbxe_0 = malloc(17 * sizeof(int));
    idxbxe_0[0] = 0;
    idxbxe_0[1] = 1;
    idxbxe_0[2] = 2;
    idxbxe_0[3] = 3;
    idxbxe_0[4] = 4;
    idxbxe_0[5] = 5;
    idxbxe_0[6] = 6;
    idxbxe_0[7] = 7;
    idxbxe_0[8] = 8;
    idxbxe_0[9] = 9;
    idxbxe_0[10] = 10;
    idxbxe_0[11] = 11;
    idxbxe_0[12] = 12;
    idxbxe_0[13] = 13;
    idxbxe_0[14] = 14;
    idxbxe_0[15] = 15;
    idxbxe_0[16] = 16;
    ocp_nlp_constraints_model_set(nlp_config, nlp_dims, nlp_in, nlp_out, 0, "idxbxe", idxbxe_0);
    free(idxbxe_0);



    // set up nonlinear constraints for last stage
    double* luh_0 = calloc(2*NH0, sizeof(double));
    double* lh_0 = luh_0;
    double* uh_0 = luh_0 + NH0;
    lh_0[6] = -10;
    lh_0[7] = -10;
    lh_0[8] = -10;
    lh_0[9] = -10;
    uh_0[0] = 100;
    uh_0[1] = 100;
    uh_0[2] = 100;
    uh_0[3] = 100;
    uh_0[4] = 100;
    uh_0[5] = 100;
    uh_0[6] = 10;
    uh_0[7] = 10;
    uh_0[8] = 10;
    uh_0[9] = 10;

    ocp_nlp_constraints_model_set(nlp_config, nlp_dims, nlp_in, nlp_out, 0, "lh", lh_0);
    ocp_nlp_constraints_model_set(nlp_config, nlp_dims, nlp_in, nlp_out, 0, "uh", uh_0);
    free(luh_0);









    /* constraints that are the same for initial and intermediate */
    // u
    int* idxbu = malloc(NBU * sizeof(int));
    idxbu[0] = 6;
    idxbu[1] = 7;
    idxbu[2] = 8;
    idxbu[3] = 9;
    double* lubu = calloc(2*NBU, sizeof(double));
    double* lbu = lubu;
    double* ubu = lubu + NBU;
    lbu[0] = -3.141592653589793;
    ubu[0] = 3.141592653589793;
    lbu[1] = -3.141592653589793;
    ubu[1] = 3.141592653589793;
    lbu[2] = -3.141592653589793;
    ubu[2] = 3.141592653589793;
    lbu[3] = -3.141592653589793;
    ubu[3] = 3.141592653589793;

    for (int i = 0; i < N; i++)
    {
        ocp_nlp_constraints_model_set(nlp_config, nlp_dims, nlp_in, nlp_out, i, "idxbu", idxbu);
        ocp_nlp_constraints_model_set(nlp_config, nlp_dims, nlp_in, nlp_out, i, "lbu", lbu);
        ocp_nlp_constraints_model_set(nlp_config, nlp_dims, nlp_in, nlp_out, i, "ubu", ubu);
    }
    free(idxbu);
    free(lubu);






    /* Path constraints */

    // x
    int* idxbx = malloc(NBX * sizeof(int));
    idxbx[0] = 7;
    idxbx[1] = 8;
    idxbx[2] = 9;
    idxbx[3] = 10;
    idxbx[4] = 11;
    idxbx[5] = 12;
    idxbx[6] = 13;
    idxbx[7] = 14;
    idxbx[8] = 15;
    idxbx[9] = 16;
    double* lubx = calloc(2*NBX, sizeof(double));
    double* lbx = lubx;
    double* ubx = lubx + NBX;
    lbx[0] = -20;
    ubx[0] = 20;
    lbx[1] = -20;
    ubx[1] = 20;
    lbx[2] = -20;
    ubx[2] = 20;
    lbx[3] = -20;
    ubx[3] = 20;
    lbx[4] = -20;
    ubx[4] = 20;
    lbx[5] = -20;
    ubx[5] = 20;
    lbx[6] = -3.141592653589793;
    ubx[6] = 3.141592653589793;
    lbx[7] = -3.141592653589793;
    ubx[7] = 3.141592653589793;
    lbx[8] = -3.141592653589793;
    ubx[8] = 3.141592653589793;
    lbx[9] = -3.141592653589793;
    ubx[9] = 3.141592653589793;

    for (int i = 1; i < N; i++)
    {
        ocp_nlp_constraints_model_set(nlp_config, nlp_dims, nlp_in, nlp_out, i, "idxbx", idxbx);
        ocp_nlp_constraints_model_set(nlp_config, nlp_dims, nlp_in, nlp_out, i, "lbx", lbx);
        ocp_nlp_constraints_model_set(nlp_config, nlp_dims, nlp_in, nlp_out, i, "ubx", ubx);
    }
    free(idxbx);
    free(lubx);


    // set up nonlinear constraints for stage 1 to N-1
    double* luh = calloc(2*NH, sizeof(double));
    double* lh = luh;
    double* uh = luh + NH;
    lh[6] = -10;
    lh[7] = -10;
    lh[8] = -10;
    lh[9] = -10;
    uh[0] = 100;
    uh[1] = 100;
    uh[2] = 100;
    uh[3] = 100;
    uh[4] = 100;
    uh[5] = 100;
    uh[6] = 10;
    uh[7] = 10;
    uh[8] = 10;
    uh[9] = 10;

    for (int i = 1; i < N; i++)
    {
        ocp_nlp_constraints_model_set(nlp_config, nlp_dims, nlp_in, nlp_out, i, "lh", lh);
        ocp_nlp_constraints_model_set(nlp_config, nlp_dims, nlp_in, nlp_out, i, "uh", uh);
    }
    free(luh);











    /* terminal constraints */

    // set up bounds for last stage
    // x
    int* idxbx_e = malloc(NBXN * sizeof(int));
    idxbx_e[0] = 7;
    idxbx_e[1] = 8;
    idxbx_e[2] = 9;
    idxbx_e[3] = 10;
    idxbx_e[4] = 11;
    idxbx_e[5] = 12;
    idxbx_e[6] = 13;
    idxbx_e[7] = 14;
    idxbx_e[8] = 15;
    idxbx_e[9] = 16;
    double* lubx_e = calloc(2*NBXN, sizeof(double));
    double* lbx_e = lubx_e;
    double* ubx_e = lubx_e + NBXN;
    lbx_e[0] = -20;
    ubx_e[0] = 20;
    lbx_e[1] = -20;
    ubx_e[1] = 20;
    lbx_e[2] = -20;
    ubx_e[2] = 20;
    lbx_e[3] = -20;
    ubx_e[3] = 20;
    lbx_e[4] = -20;
    ubx_e[4] = 20;
    lbx_e[5] = -20;
    ubx_e[5] = 20;
    lbx_e[6] = -3.141592653589793;
    ubx_e[6] = 3.141592653589793;
    lbx_e[7] = -3.141592653589793;
    ubx_e[7] = 3.141592653589793;
    lbx_e[8] = -3.141592653589793;
    ubx_e[8] = 3.141592653589793;
    lbx_e[9] = -3.141592653589793;
    ubx_e[9] = 3.141592653589793;
    ocp_nlp_constraints_model_set(nlp_config, nlp_dims, nlp_in, nlp_out, N, "idxbx", idxbx_e);
    ocp_nlp_constraints_model_set(nlp_config, nlp_dims, nlp_in, nlp_out, N, "lbx", lbx_e);
    ocp_nlp_constraints_model_set(nlp_config, nlp_dims, nlp_in, nlp_out, N, "ubx", ubx_e);
    free(idxbx_e);
    free(lubx_e);



















}

// this function only sets external functions, numerical values are set in flying_hand_fully_actuated_acados_create_setup_nlp_in_numerical_values
void flying_hand_fully_actuated_acados_create_setup_nlp_in(flying_hand_fully_actuated_solver_capsule* capsule, const int N)
{
    assert(N == capsule->nlp_solver_plan->N);
    ocp_nlp_config* nlp_config = capsule->nlp_config;
    ocp_nlp_dims* nlp_dims = capsule->nlp_dims;

    /************************************************
    *  nlp_in
    ************************************************/
    ocp_nlp_in * nlp_in = capsule->nlp_in;
    /************************************************
    *  nlp_out
    ************************************************/
    ocp_nlp_out * nlp_out = capsule->nlp_out;


    /**** Dynamics ****/
    for (int i = 0; i < N; i++)
    {
        ocp_nlp_dynamics_model_set_external_param_fun(nlp_config, nlp_dims, nlp_in, i, "expl_vde_forw", &capsule->expl_vde_forw[i]);
        
        ocp_nlp_dynamics_model_set_external_param_fun(nlp_config, nlp_dims, nlp_in, i, "expl_ode_fun", &capsule->expl_ode_fun[i]);
        ocp_nlp_dynamics_model_set_external_param_fun(nlp_config, nlp_dims, nlp_in, i, "expl_vde_adj", &capsule->expl_vde_adj[i]);
    }

    /**** Cost ****/
    ocp_nlp_cost_model_set_external_param_fun(nlp_config, nlp_dims, nlp_in, 0, "nls_y_fun", &capsule->cost_y_0_fun);
    ocp_nlp_cost_model_set_external_param_fun(nlp_config, nlp_dims, nlp_in, 0, "nls_y_fun_jac", &capsule->cost_y_0_fun_jac_ut_xt);
    for (int i = 1; i < N; i++)
    {
        ocp_nlp_cost_model_set_external_param_fun(nlp_config, nlp_dims, nlp_in, i, "nls_y_fun", &capsule->cost_y_fun[i-1]);
        ocp_nlp_cost_model_set_external_param_fun(nlp_config, nlp_dims, nlp_in, i, "nls_y_fun_jac", &capsule->cost_y_fun_jac_ut_xt[i-1]);
    }
    ocp_nlp_cost_model_set_external_param_fun(nlp_config, nlp_dims, nlp_in, N, "nls_y_fun", &capsule->cost_y_e_fun);
    ocp_nlp_cost_model_set_external_param_fun(nlp_config, nlp_dims, nlp_in, N, "nls_y_fun_jac", &capsule->cost_y_e_fun_jac_ut_xt);

    /**** Constraints ****/

    // bounds for initial stage

    // set up nonlinear constraints for initial stage
    ocp_nlp_constraints_model_set_external_param_fun(nlp_config, nlp_dims, nlp_in, 0, "nl_constr_h_fun_jac", &capsule->nl_constr_h_0_fun_jac);
    ocp_nlp_constraints_model_set_external_param_fun(nlp_config, nlp_dims, nlp_in, 0, "nl_constr_h_fun", &capsule->nl_constr_h_0_fun);
    
    
    


    // set up nonlinear constraints for stage 1 to N-1
    for (int i = 1; i < N; i++)
    {
        ocp_nlp_constraints_model_set_external_param_fun(nlp_config, nlp_dims, nlp_in, i, "nl_constr_h_fun_jac",
                                      &capsule->nl_constr_h_fun_jac[i-1]);
        ocp_nlp_constraints_model_set_external_param_fun(nlp_config, nlp_dims, nlp_in, i, "nl_constr_h_fun",
                                      &capsule->nl_constr_h_fun[i-1]);
        
        
        
    }



    /* terminal constraints */
}


static void flying_hand_fully_actuated_acados_create_set_opts(flying_hand_fully_actuated_solver_capsule* capsule)
{
    const int N = capsule->nlp_solver_plan->N;
    ocp_nlp_config* nlp_config = capsule->nlp_config;
    void *nlp_opts = capsule->nlp_opts;

    /************************************************
    *  opts
    ************************************************/



    int fixed_hess = 0;
    ocp_nlp_solver_opts_set(nlp_config, nlp_opts, "fixed_hess", &fixed_hess);

    double globalization_fixed_step_length = 1;
    ocp_nlp_solver_opts_set(nlp_config, nlp_opts, "globalization_fixed_step_length", &globalization_fixed_step_length);




    int with_solution_sens_wrt_params = false;
    ocp_nlp_solver_opts_set(nlp_config, capsule->nlp_opts, "with_solution_sens_wrt_params", &with_solution_sens_wrt_params);

    int with_value_sens_wrt_params = false;
    ocp_nlp_solver_opts_set(nlp_config, capsule->nlp_opts, "with_value_sens_wrt_params", &with_value_sens_wrt_params);

    double solution_sens_qp_t_lam_min = 0.000000001;
    ocp_nlp_solver_opts_set(nlp_config, capsule->nlp_opts, "solution_sens_qp_t_lam_min", &solution_sens_qp_t_lam_min);

    int globalization_full_step_dual = 0;
    ocp_nlp_solver_opts_set(nlp_config, capsule->nlp_opts, "globalization_full_step_dual", &globalization_full_step_dual);

    // set collocation type (relevant for implicit integrators)
    sim_collocation_type collocation_type = GAUSS_LEGENDRE;
    for (int i = 0; i < N; i++)
        ocp_nlp_solver_opts_set_at_stage(nlp_config, nlp_opts, i, "dynamics_collocation_type", &collocation_type);

    // set up sim_method_num_steps
    // all sim_method_num_steps are identical
    int sim_method_num_steps = 1;
    for (int i = 0; i < N; i++)
        ocp_nlp_solver_opts_set_at_stage(nlp_config, nlp_opts, i, "dynamics_num_steps", &sim_method_num_steps);

    // set up sim_method_num_stages
    // all sim_method_num_stages are identical
    int sim_method_num_stages = 4;
    for (int i = 0; i < N; i++)
        ocp_nlp_solver_opts_set_at_stage(nlp_config, nlp_opts, i, "dynamics_num_stages", &sim_method_num_stages);

    int newton_iter_val = 3;
    for (int i = 0; i < N; i++)
        ocp_nlp_solver_opts_set_at_stage(nlp_config, nlp_opts, i, "dynamics_newton_iter", &newton_iter_val);

    double newton_tol_val = 0;
    for (int i = 0; i < N; i++)
        ocp_nlp_solver_opts_set_at_stage(nlp_config, nlp_opts, i, "dynamics_newton_tol", &newton_tol_val);

    // set up sim_method_jac_reuse
    bool tmp_bool = (bool) 0;
    for (int i = 0; i < N; i++)
        ocp_nlp_solver_opts_set_at_stage(nlp_config, nlp_opts, i, "dynamics_jac_reuse", &tmp_bool);

    double levenberg_marquardt = 0;
    ocp_nlp_solver_opts_set(nlp_config, nlp_opts, "levenberg_marquardt", &levenberg_marquardt);

    /* options QP solver */
    int qp_solver_cond_N;const int qp_solver_cond_N_ori = 20;
    qp_solver_cond_N = N < qp_solver_cond_N_ori ? N : qp_solver_cond_N_ori; // use the minimum value here
    ocp_nlp_solver_opts_set(nlp_config, nlp_opts, "qp_cond_N", &qp_solver_cond_N);

    int nlp_solver_ext_qp_res = 0;
    ocp_nlp_solver_opts_set(nlp_config, nlp_opts, "ext_qp_res", &nlp_solver_ext_qp_res);

    bool store_iterates = false;
    ocp_nlp_solver_opts_set(nlp_config, nlp_opts, "store_iterates", &store_iterates);
    // set HPIPM mode: should be done before setting other QP solver options
    ocp_nlp_solver_opts_set(nlp_config, nlp_opts, "qp_hpipm_mode", "BALANCE");



    int qp_solver_t0_init = 2;
    ocp_nlp_solver_opts_set(nlp_config, nlp_opts, "qp_t0_init", &qp_solver_t0_init);




    int as_rti_iter = 1;
    ocp_nlp_solver_opts_set(nlp_config, nlp_opts, "as_rti_iter", &as_rti_iter);

    int as_rti_level = 4;
    ocp_nlp_solver_opts_set(nlp_config, nlp_opts, "as_rti_level", &as_rti_level);

    int rti_log_residuals = 0;
    ocp_nlp_solver_opts_set(nlp_config, nlp_opts, "rti_log_residuals", &rti_log_residuals);

    int rti_log_only_available_residuals = 0;
    ocp_nlp_solver_opts_set(nlp_config, nlp_opts, "rti_log_only_available_residuals", &rti_log_only_available_residuals);

    bool with_anderson_acceleration = false;
    ocp_nlp_solver_opts_set(nlp_config, nlp_opts, "with_anderson_acceleration", &with_anderson_acceleration);

    double anderson_activation_threshold = 10;
    ocp_nlp_solver_opts_set(nlp_config, nlp_opts, "anderson_activation_threshold", &anderson_activation_threshold);

    int qp_solver_iter_max = 50;
    ocp_nlp_solver_opts_set(nlp_config, nlp_opts, "qp_iter_max", &qp_solver_iter_max);



    int print_level = 0;
    ocp_nlp_solver_opts_set(nlp_config, nlp_opts, "print_level", &print_level);
    int qp_solver_cond_ric_alg = 1;
    ocp_nlp_solver_opts_set(nlp_config, nlp_opts, "qp_cond_ric_alg", &qp_solver_cond_ric_alg);

    int qp_solver_ric_alg = 1;
    ocp_nlp_solver_opts_set(nlp_config, nlp_opts, "qp_ric_alg", &qp_solver_ric_alg);


    int ext_cost_num_hess = 0;
}


/**
 * Internal function for flying_hand_fully_actuated_acados_create: step 7
 */
void flying_hand_fully_actuated_acados_set_nlp_out(flying_hand_fully_actuated_solver_capsule* capsule)
{
    const int N = capsule->nlp_solver_plan->N;
    ocp_nlp_config* nlp_config = capsule->nlp_config;
    ocp_nlp_dims* nlp_dims = capsule->nlp_dims;
    ocp_nlp_out* nlp_out = capsule->nlp_out;
    ocp_nlp_in* nlp_in = capsule->nlp_in;

    // initialize primal solution
    double* xu0 = calloc(NX+NU, sizeof(double));
    double* x0 = xu0;

    // initialize with x0
    x0[3] = 1;


    double* u0 = xu0 + NX;

    for (int i = 0; i < N; i++)
    {
        // x0
        ocp_nlp_out_set(nlp_config, nlp_dims, nlp_out, nlp_in, i, "x", x0);
        // u0
        ocp_nlp_out_set(nlp_config, nlp_dims, nlp_out, nlp_in, i, "u", u0);
    }
    ocp_nlp_out_set(nlp_config, nlp_dims, nlp_out, nlp_in, N, "x", x0);
    free(xu0);
}


/**
 * Internal function for flying_hand_fully_actuated_acados_create: step 9
 */
int flying_hand_fully_actuated_acados_create_precompute(flying_hand_fully_actuated_solver_capsule* capsule) {
    int status = ocp_nlp_precompute(capsule->nlp_solver, capsule->nlp_in, capsule->nlp_out);

    if (status != ACADOS_SUCCESS) {
        printf("\nocp_nlp_precompute failed!\n\n");
        exit(1);
    }

    return status;
}


int flying_hand_fully_actuated_acados_create_with_discretization(flying_hand_fully_actuated_solver_capsule* capsule, int N, double* new_time_steps)
{
    // If N does not match the number of shooting intervals used for code generation, new_time_steps must be given.
    if (N != FLYING_HAND_FULLY_ACTUATED_N && !new_time_steps) {
        fprintf(stderr, "flying_hand_fully_actuated_acados_create_with_discretization: new_time_steps is NULL " \
            "but the number of shooting intervals (= %d) differs from the number of " \
            "shooting intervals (= %d) during code generation! Please provide a new vector of time_stamps!\n", \
             N, FLYING_HAND_FULLY_ACTUATED_N);
        return 1;
    }

    // number of expected runtime parameters
    capsule->nlp_np = NP;

    // 1) create and set nlp_solver_plan; create nlp_config
    capsule->nlp_solver_plan = ocp_nlp_plan_create(N);
    flying_hand_fully_actuated_acados_create_set_plan(capsule->nlp_solver_plan, N);
    capsule->nlp_config = ocp_nlp_config_create(*capsule->nlp_solver_plan);

    // 2) create and set dimensions
    capsule->nlp_dims = flying_hand_fully_actuated_acados_create_setup_dimensions(capsule);

    // 3) create and set nlp_opts
    capsule->nlp_opts = ocp_nlp_solver_opts_create(capsule->nlp_config, capsule->nlp_dims);
    flying_hand_fully_actuated_acados_create_set_opts(capsule);

    // 4) create and set nlp_out
    // 4.1) nlp_out
    capsule->nlp_out = ocp_nlp_out_create(capsule->nlp_config, capsule->nlp_dims);
    // 4.2) sens_out
    capsule->sens_out = ocp_nlp_out_create(capsule->nlp_config, capsule->nlp_dims);
    flying_hand_fully_actuated_acados_set_nlp_out(capsule);

    // 5) create nlp_in
    capsule->nlp_in = ocp_nlp_in_create(capsule->nlp_config, capsule->nlp_dims);

    // 6) setup functions, nlp_in and default parameters
    flying_hand_fully_actuated_acados_create_setup_functions(capsule);
    flying_hand_fully_actuated_acados_create_setup_nlp_in(capsule, N);
    flying_hand_fully_actuated_acados_create_setup_nlp_in_numerical_values(capsule, N, new_time_steps);
    flying_hand_fully_actuated_acados_create_set_default_parameters(capsule);

    // 7) create solver
    capsule->nlp_solver = ocp_nlp_solver_create(capsule->nlp_config, capsule->nlp_dims, capsule->nlp_opts, capsule->nlp_in);


    // 8) do precomputations
    int status = flying_hand_fully_actuated_acados_create_precompute(capsule);

    return status;
}

/**
 * This function is for updating an already initialized solver with a different number of qp_cond_N. It is useful for code reuse after code export.
 */
int flying_hand_fully_actuated_acados_update_qp_solver_cond_N(flying_hand_fully_actuated_solver_capsule* capsule, int qp_solver_cond_N)
{
    // 1) destroy solver
    ocp_nlp_solver_destroy(capsule->nlp_solver);

    // 2) set new value for "qp_cond_N"
    const int N = capsule->nlp_solver_plan->N;
    if(qp_solver_cond_N > N)
        printf("Warning: qp_solver_cond_N = %d > N = %d\n", qp_solver_cond_N, N);
    ocp_nlp_solver_opts_set(capsule->nlp_config, capsule->nlp_opts, "qp_cond_N", &qp_solver_cond_N);

    // 3) continue with the remaining steps from flying_hand_fully_actuated_acados_create_with_discretization(...):
    // -> 8) create solver
    capsule->nlp_solver = ocp_nlp_solver_create(capsule->nlp_config, capsule->nlp_dims, capsule->nlp_opts, capsule->nlp_in);

    // -> 9) do precomputations
    int status = flying_hand_fully_actuated_acados_create_precompute(capsule);
    return status;
}


int flying_hand_fully_actuated_acados_reset(flying_hand_fully_actuated_solver_capsule* capsule, int reset_qp_solver_mem, int reset_numerical_values, int reset_solver_options, int reset_x_to_x0_bar)
{
    // set initialization to all zeros
    const int N = capsule->nlp_solver_plan->N;
    ocp_nlp_config* nlp_config = capsule->nlp_config;
    ocp_nlp_dims* nlp_dims = capsule->nlp_dims;
    ocp_nlp_out* nlp_out = capsule->nlp_out;
    ocp_nlp_in* nlp_in = capsule->nlp_in;
    ocp_nlp_solver* nlp_solver = capsule->nlp_solver;

    // sets primal and dual iterates to zero
    ocp_nlp_out_set_values_to_zero(nlp_config, nlp_dims, nlp_out);

    // reset integrator memory
    ocp_nlp_solver_reset_integrator_memory(nlp_solver, nlp_in, nlp_out);
    // get qp_status: if NaN -> reset memory
    int qp_status;
    ocp_nlp_get(capsule->nlp_solver, "qp_status", &qp_status);
    if (reset_qp_solver_mem || (qp_status == 3))
    {
        // printf("\nin reset qp_status %d -> resetting QP memory\n", qp_status);
        ocp_nlp_solver_reset_qp_memory(nlp_solver, nlp_in, nlp_out);
    }

    if (reset_numerical_values)
    {
        // reset parameters to initial values
        flying_hand_fully_actuated_acados_create_set_default_parameters(capsule);

        // reset numerical values in nlp_in
        flying_hand_fully_actuated_acados_create_setup_nlp_in_numerical_values(capsule, N, NULL);
    }

    if (reset_solver_options)
    {
        // reset solver options to initial values
        flying_hand_fully_actuated_acados_create_set_opts(capsule);
    }

    if (reset_x_to_x0_bar)
    {double* buffer = calloc(NX, sizeof(double));
        ocp_nlp_constraints_model_get(nlp_config, nlp_dims, nlp_in, 0, "lbx", buffer);
        for (int i=0; i<N+1; i++)
        {
            ocp_nlp_out_set(nlp_config, nlp_dims, nlp_out, nlp_in, i, "x", buffer);
        }
        free(buffer);
    }
    return 0;
}




int flying_hand_fully_actuated_acados_update_params(flying_hand_fully_actuated_solver_capsule* capsule, int stage, double *p, int np)
{
    int solver_status = 0;

    int casadi_np = 84;
    if (casadi_np != np) {
        printf("acados_update_params: trying to set %i parameters for external functions."
            " External function has %i parameters. Exiting.\n", np, casadi_np);
        exit(1);
    }
    ocp_nlp_in_set(capsule->nlp_config, capsule->nlp_dims, capsule->nlp_in, stage, "parameter_values", p);

    return solver_status;
}


int flying_hand_fully_actuated_acados_update_params_sparse(flying_hand_fully_actuated_solver_capsule * capsule, int stage, int *idx, double *p, int n_update)
{
    ocp_nlp_in_set_params_sparse(capsule->nlp_config, capsule->nlp_dims, capsule->nlp_in, stage, idx, p, n_update);

    return 0;
}


int flying_hand_fully_actuated_acados_set_p_global_and_precompute_dependencies(flying_hand_fully_actuated_solver_capsule* capsule, double* data, int data_len)
{

    // printf("No global_data, flying_hand_fully_actuated_acados_set_p_global_and_precompute_dependencies does nothing.\n");
    return 0;
}




int flying_hand_fully_actuated_acados_solve(flying_hand_fully_actuated_solver_capsule* capsule)
{
    // solve NLP
    int solver_status = ocp_nlp_solve(capsule->nlp_solver, capsule->nlp_in, capsule->nlp_out);

    return solver_status;
}



int flying_hand_fully_actuated_acados_setup_qp_matrices_and_factorize(flying_hand_fully_actuated_solver_capsule* capsule)
{
    int solver_status = ocp_nlp_setup_qp_matrices_and_factorize(capsule->nlp_solver, capsule->nlp_in, capsule->nlp_out);

    return solver_status;
}






int flying_hand_fully_actuated_acados_free(flying_hand_fully_actuated_solver_capsule* capsule)
{
    // before destroying, keep some info
    const int N = capsule->nlp_solver_plan->N;
    // free memory
    ocp_nlp_solver_opts_destroy(capsule->nlp_opts);
    ocp_nlp_in_destroy(capsule->nlp_in);
    ocp_nlp_out_destroy(capsule->nlp_out);
    ocp_nlp_out_destroy(capsule->sens_out);
    ocp_nlp_solver_destroy(capsule->nlp_solver);
    ocp_nlp_dims_destroy(capsule->nlp_dims);
    ocp_nlp_config_destroy(capsule->nlp_config);
    ocp_nlp_plan_destroy(capsule->nlp_solver_plan);

    /* free external function */
    // dynamics
    for (int i = 0; i < N; i++)
    {
        external_function_external_param_casadi_free(&capsule->expl_vde_forw[i]);
        
        external_function_external_param_casadi_free(&capsule->expl_ode_fun[i]);
        external_function_external_param_casadi_free(&capsule->expl_vde_adj[i]);
    }
    free(capsule->expl_vde_adj);
    free(capsule->expl_vde_forw);
    
    free(capsule->expl_ode_fun);

    // cost
    external_function_external_param_casadi_free(&capsule->cost_y_0_fun);
    external_function_external_param_casadi_free(&capsule->cost_y_0_fun_jac_ut_xt);
    for (int i = 0; i < N - 1; i++)
    {
        external_function_external_param_casadi_free(&capsule->cost_y_fun[i]);
        external_function_external_param_casadi_free(&capsule->cost_y_fun_jac_ut_xt[i]);
    }
    free(capsule->cost_y_fun);
    free(capsule->cost_y_fun_jac_ut_xt);
    external_function_external_param_casadi_free(&capsule->cost_y_e_fun);
    external_function_external_param_casadi_free(&capsule->cost_y_e_fun_jac_ut_xt);

    // constraints
    for (int i = 0; i < N-1; i++)
    {
        external_function_external_param_casadi_free(&capsule->nl_constr_h_fun_jac[i]);
        external_function_external_param_casadi_free(&capsule->nl_constr_h_fun[i]);
    }
    free(capsule->nl_constr_h_fun_jac);
    free(capsule->nl_constr_h_fun);
    external_function_external_param_casadi_free(&capsule->nl_constr_h_0_fun_jac);
    external_function_external_param_casadi_free(&capsule->nl_constr_h_0_fun);



    return 0;
}


void flying_hand_fully_actuated_acados_print_stats(flying_hand_fully_actuated_solver_capsule* capsule)
{
    int nlp_iter, stat_m, stat_n, tmp_int;
    ocp_nlp_get(capsule->nlp_solver, "nlp_iter", &nlp_iter);
    ocp_nlp_get(capsule->nlp_solver, "stat_n", &stat_n);
    ocp_nlp_get(capsule->nlp_solver, "stat_m", &stat_m);


    int stat_n_max = 16;
    if (stat_n > stat_n_max)
    {
        printf("stat_n_max = %d is too small, increase it in the template!\n", stat_n_max);
        exit(1);
    }
    double stat[1616];
    ocp_nlp_get(capsule->nlp_solver, "statistics", stat);

    int nrow = nlp_iter+1 < stat_m ? nlp_iter+1 : stat_m;


    printf("iter\tqp_stat\tqp_iter\n");
    for (int i = 0; i < nrow; i++)
    {
        for (int j = 0; j < stat_n + 1; j++)
        {
            tmp_int = (int) stat[i + j * nrow];
            printf("%d\t", tmp_int);
        }
        printf("\n");
    }
}

int flying_hand_fully_actuated_acados_custom_update(flying_hand_fully_actuated_solver_capsule* capsule, double* data, int data_len)
{
    (void)capsule;
    (void)data;
    (void)data_len;
    printf("\ndummy function that can be called in between solver calls to update parameters or numerical data efficiently in C.\n");
    printf("nothing set yet..\n");
    return 1;

}



ocp_nlp_in *flying_hand_fully_actuated_acados_get_nlp_in(flying_hand_fully_actuated_solver_capsule* capsule) { return capsule->nlp_in; }
ocp_nlp_out *flying_hand_fully_actuated_acados_get_nlp_out(flying_hand_fully_actuated_solver_capsule* capsule) { return capsule->nlp_out; }
ocp_nlp_out *flying_hand_fully_actuated_acados_get_sens_out(flying_hand_fully_actuated_solver_capsule* capsule) { return capsule->sens_out; }
ocp_nlp_solver *flying_hand_fully_actuated_acados_get_nlp_solver(flying_hand_fully_actuated_solver_capsule* capsule) { return capsule->nlp_solver; }
ocp_nlp_config *flying_hand_fully_actuated_acados_get_nlp_config(flying_hand_fully_actuated_solver_capsule* capsule) { return capsule->nlp_config; }
void *flying_hand_fully_actuated_acados_get_nlp_opts(flying_hand_fully_actuated_solver_capsule* capsule) { return capsule->nlp_opts; }
ocp_nlp_dims *flying_hand_fully_actuated_acados_get_nlp_dims(flying_hand_fully_actuated_solver_capsule* capsule) { return capsule->nlp_dims; }
ocp_nlp_plan_t *flying_hand_fully_actuated_acados_get_nlp_plan(flying_hand_fully_actuated_solver_capsule* capsule) { return capsule->nlp_solver_plan; }
