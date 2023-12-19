#include <dwocp.hpp>

dwocp::dwocp(int N, double dt)
{
    N_ = N;
    dt_ = dt;

    FL_capsule_ = FL_acados_create_capsule();
    FR_capsule_ = FR_acados_create_capsule();
    RL_capsule_ = RL_acados_create_capsule();
    RR_capsule_ = RR_acados_create_capsule();

    int status_FL = FL_acados_create(FL_capsule_);      
    int status_FR = FR_acados_create(FR_capsule_);
    int status_RL = RL_acados_create(RL_capsule_);
    int status_RR = RR_acados_create(RR_capsule_);

    nlp_config["FL"] = *FL_acados_get_nlp_config(FL_capsule_);
    nlp_dims["FL"] = *FL_acados_get_nlp_dims(FL_capsule_);
    nlp_in["FL"] = *FL_acados_get_nlp_in(FL_capsule_);
    nlp_out["FL"] = *FL_acados_get_nlp_out(FL_capsule_);
    nlp_solver["FL"] = *FL_acados_get_nlp_solver(FL_capsule_);
    nlp_opts_FL = FL_acados_get_nlp_opts(FL_capsule_);

    nlp_config["FR"] = *FR_acados_get_nlp_config(FR_capsule_);
    nlp_dims["FR"] = *FR_acados_get_nlp_dims(FR_capsule_);
    nlp_in["FR"] = *FR_acados_get_nlp_in(FR_capsule_);
    nlp_out["FR"] = *FR_acados_get_nlp_out(FR_capsule_);
    nlp_solver["FR"] = *FR_acados_get_nlp_solver(FR_capsule_);
    nlp_opts_FR = FR_acados_get_nlp_opts(FR_capsule_);

    nlp_config["RL"] = *RL_acados_get_nlp_config(RL_capsule_);
    nlp_dims["RL"] = *RL_acados_get_nlp_dims(RL_capsule_);
    nlp_in["RL"] = *RL_acados_get_nlp_in(RL_capsule_);
    nlp_out["RL"] = *RL_acados_get_nlp_out(RL_capsule_);
    nlp_solver["RL"] = *RL_acados_get_nlp_solver(RL_capsule_);
    nlp_opts_RL = RL_acados_get_nlp_opts(RL_capsule_);

    nlp_config["RR"] = *RR_acados_get_nlp_config(RR_capsule_);
    nlp_dims["RR"] = *RR_acados_get_nlp_dims(RR_capsule_);
    nlp_in["RR"] = *RR_acados_get_nlp_in(RR_capsule_);
    nlp_out["RR"] = *RR_acados_get_nlp_out(RR_capsule_);
    nlp_solver["RR"] = *RR_acados_get_nlp_solver(RR_capsule_);
    nlp_opts_RR = RR_acados_get_nlp_opts(RR_capsule_);

    status_["FL"] = 1;
    status_["FR"] = 1;
    status_["RL"] = 1;
    status_["RR"] = 1;

    //TODo make the following a map
    nx = *nlp_dims["FL"].nx;
    nu = *nlp_dims["FL"].nu;

}
// dwocp::~dwocp()
// {   
//     int status {};
//     status = FL_acados_free(FL_capsule_);
//     status = FR_acados_free(FR_capsule_);
//     status = RL_acados_free(RL_capsule_);
//     status = RR_acados_free(RR_capsule_);
//     status = FL_acados_free_capsule(FL_capsule_);
//     status = FR_acados_free_capsule(FR_capsule_);
//     status = RL_acados_free_capsule(RL_capsule_);
//     status = RR_acados_free_capsule(RR_capsule_);
// }
void dwocp::setInitialCondition(std::vector<double> &initial_condition)
{
    
    int idxbx0[nx];
    double lbx0[nx];
    double ubx0[nx];
    
    int joint_per_leg{3};
    int n_leg{4};
    
    for (int counter{0};counter<nx;counter++)
    {
        idxbx0[counter] = counter;
    }
    //fill the first 7 elements [p,quat] are the same for all sub-problem
    for (int counter{0};counter < 7; counter ++)
    {
        lbx0[counter] = initial_condition[counter];
        ubx0[counter] = initial_condition[counter];
    }
    //fill the first 11th to 16 elements [dp,omega] are the same for all sub-problem
    for (int counter{7 + joint_per_leg};counter < 13 + joint_per_leg;counter++)
    {
        lbx0[counter] = initial_condition[counter+joint_per_leg*(n_leg-1)]; //shift the initial condition to not consider 9 joints of the whole-body problem
        ubx0[counter] = initial_condition[counter+joint_per_leg*(n_leg-1)];
    }

    int leg_number{0};
    
    for(auto leg : legs)
    {   
        //set joint initial condition for the subproblems
        for(int counter{0};counter<3;counter++)
        {
            //q we are considering 3 joints per leg
            lbx0[counter+7] = initial_condition[7 + counter + leg_number*joint_per_leg];
            ubx0[counter+7] = initial_condition[7 + counter + leg_number*joint_per_leg];
             //dq
            lbx0[counter+16] = initial_condition[7 + joint_per_leg*n_leg + 6 + counter + leg_number*joint_per_leg];
            ubx0[counter+16] = initial_condition[7 + joint_per_leg*n_leg + 6 + counter + leg_number*joint_per_leg];
            //tau
            lbx0[counter+16] = initial_condition[7 + 2+joint_per_leg*n_leg + 6 + counter + leg_number*joint_per_leg];
            ubx0[counter+16] = initial_condition[7 + 2+joint_per_leg*n_leg + 6 + counter + leg_number*joint_per_leg];

        }

        ocp_nlp_constraints_model_set(&nlp_config[leg], &nlp_dims[leg], &nlp_in[leg], 0, "idxbx", idxbx0);
        ocp_nlp_constraints_model_set(&nlp_config[leg], &nlp_dims[leg], &nlp_in[leg], 0, "lbx", lbx0);
        ocp_nlp_constraints_model_set(&nlp_config[leg], &nlp_dims[leg], &nlp_in[leg], 0, "ubx", ubx0);

        leg_number++;
    }
}
void dwocp::setParameter(std::vector<std::vector<double>> &parameter,int np)
{
    double p[np];

    for (int ii = 0; ii <= N_; ii++)
    {
        int kk {0};
        for (auto param : parameter[ii])
        {
            p[kk] = param;
            kk++;
        }

        FL_acados_update_params(FL_capsule_, ii, p, np);

        FR_acados_update_params(FR_capsule_, ii, p, np);
    
        RR_acados_update_params(RR_capsule_, ii, p, np);
    
        RL_acados_update_params(RL_capsule_, ii, p, np);
    }
}
void dwocp::setWeight(std::vector<double> &weight)
{   
    double W[(nx+nu)*(nx+nu)]; 
    for (int r = 0; r < (nx+nu); r++)
    {   
        for (int c = 0; c < (nx+nu); c++)
        {   
            if (r == c)
            {
                W[r+c*(nx+nu)] = weight[r];
            }
            else
            {
                W[r+c*(nx+nu)] = 0;
            }
        }
        
    }
    double W_e[nx*nx]; 
    for (int r = 0; r < (nx); r++)
    {   
       for (int c = 0; c < (nx); c++)
        {   
            if (r == c)
            {
                W[r+c*(nx)] = weight[r];
            }
            else
            {
                W[r+c*(nx)] = 0;
            }
        }
        
    }
    for (auto leg : legs)
    {
        for(int k; k < N_;k++)
        {
            ocp_nlp_cost_model_set( &nlp_config[leg], &nlp_dims[leg], &nlp_in[leg], k, "W", W);
        }

        ocp_nlp_cost_model_set( &nlp_config[leg], &nlp_dims[leg], &nlp_in[leg], N_, "W", W_e);
    }
}
void dwocp::setReference(std::vector<std::vector<double>> &reference,int nx_ref,int nu_ref)
{   
    // refrence order : p,q,dp,omega,dq,tau,foot,dtau,grf
    double y_ref[nx_ref+nu_ref];
    int joint_per_leg{3};
    int n_leg{4};

    for (int ii = 0; ii <= N_; ii++)
    {   
        //p
        y_ref[0] = reference[ii][0];
        y_ref[1] = reference[ii][1];
        y_ref[2] = reference[ii][2];

        //dp 
        y_ref[joint_per_leg+3] = reference[ii][3+joint_per_leg*n_leg];
        y_ref[joint_per_leg+4] = reference[ii][4+joint_per_leg*n_leg];
        y_ref[joint_per_leg+5] = reference[ii][5+joint_per_leg*n_leg];

        //omega
        y_ref[joint_per_leg+6] = reference[ii][6+joint_per_leg*n_leg];
        y_ref[joint_per_leg+7] = reference[ii][7+joint_per_leg*n_leg];
        y_ref[joint_per_leg+8] = reference[ii][8+joint_per_leg*n_leg];
        int counter{0};
        for(auto leg : legs)
        {   
            for(int idx{0};idx<joint_per_leg;idx++)
            {
                //q
                y_ref[3+idx] = reference[ii][3+joint_per_leg*counter+idx];
                //dq
                y_ref[9+joint_per_leg+idx] = reference[ii][9+joint_per_leg*n_leg +joint_per_leg*counter+idx];
                //tau
                y_ref[9 + 2*joint_per_leg + idx] = reference[ii][9+2*joint_per_leg*n_leg+joint_per_leg*counter+idx];
                //dtau
                y_ref[12+3*joint_per_leg+idx] = reference[ii][9+3*joint_per_leg*n_leg+joint_per_leg*counter+3*n_leg+idx];
            }
            
            for (int idx{0};idx<3;idx++)
            {
                //grf
                y_ref[12+4*joint_per_leg+idx] = reference[ii][9+4*joint_per_leg*n_leg+3*n_leg+idx];
                // foot
                y_ref[9 + 3*joint_per_leg + idx] = reference[ii][9+3*joint_per_leg*n_leg+3*counter+idx];

            }
            // for (int i = 0; i < nx_ref + nu_ref; i++) {
            //     std::cout << ii << " y_ref[" << i << "] = " << y_ref[i] << std::endl;
            // }

            ocp_nlp_cost_model_set(&nlp_config[leg], &nlp_dims[leg], &nlp_in[leg], ii, "yref", y_ref);
            counter ++;
        }
    }
    double y_ref_e[nx_ref];
    //p
    y_ref_e[0] = reference[N_][0];
    y_ref_e[1] = reference[N_][1];
    y_ref_e[2] = reference[N_][2];
    //dp 
    y_ref_e[joint_per_leg+3] = reference[N_][3+joint_per_leg*n_leg];
    y_ref_e[joint_per_leg+4] = reference[N_][4+joint_per_leg*n_leg];
    y_ref_e[joint_per_leg+5] = reference[N_][5+joint_per_leg*n_leg];
    //omega
    y_ref_e[joint_per_leg+6] = reference[N_][6+joint_per_leg*n_leg];
    y_ref_e[joint_per_leg+7] = reference[N_][7+joint_per_leg*n_leg];
    y_ref_e[joint_per_leg+8] = reference[N_][8+joint_per_leg*n_leg];

    int counter{0};
    for(auto leg : legs)
    {   
        for(int idx{0};idx<joint_per_leg;idx++)
        {
            //q
            y_ref_e[3+idx] = reference[N_][3+joint_per_leg*counter+idx];
            //dq
            y_ref_e[9+joint_per_leg+idx] = reference[N_][9+joint_per_leg*n_leg +joint_per_leg*counter+idx];
            //tau
            y_ref_e[9+2*joint_per_leg+idx] = reference[N_][9+2*joint_per_leg*n_leg+joint_per_leg*counter+idx];

        }
        for (int idx{0};idx<3;idx++)
            {
                // foot
                y_ref_e[9 + 3*joint_per_leg + idx] = reference[N_][9+3*joint_per_leg*n_leg+3*counter+idx];

            }
        // for (int i = 0; i < nx_ref ; i++) {
        //         std::cout << "y_ref[" << i << "] = " << y_ref_e[i] << std::endl;
        //     }
        ocp_nlp_cost_model_set(&nlp_config[leg], &nlp_dims[leg], &nlp_in[leg], N_, "yref", y_ref_e);
        counter ++;
    }
}
void dwocp::setWarmStart(std::vector<std::vector<double>> &x_start,std::vector<std::vector<double>> &u_start)
{
    // initialize solution
    double x_ws[nx];
    double u_ws[nu];
    int joint_per_leg{3};
    int n_leg{4};

    for (int i = 0; i < N_; i++)
    {   
        int counter{0};
        for(int ii{0};ii<7;ii++)
        {
            x_ws[ii] = x_start[i][ii]; // p , quat
        }
        for(int ii{0};ii<6;ii++)
        {
            x_ws[ii+7+joint_per_leg] = x_start[i][ii+7+joint_per_leg*n_leg]; // dp , omega
        }
        for(auto leg : legs)
        {   
            for(int ii{0};ii<joint_per_leg;ii++)
            {
                x_ws[ii+7] = x_start[i][ii+7+joint_per_leg*counter]; //q
                x_ws[ii+13+joint_per_leg] = x_start[i][ii+13+joint_per_leg*n_leg+joint_per_leg*counter]; // dq
                x_ws[ii+13+2*joint_per_leg] = x_start[i][ii+13+2*joint_per_leg*n_leg+joint_per_leg*counter]; //tau
                u_ws[ii] = u_start[i][ii+joint_per_leg*counter]; //dtau
            }
            //grf
            for(int ii{0};ii<3*n_leg;ii++)
            {
                u_ws[ii+joint_per_leg] = u_start[i][ii+joint_per_leg*n_leg];
            }
            ocp_nlp_out_set(&nlp_config[leg], &nlp_dims[leg], &nlp_out[leg], i, "x", x_ws);
            ocp_nlp_out_set(&nlp_config[leg], &nlp_dims[leg], &nlp_out[leg], i, "u", u_ws);
            counter ++;
        }
    }
    //last node only x
    int counter{0};
    for(int ii{0};ii<7;ii++)
    {
        x_ws[ii] = x_start[N_][ii]; // p , quat
    }
    for(int ii{0};ii<6;ii++)
    {
        x_ws[ii+7+joint_per_leg] = x_start[N_][ii+7+joint_per_leg*n_leg]; // dp , omega
    }
    for(auto leg : legs)
    {   
        for(int ii{0};ii<joint_per_leg;ii++)
        {
            x_ws[ii+7] = x_start[N_][ii+7+joint_per_leg*counter]; //q
            x_ws[ii+13+joint_per_leg] = x_start[N_][ii+13+joint_per_leg*n_leg+joint_per_leg*counter]; // dq
            x_ws[ii+13+2*joint_per_leg] = x_start[N_][ii+13+2*joint_per_leg*n_leg+joint_per_leg*counter]; //tau
        }
        ocp_nlp_out_set(&nlp_config[leg], &nlp_dims[leg], &nlp_out[leg], N_, "x", x_ws);
        counter ++;
    }
    
}
void dwocp::solve()
{
    int rti_phase = 2;

    ocp_nlp_solver_opts_set(&nlp_config["FL"], nlp_opts_FL, "rti_phase", &rti_phase);
    ocp_nlp_solver_opts_set(&nlp_config["FR"], nlp_opts_FR, "rti_phase", &rti_phase);
    ocp_nlp_solver_opts_set(&nlp_config["RR"], nlp_opts_RR, "rti_phase", &rti_phase);
    ocp_nlp_solver_opts_set(&nlp_config["RL"], nlp_opts_RL, "rti_phase", &rti_phase);

    std::thread FL_thead(&dwocp::solveFL,this);
    std::thread FR_thead(&dwocp::solveFR,this);
    std::thread RL_thead(&dwocp::solveRL,this);
    std::thread RR_thead(&dwocp::solveRR,this);

    FL_thead.join();
    FR_thead.join();
    RL_thead.join();
    RR_thead.join();
    // for(auto leg : legs)
    // {   
    //     double elapsed_time;
    //     ocp_nlp_get(&nlp_config[leg],&nlp_solver[leg], "time_tot", &elapsed_time);
    //     std::cout << "time for solution of " << leg << " = " << elapsed_time << std::endl;
    // }

}
void dwocp::prepare()
{
    int rti_phase = 1;

    ocp_nlp_solver_opts_set(&nlp_config["FL"], nlp_opts_FL, "rti_phase", &rti_phase);
    ocp_nlp_solver_opts_set(&nlp_config["FR"], nlp_opts_FR, "rti_phase", &rti_phase);
    ocp_nlp_solver_opts_set(&nlp_config["RR"], nlp_opts_RR, "rti_phase", &rti_phase);
    ocp_nlp_solver_opts_set(&nlp_config["RL"], nlp_opts_RL, "rti_phase", &rti_phase);

    std::thread FL_thead(&dwocp::solveFL,this);
    std::thread FR_thead(&dwocp::solveFR,this);
    std::thread RL_thead(&dwocp::solveRL,this);
    std::thread RR_thead(&dwocp::solveRR,this);

    FL_thead.join();
    FR_thead.join();
    RL_thead.join();
    RR_thead.join();

}
void dwocp::solveFL()
{
    status_["FL"] = FL_acados_solve(FL_capsule_);
}
void dwocp::solveFR()
{
    status_["FR"] = FR_acados_solve(FR_capsule_);
}
void dwocp::solveRL()
{
    status_["RL"] = RL_acados_solve(RL_capsule_);
}
void dwocp::solveRR()
{
    status_["RR"] = RR_acados_solve(RR_capsule_);
}
void dwocp::getControl(std::vector<std::vector<double>> &torque, std::vector<std::vector<double>> &q, std::vector<std::vector<double>> &dq,std::vector<std::vector<double>> &grf)
{   
    double xtraj[nx];
    double utraj[nu];
    int joint_per_leg{3};
    int n_leg{4};

    int counter{0};
    for (auto leg : legs)
    {   
        for (int k{0};k<N_;k++)
        {
            ocp_nlp_out_get(&nlp_config[leg], &nlp_dims[leg], &nlp_out[leg], 1, "x", &xtraj);
            for(int ii{0};ii<joint_per_leg;ii++)
            {
                torque[k][ii + counter * joint_per_leg] = xtraj[7+joint_per_leg+6+joint_per_leg+ii];
                q[k][ii + counter * joint_per_leg] = xtraj[7+ii];
                dq[k][ii + counter * joint_per_leg] = xtraj[7+joint_per_leg+6+ii];
            }
            ocp_nlp_out_get(&nlp_config[leg], &nlp_dims[leg], &nlp_out[leg], 1, "u", &utraj);
            grf[k][3*counter + 0] = utraj[joint_per_leg + 3*counter + k*(3*n_leg+joint_per_leg) + 0];
            grf[k][3*counter + 1] = utraj[joint_per_leg + 3*counter + k*(3*n_leg+joint_per_leg) + 1];
            grf[k][3*counter + 2] = utraj[joint_per_leg + 3*counter + k*(3*n_leg+joint_per_leg) + 2];
        }
        counter ++;
    }    
}