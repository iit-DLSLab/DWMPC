#include "controllers/dwmpc/acadosInterface.hpp"

/**
 * Constructor for the acadosInterface class.

 */
acadosInterface::acadosInterface() : front(&acadosInterface::solvefront,this), back(&acadosInterface::solveback,this)
{   // this is made for the quadruped robot defined in front and back
    // TO DO make it general (hard because acados calls the function with the name of the problem)
    front_capsule_ = front_acados_create_capsule();
    back_capsule_ = back_acados_create_capsule();

    int status_front = front_acados_create(front_capsule_);
    int status_back = back_acados_create(back_capsule_);

    nlp_config["front"] = *front_acados_get_nlp_config(front_capsule_);
    nlp_dims["front"] = *front_acados_get_nlp_dims(front_capsule_);
    nlp_in["front"] = *front_acados_get_nlp_in(front_capsule_);
    nlp_out["front"] = *front_acados_get_nlp_out(front_capsule_);
    nlp_out_sens["front"] = *front_acados_get_nlp_out(front_capsule_);
    nlp_solver["front"] = *front_acados_get_nlp_solver(front_capsule_);
    nlp_opts_front = front_acados_get_nlp_opts(front_capsule_);

    nlp_config["back"] = *back_acados_get_nlp_config(back_capsule_);
    nlp_dims["back"] = *back_acados_get_nlp_dims(back_capsule_);
    nlp_in["back"] = *back_acados_get_nlp_in(back_capsule_);
    nlp_out["back"] = *back_acados_get_nlp_out(back_capsule_);
    nlp_out_sens["back"] = *back_acados_get_nlp_out(back_capsule_);
    nlp_solver["back"] = *back_acados_get_nlp_solver(back_capsule_);
    nlp_opts_back = back_acados_get_nlp_opts(back_capsule_);

    status_["front"] = 1;
    status_["back"] = 1;

    //TODo make the following a map
    nx_ = *nlp_dims["front"].nx;
    nu_ = *nlp_dims["front"].nu;

}
acadosInterface::~acadosInterface()
{
    run = false;
    start_front.store(true);
    start_back.store(true);
    start_front.notify_one();
    start_back.notify_one();
    front.join();
    back.join();
    front_acados_free_capsule(front_capsule_);
    back_acados_free_capsule(back_capsule_);
}
void acadosInterface::init(const int N)
{
    N_ = N;
    // int n_qp_solver_warm_start = 2;
    // ocp_nlp_solver_opts_set(&nlp_config["front"], nlp_opts_front, "qp_warm_start",&n_qp_solver_warm_start);
    // ocp_nlp_solver_opts_set(&nlp_config["back"], nlp_opts_back, "qp_warm_start",&n_qp_solver_warm_start);
    // char* hpipm_mode = "speed_abs";
    // ocp_nlp_solver_opts_set(&nlp_config["front"], nlp_opts_front, "qp_hpipm_mode", hpipm_mode);
    // ocp_nlp_solver_opts_set(&nlp_config["back"], nlp_opts_back, "qp_hpipm_mode", hpipm_mode);

}
/**
 * Set the initial condition for the optimization problem.
 *
 * @param initial_condition The initial condition vector.
 * @param problem The name of the problem ("front" or "back").
 */
void acadosInterface::setInitialCondition(const std::vector<double> &initial_condition,const std::string &problem)
{
    int idxbx0[nx_]; // Array to store the indices of the box constraints
    double lbx0[nx_]; // Array to store the lower bounds of the box constraints
    double ubx0[nx_]; // Array to store the upper bounds of the box constraints

    // Set the indices, lower bounds, and upper bounds for the box constraints
    for (int counter{0};counter<nx_;counter++)
    {
        idxbx0[counter] = counter;
        lbx0[counter] = initial_condition[counter];
        ubx0[counter] = initial_condition[counter];
    }


    // Set the initial condition constraints for the optimization problem
    ocp_nlp_constraints_model_set(&nlp_config[problem], &nlp_dims[problem], &nlp_in[problem], 0, "idxbx", idxbx0);
    ocp_nlp_constraints_model_set(&nlp_config[problem], &nlp_dims[problem], &nlp_in[problem], 0, "lbx", lbx0);
    ocp_nlp_constraints_model_set(&nlp_config[problem], &nlp_dims[problem], &nlp_in[problem], 0, "ubx", ubx0);
}
/**
 * Set the parameter values for the optimization problem.
 *
 * @param parameter The parameter values for each time step size [np X N].
 * @param np The number of parameters per stage.
 * @param problem The name of the problem ("front" or "back").
 */
void acadosInterface::setParameter(const std::vector<std::vector<double>> &parameter, const int np, const std::string &problem)
{
    // This is written for the case of the quadruped robot defined in front and back
    // generalized version needs to be written need a genral version of *_acados_update_params
    double p[np]; // Array to store the parameter values

    if (problem == "front")
    {
        for (int k = 0; k <= N_; k++)
        {
            std::copy(parameter[k].begin(), parameter[k].end(), p);
            front_acados_update_params(front_capsule_, k, p, np); // Update the parameter values for the front problem
        }
    }
    else if (problem == "back")
    {
        for (int k = 0; k <= N_; k++)
        {
            std::copy(parameter[k].begin(), parameter[k].end(), p);
            back_acados_update_params(back_capsule_, k, p, np); // Update the parameter values for the back problem
        }
    }
}
void acadosInterface::setConstraints(const std::vector<std::vector<double>> &constraint, const int nlh0, const int nlh, const std::string &problem)
{
    double lh0[nlh0];
    double lh[nlh]; 
    
    for (int k = 0; k <= N_; k++)
    { 
        if(k == 0)
        {   
            std::copy(constraint[k].begin(), constraint[k].end(), lh0);
            ocp_nlp_constraints_model_set(&nlp_config[problem],
            &nlp_dims[problem],&nlp_in[problem],k,"lh", lh0);
        }
        else
        {   
            std::copy(constraint[k].begin(), constraint[k].end(), lh);
            ocp_nlp_constraints_model_set(&nlp_config[problem],
            &nlp_dims[problem],&nlp_in[problem],k,"lh", lh);
        }
    }
}
/**
 * Set the weight matrix for the cost function of the optimization problem.
 *
 * @param weight The weight matrix for each time step size [nx+nu X nx+nu] X N [nx X nx] X 1 for the last stage.
 * @param problem The name of the problem ("front" or "back").
 */
void acadosInterface::setWeight(const std::vector<std::vector<double>> &weight,const int &n_w0,const int &n_w, const int &n_we,const std::string &problem)
{

    double W_0[n_w0*n_w0];

    memset(W_0, 0.0, sizeof(W_0));
        for (int r = 0; r < n_w0; r++)
        {
            W_0[r+r*(n_w0)] = weight[0][r];
        }
    ocp_nlp_cost_model_set( &nlp_config[problem], &nlp_dims[problem], &nlp_in[problem], 0, "W", W_0);

    double W[n_w*n_w];

    memset(W, 0.0, sizeof(W));

    for(int k{1}; k < N_;k++)
    {
        for (int r = 0; r < n_w ; r++)
        {
            W[r+r*(n_w)] = weight[k][r];
        }
        ocp_nlp_cost_model_set( &nlp_config[problem], &nlp_dims[problem], &nlp_in[problem], k, "W", W);
    }
    double W_e[n_we*n_we];
    memset(W_e, 0.0, sizeof(W_e));
        for (int r = 0; r < n_we; r++)
        {
            W_e[r+r*(n_we)] = weight[N_][r];
        }
    ocp_nlp_cost_model_set( &nlp_config[problem], &nlp_dims[problem], &nlp_in[problem], N_, "W", W_e);
}

/**
 * Set the reference values for the optimization problem.
 *
 * @param reference The reference values for each time step size [nx_ref+nu_ref X N] and [nx_ref X 1] for the last stage.
 * @param nx__ref The number of states related element in the reference vector.
 * @param nu__ref The number of inputs related element in the reference vector.
 * @param problem The name of the problem ("front" or "back").
 */
void acadosInterface::setReference(const std::vector<std::vector<double>> &reference, const int n_ref0, const int n_ref, const int n_ref_e, const std::string &problem)
{   
    double y_ref0[n_ref0];
    std::copy(reference[0].begin(), reference[0].end(), y_ref0);
    ocp_nlp_cost_model_set(&nlp_config[problem], &nlp_dims[problem], &nlp_in[problem], 0, "yref", y_ref0);

    // The reference is a vector of vectors of size (nx_ref+nu_ref) X N and nx_ref X 1 for the last stage
    double y_ref[n_ref]; // Array to store the reference values
    // Set the reference for each stage of the optimization problem
    for (int k = 1; k < N_; k++)
    {
        // Copy the reference values to the y_ref array
        std::copy(reference[k].begin(), reference[k].end(), y_ref);
        // Set the reference for the current stage of the optimization problem
        ocp_nlp_cost_model_set(&nlp_config[problem], &nlp_dims[problem], &nlp_in[problem], k, "yref", y_ref);
    }
    // Set the reference for the last stage of the optimization problem
    double y_ref_e[n_ref_e];
    std::copy(reference[N_].begin(), reference[N_].end(), y_ref_e);
    ocp_nlp_cost_model_set(&nlp_config[problem], &nlp_dims[problem], &nlp_in[problem], N_, "yref", y_ref_e);
}


/**
 * Solve the optimization problem for the quadruped robot defined in front and back.
 *
 * This function sets the RTI phase to 2, configures the solver options for both "front" and "back" problems,
 * and solves them concurrently using separate threads.
 *
 * Note: The generalized version of this function needs to be implemented to iterate over multiple problems
 * and call the general solve function.
 */
void acadosInterface::solve()
{
    int rti_phase = 2; // Set the RTI phase to 2

    // Configure solver options for "front" problem
    ocp_nlp_solver_opts_set(&nlp_config["front"], nlp_opts_front, "rti_phase", &rti_phase);

    // Configure solver options for "back" problem
    ocp_nlp_solver_opts_set(&nlp_config["back"], nlp_opts_back, "rti_phase", &rti_phase);

    // Solve "front" problem in a separate thread
    start_front.store(true);
    // Solve "back" problem in a separate thread
    start_back.store(true);

    start_front.notify_one();
    start_back.notify_one();
    // // wait work of threads
    waitWorkers();
    for(auto leg : {"front","back"})
    {   
        double elapsed_time;
        int iter_qp;
        ocp_nlp_get(&nlp_config[leg],&nlp_solver[leg], "time_tot", &elapsed_time);
        ocp_nlp_get(&nlp_config[leg],&nlp_solver[leg], "qp_iter", &iter_qp);
        
        // if(elapsed_time > 0.006)
        // {
        //     std::cout << "time for solution of " << leg << " = " << elapsed_time << std::endl;
        //     std::cout << "iter qp for solution of " << leg << " = " << iter_qp << std::endl;
        // }
    }

}
/**
 * Prepare the optimization problem for solving.
 *
 * This function sets the RTI phase to 1, configures the solver options for both "front" and "back" problems,
 * and solves them concurrently using separate threads.
 *
 * Note: The generalized version of this function needs to be implemented to iterate over multiple problems
 * and call the general solve function.
 */
void acadosInterface::prepare()
{
    int rti_phase = 1; // Set the RTI phase to 1

    // Configure solver options for "front" problem
    ocp_nlp_solver_opts_set(&nlp_config["front"], nlp_opts_front, "rti_phase", &rti_phase);

    // Configure solver options for "back" problem
    ocp_nlp_solver_opts_set(&nlp_config["back"], nlp_opts_back, "rti_phase", &rti_phase);

     // Solve "front" problem in a separate thread
    start_front.store(true);
    // Solve "back" problem in a separate thread
    start_back.store(true);

    start_front.notify_one();
    start_back.notify_one();
    // wait work of threads
    waitWorkers();
    // TODO: Implement iteration over multiple problems and call general solve function
}
void acadosInterface::solvefront()
{
    while(run)
    {
        start_front.wait(false);
        if(!run)
            break;
        status_["front"] = front_acados_solve(front_capsule_);
        start_front.store(false);
    }
}
void acadosInterface::solveback()
{
    while(run)
    {
        start_back.wait(false);
        if(!run)
            break;
        status_["back"] = back_acados_solve(back_capsule_);
        start_back.store(false);
    }
}

/**
 * Update the prediction vectors for states and inputs.
 *
 * This function retrieves the predicted states and inputs from the solver output
 * and updates the provided vectors accordingly.
 *
 * @param x The vector of predicted states [nx X N+1].
 * @param u The vector of predicted inputs [nu X N].
 * @param problem The name of the problem ("front" or "back").
 */
void acadosInterface::UpdatePrediction(std::vector<std::vector<double>> &x, std::vector<std::vector<double>> &u, const std::string &problem)
{
    double xtraj[nx_]; // Temporary array to store the predicted states
    double utraj[nu_]; // Temporary array to store the predicted inputs

    std::vector<double> xtraj_vec(nx_);
    std::vector<double> utraj_vec(nu_);

    for (int k{0}; k < N_; k++)
    {
        // Get the predicted state at time step k
        ocp_nlp_out_get(&nlp_config[problem], &nlp_dims[problem], &nlp_out[problem], k, "x", &xtraj);
        xtraj_vec.assign(xtraj, xtraj + nx_); // Update the state vector at time step k
        x.push_back(xtraj_vec);
        // Get the predicted input at time step k
        ocp_nlp_out_get(&nlp_config[problem], &nlp_dims[problem], &nlp_out[problem], k, "u", &utraj);
        utraj_vec.assign(utraj, utraj + nu_); // Update the input vector at time step k
        u.push_back(utraj_vec);
    }

    // Get the predicted state at the last time step N
    ocp_nlp_out_get(&nlp_config[problem], &nlp_dims[problem], &nlp_out[problem], N_, "x", &xtraj);
    xtraj_vec.assign(xtraj, xtraj + nx_); // Update the state vector at the last time step
    x.push_back(xtraj_vec);

}
void acadosInterface::warmstart(const std::string &problem)
{
    double xtraj[nx_]; // Temporary array to store the predicted states
    double utraj[nu_]; // Temporary array to store the predicted inputs

    for (int k{0}; k < N_; k++)
    {
        // Get the predicted state at time step k
        ocp_nlp_out_get(&nlp_config[problem], &nlp_dims[problem], &nlp_out[problem], k, "x", &xtraj);
        ocp_nlp_out_set(&nlp_config[problem], &nlp_dims[problem], &nlp_out[problem], k, "x", &xtraj);
        // Get the predicted input at time step k
        ocp_nlp_out_get(&nlp_config[problem], &nlp_dims[problem], &nlp_out[problem], k, "u", &utraj);
        ocp_nlp_out_set(&nlp_config[problem], &nlp_dims[problem], &nlp_out[problem], k, "u", &utraj);
    }

    // Get the predicted state at the last time step N
    ocp_nlp_out_get(&nlp_config[problem], &nlp_dims[problem], &nlp_out[problem], N_, "x", &xtraj);
    ocp_nlp_out_set(&nlp_config[problem], &nlp_dims[problem], &nlp_out[problem], N_, "x", &xtraj);

}
void acadosInterface::waitWorkers(){
    // it is implemented as a spin lock (not efficient but unblocking)
    while(start_front.load() || start_back.load()){}
}
