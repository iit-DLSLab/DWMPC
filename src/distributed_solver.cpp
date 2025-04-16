#include "controllers/dwmpc/distributed_solver.hpp"

dsolver::dsolver() : acados_interface_()
{}
void dsolver::init(const parameter &solver_param)
{
    solver_param_ = solver_param;
    acados_interface_.init(solver_param_.N_);
    //init to the dimension of data_ based on solver_param_
    for(auto name : solver_param_.subsystems_name)
    {
        pdata subsystem_data{};
        data_[name] = subsystem_data; 
    }

}
void dsolver::solve( bool &do_init,
                    const std::map<std::string,std::vector<double>> inital_condition,
                    const std::map<std::string,std::vector<std::vector<double>>> &ref,
                    const std::map<std::string,std::vector<std::vector<double>>> &param,
                    const std::map<std::string,std::vector<double>> &weight_vec) 
{   
    // init data to the reference 
    int max_iterations{1}; // maximum number of iteration for the distributed solver
    if (do_init)
    {    
        for(auto name : solver_param_.subsystems_name)
        {   
            // init to refernce
            data_[name].p = ref.at("p");
            data_[name].quat = ref.at("quat");
            data_[name].dp = ref.at("dp");
            data_[name].omega = ref.at("omega");
            for (int k{0};k<solver_param_.N_+1;k++)
            {   
                std::vector<double> q,dq,tau,grf;
                for (auto idx : solver_param_.subsystems_map_joint[name])
                {
                    q.push_back(ref.at("q")[k][idx]);
                    dq.push_back(ref.at("dq")[k][idx]);
                    tau.push_back(ref.at("tau")[k][idx]);
                }
                for (auto idx : solver_param_.subsystems_map_contact[name])
                {
                    grf.push_back(ref.at("grf")[k][3*idx]);
                    grf.push_back(ref.at("grf")[k][3*idx+1]);
                    grf.push_back(ref.at("grf")[k][3*idx+2]);
                }
                data_[name].tau.push_back(tau);
                data_[name].grf.push_back(grf);
                data_[name].q.push_back(q);
                data_[name].dq.push_back(dq);
                data_[name].dual.push_back(std::vector<double>(6,0));
                data_[name].residual.push_back(std::vector<double>(6,0));
            }
        }
        max_iterations = solver_param_.max_iteration;
        // do_init = false;
    } 
    else if (solver_param_.receding_horizon)
    {
        // shift past prediction for receding horizon
        for(auto problem : solver_param_.subsystems_name)
        {
            for (int k{0};k<solver_param_.N_;k++)
            {
                data_[problem].p[k] = data_[problem].p[k+1];
                data_[problem].quat[k] = data_[problem].quat[k+1];
                data_[problem].q[k] = data_[problem].q[k+1];
                data_[problem].dp[k] = data_[problem].dp[k+1];
                data_[problem].omega[k] = data_[problem].omega[k+1];
                data_[problem].dq[k] = data_[problem].dq[k+1];
                if (k < solver_param_.N_ - 1)
                {
                    data_[problem].tau[k] = data_[problem].tau[k+1];
                    data_[problem].grf[k] = data_[problem].grf[k+1];
                }
                
                data_[problem].dual[k] = data_[problem].dual[k+1];
            }
            data_[problem].p[solver_param_.N_] = std::vector<double>(3,0);
            data_[problem].quat[solver_param_.N_] = std::vector<double>(4,0);
            data_[problem].q[solver_param_.N_] = std::vector<double>(solver_param_.subsystems_map_joint[problem].size(),0);
            data_[problem].dp[solver_param_.N_] = std::vector<double>(3,0);
            data_[problem].omega[solver_param_.N_] = std::vector<double>(3,0);
            data_[problem].dq[solver_param_.N_] = std::vector<double>(solver_param_.subsystems_map_joint[problem].size(),0);
            data_[problem].tau[solver_param_.N_-1] = std::vector<double>(solver_param_.subsystems_map_joint[problem].size(),0);
            data_[problem].grf[solver_param_.N_-1] = std::vector<double>(solver_param_.subsystems_map_contact[problem].size()*3,0);
            data_[problem].dual[solver_param_.N_] = std::vector<double>(6,0);
        }
        max_iterations = 1;
    }

    // main loop  (number of iteration)
    for(auto i = 0; i < max_iterations;i++)
    {   
        //problem loop
        for (auto problem : solver_param_.subsystems_name)
        {   
            // if whole body problem skip
            if (problem == "wb")
                continue;

            // ============ INITAIAL CONDITION ============
            
            std::vector<double> problem_inital_condition;
            
            problem_inital_condition.push_back(inital_condition.at("p")[0]);
            problem_inital_condition.push_back(inital_condition.at("p")[1]);
            problem_inital_condition.push_back(inital_condition.at("p")[2]);

            problem_inital_condition.push_back(inital_condition.at("quat")[0]);
            problem_inital_condition.push_back(inital_condition.at("quat")[1]);
            problem_inital_condition.push_back(inital_condition.at("quat")[2]);
            problem_inital_condition.push_back(inital_condition.at("quat")[3]);

            for(auto idx : solver_param_.subsystems_map_joint[problem])
            {
                problem_inital_condition.push_back(inital_condition.at("q")[idx]);
            }

            problem_inital_condition.push_back(inital_condition.at("dp")[0]);
            problem_inital_condition.push_back(inital_condition.at("dp")[1]);
            problem_inital_condition.push_back(inital_condition.at("dp")[2]);

            problem_inital_condition.push_back(inital_condition.at("omega")[0]);
            problem_inital_condition.push_back(inital_condition.at("omega")[1]);
            problem_inital_condition.push_back(inital_condition.at("omega")[2]);

            for(auto idx : solver_param_.subsystems_map_joint[problem])
            {
                problem_inital_condition.push_back(inital_condition.at("dq")[idx]);
            }
            //std::cout << std::endl;

            std::vector<std::vector<double>> problem_ref;
            std::vector<std::vector<double>> problem_param;
            std::vector<std::vector<double>> problem_weight;
            std::vector<std::vector<double>> problem_constraints;

            // horizon loop
            for (auto k{0};k< solver_param_.N_ + 1;k++)
            {   
                ////  ============ REFERENCE  ============
                std::vector<double> ref_k;
                
                //set p,quat 
                ref_k.push_back(ref.at("p")[k][0]);
                ref_k.push_back(ref.at("p")[k][1]);
                ref_k.push_back(ref.at("p")[k][2]);

                ref_k.push_back(0);
                ref_k.push_back(0);
                ref_k.push_back(0);


                // set q
                for(auto idx : solver_param_.subsystems_map_joint[problem])
                {   
                    ref_k.push_back(ref.at("q")[k][idx]);
                }

                // set dp omega
                ref_k.push_back(ref.at("dp")[k][0]);
                ref_k.push_back(ref.at("dp")[k][1]);
                ref_k.push_back(ref.at("dp")[k][2]);

                ref_k.push_back(ref.at("omega")[k][0]);
                ref_k.push_back(ref.at("omega")[k][1]);
                ref_k.push_back(ref.at("omega")[k][2]);


                // set dq
                for(auto idx : solver_param_.subsystems_map_joint[problem])
                {
                    ref_k.push_back(ref.at("dq")[k][idx]);
                }
                // set foot
                for (auto idx : solver_param_.subsystems_map_contact[problem])
                {
                    ref_k.push_back(ref.at("foot")[k][3*idx]);
                    ref_k.push_back(ref.at("foot")[k][3*idx+1]);
                    ref_k.push_back(ref.at("foot")[k][3*idx+2]);
                }
                if (k < solver_param_.N_)
                {   
                    // set tau
                    for(auto idx : solver_param_.subsystems_map_joint[problem])
                    {
                        ref_k.push_back(ref.at("tau")[k][idx]);
                    }
                    // set grf
                    for (auto p : solver_param_.subsystems_name)
                    {   
                        if (p == "wb")
                        { 
                            continue;
                        }
                        for(auto idx : solver_param_.subsystems_map_contact[p])
                            {
                                if( p == problem)
                                {   
                                    ref_k.push_back(ref.at("grf")[k][3*idx]);
                                    ref_k.push_back(ref.at("grf")[k][3*idx+1]);
                                    ref_k.push_back(ref.at("grf")[k][3*idx+2]);                                   
                                }
                                else
                                {   
                                    ref_k.push_back(data_["wb"].grf[k][3*idx]);
                                    ref_k.push_back(data_["wb"].grf[k][3*idx+1]);
                                    ref_k.push_back(data_["wb"].grf[k][3*idx+2]);
                                    // ref_k.push_back(ref.at("grf")[k][3*idx]);
                                    // ref_k.push_back(ref.at("grf")[k][3*idx+1]);
                                    // ref_k.push_back(ref.at("grf")[k][3*idx+2]);
                                }
                            }
                    }
                    // set consensus
                    // if(problem == "front")
                    // {
                    //     ref_k.push_back(data_["back"].dp[k][0] - data_[problem].dual[k][0]);
                    //     ref_k.push_back(data_["back"].dp[k][1] - data_[problem].dual[k][1]);
                    //     ref_k.push_back(data_["back"].dp[k][2] - data_[problem].dual[k][2]);

                    //     ref_k.push_back(data_["back"].omega[k][0] - data_[problem].dual[k][3]);
                    //     ref_k.push_back(data_["back"].omega[k][1] - data_[problem].dual[k][4]);
                    //     ref_k.push_back(data_["back"].omega[k][2] - data_[problem].dual[k][5]);
                    // }
                    // else
                    // {
                    //     ref_k.push_back(-data_["front"].dp[k][0] - data_[problem].dual[k][0]);
                    //     ref_k.push_back(-data_["front"].dp[k][1] - data_[problem].dual[k][1]);
                    //     ref_k.push_back(-data_["front"].dp[k][2] - data_[problem].dual[k][2]);

                    //     ref_k.push_back(-data_["front"].omega[k][0] - data_[problem].dual[k][3]);
                    //     ref_k.push_back(-data_["front"].omega[k][1] - data_[problem].dual[k][4]);
                    //     ref_k.push_back(-data_["front"].omega[k][2] - data_[problem].dual[k][5]);
                    // }
                    ref_k.push_back(data_["wb"].dp[k][0] - data_[problem].dual[k][0]);
                    ref_k.push_back(data_["wb"].dp[k][1] - data_[problem].dual[k][1]);
                    ref_k.push_back(data_["wb"].dp[k][2] - data_[problem].dual[k][2]);

                    ref_k.push_back(data_["wb"].omega[k][0] - data_[problem].dual[k][3]);
                    ref_k.push_back(data_["wb"].omega[k][1] - data_[problem].dual[k][4]);
                    ref_k.push_back(data_["wb"].omega[k][2] - data_[problem].dual[k][5]);

                }
                if(k == 0)
                {
                    for(int idx{0};idx < solver_param_.n_ineq_0;idx++)
                    {
                        ref_k.push_back(0);
                    }
                }
                else
                {
                    for(int idx{0};idx < solver_param_.n_ineq;idx++)
                    {
                        ref_k.push_back(0);
                    }
                }
                problem_ref.push_back(ref_k);

                ////  ============ PARAMETER  ============

                std::vector<double> param_k;

                // set contact sequence 
                for(int idx{0}; idx < solver_param_.N_legs; idx++)
                {   
                    param_k.push_back(param.at("contact_seq")[k][idx]);
                }
                // set foot reference 
                for(auto idx : solver_param_.subsystems_map_contact[problem])
                {
                    param_k.push_back(param.at("foot")[k][3*idx]);
                    param_k.push_back(param.at("foot")[k][3*idx+1]);
                    param_k.push_back(param.at("foot")[k][3*idx+2]);
                }                
                // set q dq 
                for (auto idx : solver_param_.subsystems_map_joint["wb"])
                {
                    param_k.push_back(data_["wb"].q[k][idx]);
                }
                for (auto idx : solver_param_.subsystems_map_joint["wb"])
                {
                    param_k.push_back(data_["wb"].dq[k][idx]);
                }
                // set quat_ref
                param_k.push_back(ref.at("quat")[k][0]);
                param_k.push_back(ref.at("quat")[k][1]);
                param_k.push_back(ref.at("quat")[k][2]);
                param_k.push_back(ref.at("quat")[k][3]);
                
               
                // set dt 
                param_k.push_back(param.at("dt")[k][0]);
                // set grf
                for(auto idx : solver_param_.subsystems_map_contact["wb"])
                {
                    param_k.push_back(data_["wb"].grf[k][3*idx]);
                    param_k.push_back(data_["wb"].grf[k][3*idx+1]);
                    param_k.push_back(data_["wb"].grf[k][3*idx+2]);
                }
                problem_param.push_back(param_k);
                ////  ============ WEIGHT  ============
                int nw{0};
                if(k == 0)
                {
                    nw = solver_param_.nw0;
                }
                else if(k == solver_param_.N_)
                {
                    nw = solver_param_.nwe;
                }
                else
                {
                    nw = solver_param_.nw;
                }
                
                if(do_init)
                {
                    std::vector<double> weight_k(nw*nw,0);

                    // weight p 
                    weight_k[0] = weight_vec.at("p")[0];
                    weight_k[1 + nw] = weight_vec.at("p")[1];
                    weight_k[2 + 2*nw] = weight_vec.at("p")[2];

                    // weight quat
                    weight_k[3 + 3*nw] = weight_vec.at("quat")[0];
                    weight_k[4 + 4*nw] = weight_vec.at("quat")[1];
                    weight_k[5 + 5*nw] = weight_vec.at("quat")[2];

                    // weight q
                    // weight dq
                    int count{0};
                    for(auto idx : solver_param_.subsystems_map_joint[problem])
                    {
                        weight_k[6 + count + (6+count)*nw] = weight_vec.at("q")[0];
                        weight_k[12 + 6 + count + (12+count+6)*nw] = weight_vec.at("dq")[0];
                        count++;
                    }
                
                    // weight dp
                    weight_k[6 + count + (6+count)*nw] = weight_vec.at("dp")[0];
                    weight_k[7 + count + (7+count)*nw] = weight_vec.at("dp")[1];
                    weight_k[8 + count + (8+count)*nw] = weight_vec.at("dp")[2];

                    // weight omega
                    weight_k[9 + count + (9+count)*nw] = weight_vec.at("omega")[0];
                    weight_k[10 + count + (10+count)*nw] = weight_vec.at("omega")[1];
                    weight_k[11 + count + (11+count)*nw] = weight_vec.at("omega")[2];

                    count+=6;
                    // weight foot
                    for(auto idx : solver_param_.subsystems_map_contact[problem])
                    {   
                        if (param.at("contact_seq")[k][idx] == 1)
                        {
                            weight_k[12 + count + (12+count)*nw] = weight_vec.at("foot_stance")[0];
                            weight_k[13 + count + (13+count)*nw] = weight_vec.at("foot_stance")[1];
                            weight_k[14 + count + (14+count)*nw] = weight_vec.at("foot_stance")[2];
                        }
                        else
                        {
                            weight_k[12 + count + (12+count)*nw] = weight_vec.at("foot_swing")[0];
                            weight_k[13 + count + (13+count)*nw] = weight_vec.at("foot_swing")[1];
                            weight_k[14 + count + (14+count)*nw] = weight_vec.at("foot_swing")[2];
                        }
                        count+=3;
                    }
                    if (k < solver_param_.N_)
                    {
                        // weight tau
                        for(auto idx : solver_param_.subsystems_map_joint[problem])
                        {
                            weight_k[12 + count + (12+count)*nw] = weight_vec.at("tau")[0];
                            count++;
                        }
                        // weight grf
                        for(auto idx : solver_param_.subsystems_map_contact["wb"])
                        {
                            weight_k[12 + count + (12+count)*nw] = weight_vec.at("grf")[0];
                            weight_k[13 + count + (13+count)*nw] = weight_vec.at("grf")[0];
                            weight_k[14 + count + (14+count)*nw] = weight_vec.at("grf")[0];
                            count+=3;
                        }
                        // weight consensus 
                        weight_k[15 + count + (15+count)*nw] = weight_vec.at("consensus")[0];
                        weight_k[16 + count + (16+count)*nw] = weight_vec.at("consensus")[0];
                        weight_k[17 + count + (17+count)*nw] = weight_vec.at("consensus")[0];
                        weight_k[18 + count + (18+count)*nw] = weight_vec.at("consensus")[0];
                        weight_k[19 + count + (19+count)*nw] = weight_vec.at("consensus")[0];
                        weight_k[20 + count + (20+count)*nw] = weight_vec.at("consensus")[0];

                        if(k == 0)
                        {
                            for(int idx{0};idx < solver_param_.n_ineq_0;idx++)
                            {
                                weight_k[24 + count + (24+count)*nw] = 1;
                                count++;
                            }
                        }
                        else
                        {  
                            for(int idx{0};idx < solver_param_.n_ineq;idx++)
                            {
                                weight_k[24 + count+ (24+count)*nw] = 1;
                                count++;
                            }
                        }
                    }
                    problem_weight.push_back(weight_k);
                }
                // // set the constraints
                // std::vector<double> constraints_k{};
                // if (k == 0)
                // {
                    // constraints_k = solver_param_.lh0;
                // }
                // else
                // {
                    // constraints_k = solver_param_.lh;
                // }
                // for(auto leg : solver_param_.subsystems_map_contact["wb"])
                // {
                    // constraints_k[5*leg] = param.at("contact_seq")[k][leg]*5;
                // }
                // problem_constraints.push_back(constraints_k);                
            }
            // pass to the acados sovler
            acados_interface_.setParameter(problem_param,static_cast<int>(problem_param[0].size()),problem);
            acados_interface_.setReference(problem_ref,static_cast<int>(problem_ref[0].size()),static_cast<int>(problem_ref[1].size()),static_cast<int>(problem_ref[solver_param_.N_].size())-solver_param_.n_ineq,problem); 
            // auto start_Weight = std::chrono::high_resolution_clock::now();
            if(do_init)
            {
                acados_interface_.setWeight(problem_weight,solver_param_.nw0,solver_param_.nw,solver_param_.nwe,problem);
            }
            // auto stop_Weight = std::chrono::high_resolution_clock::now();
            acados_interface_.setInitialCondition(problem_inital_condition,problem);    
            // acados_interface_.setConstraints(problem_constraints,static_cast<int>(problem_constraints[0].size()),static_cast<int>(problem_constraints[1].size()),problem);
        }
        
        //solve problems
        if(do_init || !solver_param_.receding_horizon)
        {
            acados_interface_.prepare();
        }
        // acados_interface_.prepare();
        acados_interface_.solve();
        

        
        for (auto problem : solver_param_.subsystems_name)
        {   
                
            // update state from solution
            std::vector<std::vector<double>> x{};
            std::vector<std::vector<double>> u{};
            if (problem == "wb")
                continue;
            acados_interface_.UpdatePrediction(x,u,problem);
            int n_joints {solver_param_.subsystems_map_joint[problem].size()};
            int counter{0};
            //update data state
            for (int k{0};k<solver_param_.N_+1;k++)
            {   
                //p 
                data_[problem].p[k][0] = x[k][0];
                data_[problem].p[k][1] = x[k][1];
                data_[problem].p[k][2] = x[k][2];

                //quat
                data_[problem].quat[k][0] = x[k][3];
                data_[problem].quat[k][1] = x[k][4];
                data_[problem].quat[k][2] = x[k][5];
                data_[problem].quat[k][3] = x[k][6];

                //q
                counter = 0;
                for(auto idx : solver_param_.subsystems_map_joint[problem])
                {
                    data_["wb"].q[k][idx] = x[k][7+counter];    
                    counter++;
                }
                
                //dp
                data_[problem].dp[k][0] = x[k][7+n_joints];
                data_[problem].dp[k][1] = x[k][8+n_joints];
                data_[problem].dp[k][2] = x[k][9+n_joints];

                //omega
                data_[problem].omega[k][0] = x[k][10+n_joints];
                data_[problem].omega[k][1] = x[k][11+n_joints];
                data_[problem].omega[k][2] = x[k][12+n_joints];

                //dq
                counter = 0;
                for(auto idx : solver_param_.subsystems_map_joint[problem])
                {
                    data_["wb"].dq[k][idx] = x[k][13+n_joints+counter];
                    counter++;
                }
                
                if (k < solver_param_.N_)
                {
                    //tau
                    counter = 0;
                    for(auto idx : solver_param_.subsystems_map_joint[problem])
                    {
                        data_["wb"].tau[k][idx] = u[k][counter];
                        counter++;
                    }
                    //grf
                    counter = 0;
                    for(auto idx : solver_param_.subsystems_map_contact[problem])
                    {
                        data_["wb"].grf[k][3*idx] = u[k][n_joints+3*counter];
                        data_["wb"].grf[k][3*idx+1] = u[k][n_joints+3*counter+1];
                        data_["wb"].grf[k][3*idx+2] = u[k][n_joints+3*counter+2];
                        counter++;
                    } 
                }
            }
        }

        // update whole body speeds
        for(int k{0};k<solver_param_.N_+1;k++)
        {
            data_["wb"].dp[k][0] = 0;
            data_["wb"].dp[k][1] = 0;
            data_["wb"].dp[k][2] = 0;

            data_["wb"].omega[k][0] = 0;
            data_["wb"].omega[k][1] = 0;
            data_["wb"].omega[k][2] = 0;

            double n{solver_param_.subsystems_name.size()-1};

            for(auto problem : solver_param_.subsystems_name)
            {
                if (problem == "wb")
                    continue;
                data_["wb"].dp[k][0] += (data_[problem].dp[k][0] + data_[problem].dual[k][0]/weight_vec.at("consensus")[0])/n;
                data_["wb"].dp[k][1] += (data_[problem].dp[k][1] + data_[problem].dual[k][1]/weight_vec.at("consensus")[0])/n;
                data_["wb"].dp[k][2] += (data_[problem].dp[k][2] + data_[problem].dual[k][2]/weight_vec.at("consensus")[0])/n;

                data_["wb"].omega[k][0] += (data_[problem].omega[k][0] + data_[problem].dual[k][3]/weight_vec.at("consensus")[0])/n;
                data_["wb"].omega[k][1] += (data_[problem].omega[k][1] + data_[problem].dual[k][4]/weight_vec.at("consensus")[0])/n;
                data_["wb"].omega[k][2] += (data_[problem].omega[k][2] + data_[problem].dual[k][5]/weight_vec.at("consensus")[0])/n;
            }
        }
        // update dual
        for(auto problem : solver_param_.subsystems_name)
        {
            if (problem == "wb")
            {
                continue;
            }
            for (int k{0};k<solver_param_.N_;k++)
            {
                data_[problem].dual[k][0] += (data_[problem].dp[k][0] - data_["wb"].dp[k][0])*weight_vec.at("consensus")[0];
                data_[problem].dual[k][1] += (data_[problem].dp[k][1] - data_["wb"].dp[k][1])*weight_vec.at("consensus")[0];
                data_[problem].dual[k][2] += (data_[problem].dp[k][2] - data_["wb"].dp[k][2])*weight_vec.at("consensus")[0];
                data_[problem].dual[k][3] += (data_[problem].omega[k][0] - data_["wb"].omega[k][0])*weight_vec.at("consensus")[0];
                data_[problem].dual[k][4] += (data_[problem].omega[k][1] - data_["wb"].omega[k][1])*weight_vec.at("consensus")[0];
                data_[problem].dual[k][5] += (data_[problem].omega[k][2] - data_["wb"].omega[k][2])*weight_vec.at("consensus")[0];

                data_[problem].residual[k][0] = (data_[problem].dp[k][0] - data_["wb"].dp[k][0]);
                data_[problem].residual[k][1] = (data_[problem].dp[k][1] - data_["wb"].dp[k][1]);
                data_[problem].residual[k][2] = (data_[problem].dp[k][2] - data_["wb"].dp[k][2]);
                data_[problem].residual[k][3] = (data_[problem].omega[k][0] - data_["wb"].omega[k][0]);
                data_[problem].residual[k][4] = (data_[problem].omega[k][1] - data_["wb"].omega[k][1]);
                data_[problem].residual[k][5] = (data_[problem].omega[k][2] - data_["wb"].omega[k][2]);
            }
            // for (int k{0};k<solver_param_.N_;k++)
            // {
            //     data_[problem].dual[k][0] = (data_["front"].dp[k][0] - (data_["back"].dp[k][0]))*weight_vec.at("consensus")[0];
            //     data_[problem].dual[k][1] = (data_["front"].dp[k][1] - (data_["back"].dp[k][1]))*weight_vec.at("consensus")[0];
            //     data_[problem].dual[k][2] = (data_["front"].dp[k][2] - (data_["back"].dp[k][2]))*weight_vec.at("consensus")[0];
            //     data_[problem].dual[k][3] = (data_["front"].omega[k][0] - (data_["back"].omega[k][0]))*weight_vec.at("consensus")[0];
            //     data_[problem].dual[k][4] = (data_["front"].omega[k][1] - (data_["back"].omega[k][1]))*weight_vec.at("consensus")[0];
            //     data_[problem].dual[k][5] = (data_["front"].omega[k][2] - (data_["back"].omega[k][2]))*weight_vec.at("consensus")[0];
            // }
        }
        // check stopping criteria
        //TODO
    }
    do_init = false;
}
void dsolver::prepare()
{
    acados_interface_.prepare();
}
void dsolver::getControl(std::vector<double> &des_q,std::vector<double> &des_dq,std::vector<double> &des_tau)
{   
    des_q = data_["wb"].q[1];
    des_dq = data_["wb"].dq[1];
    des_tau = data_["wb"].tau[0];

}
void dsolver::getData(std::map<std::string,pdata> &data)
{  
    data = data_;
}