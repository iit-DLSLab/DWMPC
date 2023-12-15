// #include <acados_c/ocp_nlp_interface.h>
// #include <acados_c/external_function_interface.h>

//#include "acados_solver_FL.h"
//#include "acados_solver_FR.h"
//#include "acados_solver_RL.h"
//#include "acados_solver_RR.h"

#include <iostream>
#include <vector>

#include <map>
#include <thread>

class dwocp {
    public:
        dwocp(int N, double dt);

//        // setter
//        void setReference(std::vector<std::vector<double>> &reference,int nx_ref,int nu_ref);
//        void setInitialCondition(std::vector<double> &initial_condition);
//        void setParameter(std::vector<std::vector<double>> &parameter,int np);
//        void setWeight(std::vector<double> &weight);
//        void setWarmStart(std::vector<std::vector<double>> &x_start,std::vector<std::vector<double>> &u_start);
//        void solve();
//        void prepare();
//        // getter
//        void getControl(std::vector<std::vector<double>> &torque, std::vector<std::vector<double>> &q, std::vector<std::vector<double>> &dq,std::vector<std::vector<double>> &grf);   
//        // void getSolution(Eigen::VectorXd &solution);

    private:

        //internal variables

//        // acados objects
//        FL_solver_capsule *FL_capsule_; 
//        FR_solver_capsule *FR_capsule_; 
//        RL_solver_capsule *RL_capsule_; 
//        RR_solver_capsule *RR_capsule_; 
//
//        std::map<std::string,ocp_nlp_config> nlp_config;
//        std::map<std::string,ocp_nlp_dims> nlp_dims;
//        std::map<std::string,ocp_nlp_in> nlp_in;
//        std::map<std::string,ocp_nlp_out> nlp_out;
//        std::map<std::string,ocp_nlp_solver> nlp_solver;
//        //TO DO try to make this a map to 
//        void *nlp_opts_FL;
//        void *nlp_opts_FR;
//        void *nlp_opts_RR;
//        void *nlp_opts_RL;
//        std::string legs[4] = { "FR", "FL", "RR", "RL" };

        int N_; // prediction horizon
        double dt_; // sampling time

//        //TO Do make it a map
//        int nx;
//        int nu;
//        std::map<std::string,int> status_;
//    
//        // //internal functions
//        void solveFL();
//        void solveFR();
//        void solveRL();
//        void solveRR();
};