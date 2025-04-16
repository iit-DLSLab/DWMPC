// #include <acados_c/ocp_nlp_interface.h>
// #include <acados_c/external_function_interface.h>

#include "acados_solver_front.h"
#include "acados_solver_back.h"


#include <vector>
#include <map>
#include <cstring>
#include <chrono>
#include <cstdio>
#include <stdlib.h>
#include <csignal>
#include <iostream>
#include <thread>
#include <string>


class acadosInterface {
    
    public:

        acadosInterface();
        ~acadosInterface();
        void init(const int N);
        // setter
        void setInitialCondition(std::vector<double> &initial_condition,const std::string &problem);
        void warmstart(const std::string &problem);
        void setParameter(std::vector<std::vector<double>> &parameter,const  int np, const std::string &problem);
        void setConstraints(std::vector<std::vector<double>> &constraint, const int nlh0, const int nlh, const std::string &problem);
        void setWeight(std::vector<std::vector<double>> &weight,const int &n_w0,const int &n_w, const int &n_we,const std::string &problem);
        void setReference(std::vector<std::vector<double>> &reference,const int n_ref0, const int n_ref, const int n_ref_e, const std::string &problem);
        void solve();
        void prepare();
        // getter
        void UpdatePrediction(std::vector<std::vector<double>> &x,std::vector<std::vector<double>> &u, const std::string &problem);   
        // void getSolution(Eigen::VectorXd &solution);

    private:

        //internal variables

        // acados objects
        front_solver_capsule *front_capsule_; 
        back_solver_capsule *back_capsule_; 

        std::map<std::string,ocp_nlp_config> nlp_config;
        std::map<std::string,ocp_nlp_dims> nlp_dims;
        std::map<std::string,ocp_nlp_in> nlp_in;
        std::map<std::string,ocp_nlp_out> nlp_out;
        std::map<std::string,ocp_nlp_out> nlp_out_sens;
        std::map<std::string,ocp_nlp_solver> nlp_solver;
        //TO DO try to make this a map to 
        void *nlp_opts_front;
        void *nlp_opts_back;

        int N_; // prediction horizon
        int nx_;
        int nu_;
        std::map<std::string,int> status_;
    
        // //internal functions
        void solvefront();
        void solveback();
        void waitWorkers();
        bool run{true};

        std::atomic_bool start_front{false};
        std::atomic_bool start_back{false};

        std::thread front;
        std::thread back;
};