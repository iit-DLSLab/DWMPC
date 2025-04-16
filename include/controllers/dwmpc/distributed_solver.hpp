#include <iostream>
#include <vector>
#include <cmath>
#include <algorithm>
#include "controllers/dwmpc/acadosInterface.hpp"
#include <chrono>

class parameter
{   public:
    int max_iteration{100}; // maximum number of iteration for the distributed solver
    bool receding_horizon{true}; // flag if using a reciding horizon
    std::vector<std::string> subsystems_name; //vecotr of the names of the subsystems
    std::map<std::string,std::vector<int>> subsystems_map_joint;//nap the name of the subsistem to the joint number in the whole body 
    std::map<std::string,std::vector<int>> subsystems_map_contact;//nap the name of the subsistem to the contact number in the whole body 
    int n_problem{}; // number of the subsystems
    int N_legs{}; // number of legs
    int N_{};// lenght of the horizon  
    int n_ineq_0{}; // number of inequality constraints for the first node
    int n_ineq{}; // number of inequality constraints for the rest of the nodes
    std::vector<double> lh0{}; // lower bound for inequality constraints for the first node
    std::vector<double> lh{}; // lower bound for inequality constraints for the rest of the nodes
    int nw0{}; // size of the weight matrix for the first node
    int nw{}; // size of the weight matrix for the rest of the nodes
    int nwe{}; // size of the weight matrix for the last node
};
class pdata
{   
    public:
    std::vector<std::vector<double>> p{}; // position
    std::vector<std::vector<double>> quat{}; // quaternion
    std::vector<std::vector<double>> q{}; // joint angle
    std::vector<std::vector<double>> dp{}; // linear velocity prediction
    std::vector<std::vector<double>> omega{}; //angular velocity 
    std::vector<std::vector<double>> dq{}; // joint velocity
    std::vector<std::vector<double>> grf{}; // ground reaction forces
    std::vector<std::vector<double>> tau{}; // joint torque

    std::vector<std::vector<double>> dual{}; //dual 
    std::vector<std::vector<double>> residual{}; //residual

};
class dsolver {
    public:
        dsolver();
        void init(const parameter &solver_param);
        void solve( bool &do_init,
                    const std::map<std::string,std::vector<double>> inital_condition,
                    const std::map<std::string,std::vector<std::vector<double>>> &ref,
                    const std::map<std::string,std::vector<std::vector<double>>> &param,
                    const std::map<std::string,std::vector<double>> &weight_vec);
        void getControl(std::vector<double> &des_q,std::vector<double> &des_dq,std::vector<double> &des_tau);
        void getData(std::map<std::string,pdata> &data);
        void prepare(); 
    private:
        parameter solver_param_;
        std::map<std::string,pdata> data_;
        acadosInterface acados_interface_;
};
