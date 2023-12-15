#include <dwocp.hpp>
//#include <timer.hpp>

#include "ndcurves/bezier_curve.h"
#include <Eigen/Dense>

// #include <matplot/matplot.h>

//typedef double timer_param_t;
//typedef Eigen::VectorXd pointX_t;
//typedef double num_t;
//typedef ndcurves::bezier_curve <timer_param_t, num_t, true, pointX_t> bezier_curves_t; 

class dwmpc {
    public:
        dwmpc(const int N, const double dt);
//        void setState(const Eigen::VectorXd &state,const Eigen::VectorXi &contact0,const Eigen::MatrixXd &foot_op);
//        void setDesired(const Eigen::VectorXd &desired);
//        void setGaitParam(const double duty_factor,const double step_freq,const std::vector<double> &delta);
//        void updateTimer(const std::vector<double> &t, const std::vector<bool> &init);
//        void solve(std::vector<std::vector<double>> &torque);
//        void prepare();
    private:
//        void setParameter(const Eigen::MatrixXd &foot_ref);
//        void upate_terrain_height();
        dwocp ocp_;
        int N_;
        double dt_;
//        
//        int n_leg_;
//        int n_joint_;
//        Eigen::MatrixXi contact_seq_{}; // contact sequence
//        bool first_{true};
//
//        Eigen::VectorXd q0; // home joint angle
//        Eigen::VectorXd tau0; // home  torque
//        Eigen::VectorXd grf0; // home ground reaction force
//        Eigen::MatrixXd foot0; // home foot position
//
//        Eigen::VectorXd xop_; // initial state
//        Eigen::MatrixXd foot_op_; // current foot position
//        Eigen::VectorXd terrain_height; // terrain height for each leg
//
//        std::vector<std::vector<double>> q_; // joint angle
//        std::vector<std::vector<double>> dq_; // joint velocity
//        std::vector<std::vector<double>> grf_; // joint torque
//
//        Timer timer_;
//
};  // 
