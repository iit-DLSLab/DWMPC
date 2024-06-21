#ifndef DWMPC_HPP
#define DWMPC_HPP

#include <iostream>
#include "yaml-cpp/yaml.h"
#include "controllers/dwmpc/timer.hpp"

#include "ndcurves/bezier_curve.h"
#include <Eigen/Dense>
#include "controllers/dwmpc/distributed_solver.hpp"
#include <numeric>

typedef double timer_param_t;
typedef Eigen::VectorXd pointX_t;
typedef double num_t;
typedef ndcurves::bezier_curve <timer_param_t, num_t, true, pointX_t> bezier_curves_t; 
// add other includes here

namespace controllers
{    
    class Dwmpc
    {
        public:
        Dwmpc();
        ~Dwmpc();
        void setDesiredAndParameter(const std::vector<double> &contact0,
                                    const Eigen::MatrixXd &foot_op,
                                    const std::map<std::string,std::vector<double>> initial_condition,
                                    std::map<std::string,std::vector<std::vector<double>>> &ref,
                                    std::map<std::string,std::vector<std::vector<double>>> &param);
        void run(const Eigen::VectorXd &p,
                 const Eigen::Quaterniond &quat,
                 const Eigen::VectorXd &q,
                 const Eigen::VectorXd &dp,
                 const Eigen::VectorXd &omega,
                 const Eigen::VectorXd &dq,
                 const double &loop_dt,
                 const Eigen::MatrixXd &foot_op,
                 const Eigen::VectorXd &desired_linear_speed,
                 const Eigen::VectorXd &desired_angular_speed,
                 const Eigen::Quaterniond &desired_orientation,
                 std::vector<Eigen::Vector3d> &sphere_pos,
                 std::vector<Eigen::Vector4d> &sphere_color,
                 std::vector<double> &sphere_radius,
                 std::vector<Eigen::Vector3d> &arrow_pos,
                 std::vector<Eigen::Vector4d> &arrow_color,
                 std::vector<Eigen::Vector4d> &arrow_quat,
                 std::vector<double> &arrow_length,
                 std::vector<double> &des_contact,
                 std::vector<double> &des_tau,
                 std::vector<double> &des_q,
                 std::vector<double> &des_dq);
        void init();
        void setWeight(const std::map<std::string,std::vector<double>> &weight_vec);
        void setGaitParam(const double duty_factor,const double step_freq,const std::vector<double> &delta);
        void setGaitParam(const double duty_factor,const double step_freq, const int gait_type);
        void updateTimer(const std::vector<double> &t, const std::vector<bool> &init);
        void reset();
        void startWalking();
        void stopWalking();
        void reorder_contact(Eigen::MatrixXd &contact);
        void reorder_contact(std::vector<double> &contact);
        void reorder_joints(Eigen::VectorXd &joint,const bool in);
        void reorder_joints(std::vector<double> &joint,const bool in);
        void getFullPrediction(std::map<std::string,pdata> &data);
        void prepare();
        void sineWave(std::map<std::string,std::vector<std::vector<double>>> &ref, std::map<std::string,std::vector<std::vector<double>>> &param);
        void setSineParam(double frequency, double amplitude);
        void startSineWave();
        void stopSineWave();
        std::map<std::string,std::vector<double>> getWeight(){return weight_vec_;};
    private:
        YAML::Node config;
        double proprioHeight(double desired_height);
        void upate_terrain_height(const std::vector<double> &contact0,
                                  const Eigen::MatrixXd &foot_op);
        dsolver ocp_;
        int N_;

        int n_joint_wb_;
        int n_contact_wb_;
        double time_{0};
        double frequency_{1};
        double amplitude_{0.1};
        bool do_sine_wave_{false};
        bool do_init_{true};
        
        std::vector<double> terrain_height_; // terrain height for each leg
        std::map<std::string,std::vector<double>> weight_vec_; // cost function weights

        Timer timer_; //timer to define the gait
        std::vector<bezier_curves_t> bcs_{4}; // TODO CHANGE THIS
        std::map<std::string,std::vector<double>> desired_; // desired values for the controller

        // TO DO change this to separate class that reads from a config file
        std::vector<double> q0_ {}; // home joint angle
        std::vector<double> foot0_ {}; // home foot position 
    };
} //namespace controllers

#endif /* end of include guard: DWMPC_HPP */
