#include <iostream>
#include <chrono>

#include "controllers/dwmpc/dwmpc.hpp"
#include "motion_generators/vis/vis.hpp"

int main(int argc, char** argv) {
    controllers::Dwmpc mpc;
    mpc.init();

    Eigen::Vector3d p{0,0,0.33};
    Eigen::Quaterniond quat{};
    quat.coeffs() << 0,0,0,1;
    Eigen::Vector3d dp{0,0,0};
    Eigen::Vector3d omega{0,0,0};
    Eigen::Vector3d desired_linear_speed{0.0,0,0};
    Eigen::Vector3d desired_angular_speed{0,0,0};
    Eigen::Quaterniond desired_orientation{};
    desired_orientation.coeffs() << 0,0,0,1;
    std::vector<double> des_tau;
    std::vector<double> des_q;
    std::vector<double> des_dq;
    Eigen::VectorXd q{};
    q.setZero(12);
    std::cout << "desired_orientation" << desired_orientation.coeffs() << std::endl;
    q << 0,0.8,-1.8,0,0.8,-1.8,0,0.8,-1.8,0,0.8,-1.8;
    Eigen::VectorXd dq{Eigen::VectorXd::Zero(12)};
    Eigen::MatrixXd foot_op{Eigen::MatrixXd::Zero(3, 4)};

    foot_op << 0.27092872, 0.27092872, -0.20887128, -0.20887128,
           0.134, -0.134, 0.134, -0.134,
           0.0207477, 0.02074775, 0.02074775, 0.02074775;

    std::cout << "foot_op" <<  foot_op << std::endl;
    auto start = std::chrono::high_resolution_clock::now();
    std::vector<Eigen::Vector3d> sphere_pos;
    std::vector<Eigen::Vector4d> sphere_color;
    std::vector<double> sphere_radius;
    std::vector<Eigen::Vector3d> arrow_pos;
    std::vector<Eigen::Vector4d> arrow_color;
    std::vector<Eigen::Vector4d> arrow_quat;
    std::vector<double> arrow_length;
    std::vector<double> des_contact;
    motion_generators::Vis vis;
    vis.init();
    Eigen::Vector4d current_contact{1,1,1,1};
    // mpc.startWalking();
    mpc.run(p,
              quat,
              q,
              dp,
              omega,
              dq,
              0.01,
              current_contact,
             foot_op,
             desired_linear_speed,
             desired_angular_speed,
             desired_orientation,
             sphere_pos,
             sphere_color,
             sphere_radius,
             arrow_pos,
             arrow_color,
             arrow_quat,
             arrow_length,
             des_contact,
             des_tau,
             des_q,
             des_dq);
   
    std::vector<double> p_vec{};
    std::vector<double> quat_vec{};
    std::vector<double> q_vec{};

    std::map<std::string,pdata> data;
    mpc.getFullPrediction(data);
     auto stop = std::chrono::high_resolution_clock::now();

    std::cout << "Time taken by function test: " << std::chrono::duration_cast<std::chrono::milliseconds>(stop - start).count() << " ms" << std::endl;
    for (int i = 0; i < des_tau.size(); i++) {
        std::cout << "des_tau[" << i << "]: " << des_tau[i] << std::endl;
        std::cout << "des_q[" << i << "]: " << des_q[i] << std::endl;
        std::cout << "des_dq[" << i << "]: " << des_dq[i] << std::endl;
    }
    int k = 0;
    while(true)
    {
        // for (int i = 0; i < p.size(); i++) 
        // {
        //     p_vec.push_back(p[i]);
        // }
        // for (int i = 0; i < quat.coeffs().size(); i++) 
        // {
        //     quat_vec.push_back(quat.coeffs()[i]);
        // }
        // for (int i = 0; i < q.size(); i++) 
        // {
        //     q_vec.push_back(q[i]);
        // }

        // vis.run(q_vec,p_vec,quat_vec,sphere_pos,sphere_color,sphere_radius,arrow_pos,arrow_quat,arrow_color,arrow_length);
        vis.run(data["wb"].q[k],data["front"].p[k],data["front"].quat[k],sphere_pos,sphere_color,sphere_radius,arrow_pos,arrow_quat,arrow_color,arrow_length);

        std::this_thread::sleep_for(std::chrono::milliseconds(20));
        

        k++;
        if(k == 30)
        {   
            // p[0] = data["front"].p[k][0];
            // p[1] = data["front"].p[k][1];
            // p[2] = data["front"].p[k][2];
            // quat.x() = data["front"].quat[k][0];
            // quat.y() = data["front"].quat[k][1];
            // quat.z() = data["front"].quat[k][2];
            // quat.w() = data["front"].quat[k][3];
            // for (int i = 0; i < q.size(); i++) 
            // {
            //     q[i] = data["wb"].q[k][i];
            // }
            // dp[0] = data["front"].p[k][0];
            // dp[1] = data["front"].p[k][1];
            // dp[2] = data["front"].p[k][2];
            // omega[0] = data["front"].omega[k][0];
            // omega[1] = data["front"].omega[k][1];
            // omega[2] = data["front"].omega[k][2];
            // for (int i = 0; i < dq.size(); i++) 
            // {
            //     dq[i] = data["wb"].dq[k][i];
            // }
            mpc.run(p,
              quat,
              q,
              dp,
              omega,
              dq,
              0.01,
              current_contact,
             foot_op,
             desired_linear_speed,
             desired_angular_speed,
             desired_orientation,
             sphere_pos,
             sphere_color,
             sphere_radius,
             arrow_pos,
             arrow_color,
             arrow_quat,
             arrow_length,
             des_contact,
             des_tau,
             des_q,
             des_dq);

            k = 0;
            // mpc.getFullPrediction(data);
             }
    
    }
   
    
    return 0;
}