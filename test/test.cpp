#include <iostream>
#include <chrono>

#include <dwmpc.hpp>

int main(int argc, char** argv) {
  // Create an instance of dwmpc
  double dt = 0.01;
  int N = 50;
dwmpc dwmpc(N, dt);

int n_joint = 12;
int n_leg = 4;

Timer timer;

double duty_factor = 0.6;
double step_freq = 1.5;
std::vector<double> delta {0.0,0.5,0.5,0.0};

timer.setParam(duty_factor, step_freq);
timer.setDelta(delta);

dwmpc.setGaitParam(duty_factor, step_freq, delta);

Eigen::VectorXd state;
Eigen::VectorXi contact;
Eigen::MatrixXd foot_op;

state.setZero(7 + 6 + 3 * n_joint);
state.segment(0, 7) << 0.0, 0.0, 0.33, 0.0, 0.0, 0.0, 1.0;
state.segment(7, n_joint) << 0,0.8,-1.8,0,0.8,-1.8,0,0.8,-1.8,0,0.8,-1.8;
state.segment(7 + 6 + 2 * n_joint,n_joint) << 4.8, 0.3, 10.7,-4.8 ,0.3, 10.7, 4.8, 0.3, 10.7, -4.8, 0.3, 10.7;

contact.setZero(n_leg);
contact << 1, 1, 1, 1;

foot_op.setZero(3, n_leg);
foot_op << 0.27092872, 0.27092872, -0.20887128, -0.20887128,
           -0.134, 0.134, -0.134, 0.134,
           0.0207477, 0.02074775, 0.02074775, 0.02074775;

dwmpc.setState(state, contact, foot_op);

double robot_height {0.33};
double step_height {0.06};

Eigen::VectorXd desired;

desired.setZero(8);

desired << robot_height, step_height, 0.1, 0.0, 0.0, 0.0, 0.0, 0.0;

dwmpc.setDesired(desired);

dwmpc.prepare();

std::vector<std::vector<double>> torque;

std::vector<double> torque_temp;

for(int i{0};i < n_joint;i++)
{
    torque_temp.push_back(0.0);
}
for(int k{0};k < N;k++)
{
    torque.push_back(torque_temp);
}


dwmpc.solve(torque);
// timer.run(dt);
auto start = std::chrono::high_resolution_clock::now();

dwmpc.updateTimer({0.0,0.0,0.0,0.0},{true,true,true,true});
dwmpc.setDesired(desired);
dwmpc.prepare();
dwmpc.setState(state, contact, foot_op);
dwmpc.solve(torque);

auto stop = std::chrono::high_resolution_clock::now();

std::cout << "Time taken by function: " << std::chrono::duration_cast<std::chrono::milliseconds>(stop - start).count() << " ms" << std::endl;

std::cout << "Torque: " << std::endl;

for(int i{0};i < n_joint;i++)
{
    std::cout << torque[0][i] << std::endl;
}

}