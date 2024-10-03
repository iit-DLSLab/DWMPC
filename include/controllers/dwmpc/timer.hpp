#ifndef TIMER_HPP
#define TIMER_HPP
#include <Eigen/Dense>
#include <algorithm>

class Timer {
public:
    double duty_factor;
    double step_freq;
    std::vector<double> delta;
    std::vector<double> t;
    int n_contact;
    std::vector<bool> init;
    bool start{false};

    Timer();

    std::vector<double> run(double dt);
    
    void get(std::vector<double> &t_out, std::vector<bool> &init_out);
    void set(const std::vector<double> &t,const std::vector<bool> &init);
    void setDelta(const std::vector<double> &delta_in);
    void reset();
    void setParam(double duty_factor, double step_freq);
    void startTimer();
    void stopTimer();
};
#endif