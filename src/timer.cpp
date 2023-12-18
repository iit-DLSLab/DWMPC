#include <timer.hpp>
 
Timer::Timer()
{
    n_contact = 4;
    t = std::vector<double>(n_contact, 0.0);
    init = std::vector<bool>(n_contact, true);
}
 
Eigen::VectorXi Timer::run(double dt) {
    Eigen::VectorXi contact{};
    contact.setZero(n_contact);
    for (int leg = 0; leg < n_contact; ++leg) {
        if (t[leg] == 1.0) {
            t[leg] = 0; //restart
        }
        t[leg] += dt * step_freq;
        if (delta[leg] == -1) {
            contact[leg] = 0;
        } else {
            if (init[leg]) {
                if (t[leg] < delta[leg]) {
                    contact[leg] = 1;
                } else {
                    init[leg] = false;
                    contact[leg] = 1;
                    t[leg] = 0;
                }
            } else {
                if (t[leg] < duty_factor) {
                    contact[leg] = 1;
                } else {
                    contact[leg] = 0;
                }
            }
            if (t[leg] > 1) {
                t[leg] = 1;
            }
        }
    }
    return contact;
}
 
void Timer::set(std::vector<double> t, std::vector<bool> init) {
    this->t = t;
    this->init = init;
}
void Timer::setDelta(const std::vector<double> &delta_in) {
    delta = delta_in;
    reset();
}
void Timer::reset()
{
    std::fill(t.begin(), t.end(), 0.0);
    std::fill(init.begin(), init.end(), true);
}
 
void Timer::setParam(double duty_factor, double step_freq)
{
    this->duty_factor = duty_factor;
    this->step_freq = step_freq;
    reset();
}