#include "controllers/dwmpc/timer.hpp"

Timer::Timer()
{
    n_contact = 4;
    t = std::vector<double>(n_contact, 0.0);
    init = std::vector<bool>(n_contact, true);
}

std::vector<double> Timer::run(double dt) {
    std::vector<double> contact(n_contact, 0);
    for (int leg = 0; leg < n_contact; ++leg) {
        if(start)
        {
            t[leg] += dt * step_freq;
            if(t[leg] > 1.0)
            {
                t[leg] = t[leg] - 1.0;
            }
        }
        else
        {
            t[leg] = 0.0;
            init[leg] = true;
        }
        
        if (delta[leg] == -1)
        {
            contact[leg] = 0.0;
        } 
        else
        {
            if (init[leg]) 
            {
                if (t[leg] < delta[leg]) 
                {
                    contact[leg] = 1;
                } 
                else
                {
                    init[leg] = false;
                    contact[leg] = 1;
                    t[leg] = 0.0;
                }
            } 
            else
            {
                if (t[leg] < duty_factor)
                {
                    contact[leg] = 1;
                } else
                {
                    contact[leg] = 0.0;
                }
            }
            // if (t[leg] > 1) {
            //     t[leg] = 0.99;
            // }
        }
    }
    return contact;
}
void Timer::get(std::vector<double> &t_out, std::vector<bool> &init_out) {
    t_out = t;
    init_out = init;
}
void Timer::set(const std::vector<double> &t,const std::vector<bool> &init) {
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
void Timer::startTimer()
{
    start = true;
}
void Timer::stopTimer()
{   
    start = false;
    
}
