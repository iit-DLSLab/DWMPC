#include <Eigen/Dense>
#include <algorithm>
 
class Timer {
private:
    double duty_factor;
    double step_freq;
    std::vector<double> delta;
    std::vector<double> t;
    int n_contact;
    std::vector<bool> init;
 
public:
 
    Timer();
 
    Eigen::VectorXi run(double dt);
 
    void set(std::vector<double> t, std::vector<bool> init);
    void setDelta(const std::vector<double> &delta_in);
    void reset();
    void setParam(double duty_factor, double step_freq);
};