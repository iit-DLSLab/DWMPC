#include <dwmpc.hpp>

dwmpc::dwmpc(const int N,const  double dt) : ocp_(N, dt)
{
    N_ = N;
    dt_ = dt;

    // make this an input of the constructor 
    // maybe it would be a hgood idea to have a class for the robot

    n_leg_ = 4;
    n_joint_ = 12;
    contact_seq_.setZero(n_leg_,N_ + 1); // contact sequence

    q0.setZero(n_joint_); // home joint angle
    grf0.setZero(3*n_leg_); // home ground reaction force
    tau0.setZero(n_joint_); // home  torque
    foot0.setZero(3,n_leg_); // home foot position

    //TO DO move all of thid in a robot class
    q0  << 0,0.8,-1.8,0,0.8,-1.8,0,0.8,-1.8,0,0.8,-1.8;
    grf0 << 0, 0, 55, 0, 0, 55, 0, 0, 55, 0, 0, 55;
    tau0 << 4.8, 0.3, 10.7,-4.8 ,0.3, 10.7, 4.8, 0.3, 10.7, -4.8, 0.3, 10.7;
    foot0 << 0.27092872, 0.27092872, -0.20887128, -0.20887128,
           -0.134, 0.134, -0.134, 0.134,
           0.0207477, 0.02074775, 0.02074775, 0.02074775;

    xop_.setZero(7+6+3*n_joint_); // initial state   
    foot_op_.setZero(3,n_leg_); // current foot position
    terrain_height.setZero(n_leg_); // terrain height for each leg

    std::vector<double> q_temp; 
    std::vector<double> dq_temp;
    std::vector<double> grf_temp;

    for(int i{0};i< n_joint_;i++)
    {
        q_temp.push_back(q0[i]);
        dq_temp.push_back(0.0);
    }
    for(int i{0};i< 3*n_leg_;i++)
    {
        grf_temp.push_back(grf0[i]);
    }
    for(int k{0};k < N_;k++)
    {   
        q_.push_back(q_temp);
        dq_.push_back(dq_temp);
        grf_.push_back(grf_temp);
    }
}
void dwmpc::setState(const Eigen::VectorXd &state,const Eigen::VectorXi &contact0,const Eigen::MatrixXd &foot_op)
{   
    xop_ = state;
    foot_op_ = foot_op;
    std::vector<double> initial_condition;
    for(int i = 0; i < state.size();i++)
    {
        initial_condition.push_back(state(i));
    }
    ocp_.setInitialCondition(initial_condition);
    contact_seq_.block(0,0,n_leg_,1) = contact0;
    upate_terrain_height();
}
void dwmpc::upate_terrain_height()
{
    for(int leg{0};leg < n_leg_;leg++)
    {
        if(contact_seq_(leg,0))
        {
            terrain_height(leg) = foot_op_(2,leg);
        }
    }
}
void dwmpc::setGaitParam(double duty_factor, double step_freq, const std::vector<double> &delta)
{
    timer_.setDelta(delta);
    timer_.setParam(duty_factor,step_freq);
}
void dwmpc::updateTimer(const std::vector<double> &t, const std::vector<bool> &init)
{
    timer_.set(t,init);
}
void dwmpc::setDesired(const Eigen::VectorXd &desired)
{   
    // desired robot_height , step_height, linear speed, angular speed
    // 
    // refrence order : p,q,dp,omega,dq,tau,foot_ref,dtau,grf
    int joint_per_leg{3};
    int nx_ref {9+3*joint_per_leg+3}; 
    int nu_ref {joint_per_leg + 3};

    std::vector<std::vector<double>> reference;
    std::vector<double> ref;

    std::vector<bezier_curves_t> bcs(n_leg_);

    Eigen::MatrixXd foot_ref{};
    foot_ref.setZero(3*n_leg_,N_+ 1);
    // p
    ref.push_back(xop_[0]);
    ref.push_back(xop_[0]);
    ref.push_back(desired[0]); // set the robot z to the desired robot height
    // q
    for (int joint{0};joint < n_joint_;joint++)
    {
        ref.push_back(q0[joint]);
    }
    // dp
    ref.push_back(desired[2]);
    ref.push_back(desired[3]);
    ref.push_back(desired[4]);
    // omega
    ref.push_back(desired[5]);
    ref.push_back(desired[6]);
    ref.push_back(desired[7]);
    // dq
    for (int leg{0};leg < n_leg_;leg++)
    {
        ref.push_back(0);
        ref.push_back(0);
        ref.push_back(0);
    }
    // tau
    for (int joint{0};joint < n_joint_;joint++)
    {
        ref.push_back(tau0[joint]);
    }
    // foot_ref
    for (int leg{0};leg < n_leg_;leg++)
    {
        ref.push_back(foot_op_(0,leg));
        ref.push_back(foot_op_(1,leg));
        ref.push_back(foot_op_(2,leg));

        foot_ref(0 + 3 * leg,0) = foot0(0,leg);
        foot_ref(1 + 3 * leg,0) = foot0(1,leg);
        foot_ref(2 + 3 * leg,0) = foot0(2,leg);

    }
    // dtau
    for (int joint{0};joint < n_joint_;joint++)
    {
        ref.push_back(0);
    }
    // grf
    for (int leg{0};leg < n_leg_;leg++)
    {
        ref.push_back(grf0(0 + 3 * leg));
        ref.push_back(grf0(1 + 3 * leg));
        ref.push_back(grf0(2 + 3 * leg));
    }
    reference.push_back(ref);
    for(int k {1}; k < N_ + 1; k++)
    {   
        contact_seq_.block(0,k,n_leg_,1) = timer_.run(dt_);
        //integrate using the desired speed 
        // p
        ref[0] = (xop_[0] + desired[2]*dt_*k);
        ref[1] = (xop_[1] + desired[3]*dt_*k);
        ref[2] = (xop_[2] + desired[4]*dt_*k);

        for (int leg{0};leg < n_leg_;leg++)
        {
            if((!contact_seq_(leg,k) & contact_seq_(leg,k-1)) || !contact_seq_(leg,k-1)  && k == 1) // lift off or already on swing
            {
                Eigen::Vector3d foot_hold;
                foot_hold << foot0(0,leg),foot0(1,leg),terrain_height(leg);
                foot_hold.segment<2>(0) += xop_.segment<2>(0) + desired.segment<2>(2)*dt_*k ;
                foot_hold.segment<2>(0) += 0.5*desired.segment<2>(2)*timer_.duty_factor*timer_.step_freq ;
                
                std::vector<Eigen::Vector3d> cp{};

                if((timer_.t[leg]-timer_.duty_factor)/(1-timer_.duty_factor)<0.5)
                {
                    cp.push_back(Eigen::Vector3d(ref[45 + 3 * leg],ref[46 + 3 * leg],ref[47 + 3 * leg]));
                    cp.push_back(Eigen::Vector3d(ref[45 + 3 * leg],ref[46 + 3 * leg],terrain_height(leg)+desired[1])); // if the state estimator is driffting badliy in z this can be a problem
                    cp.push_back(Eigen::Vector3d(foot_hold[0],foot_hold[1],terrain_height(leg)+desired[1]));
                    cp.push_back(Eigen::Vector3d(foot_hold[0],foot_hold[1],terrain_height(leg)));
                }
                else
                {
                    cp.push_back(Eigen::Vector3d(ref[45 + 3 * leg],ref[46 + 3 * leg],ref[47 + 3 * leg]));
                    cp.push_back(Eigen::Vector3d(foot_hold[0],foot_hold[1],ref[47 + 3 * leg]));
                    cp.push_back(Eigen::Vector3d(foot_hold[0],foot_hold[1],terrain_height(leg)));
                }

                bezier_curves_t bc(cp.begin(), cp.end(),(timer_.t[leg]-timer_.duty_factor)/(1-timer_.duty_factor),1);
                bcs[leg] = bc;

            }
            if (!contact_seq_(leg,k))
            {   
                // feet position just count the elemenr before for a quadruoed if you dont get it this code/research is not for you

                Eigen::Vector3d foot_position{bcs[leg]((timer_.t[leg]-timer_.duty_factor)/(1-timer_.duty_factor))};
                ref[45 + 3 * leg] = foot_position[0];
                ref[46 + 3 * leg] = foot_position[1];
                ref[47 + 3 * leg] = foot_position[2];

                foot_ref(0 + 3 * leg,k) = ref[45 + 3 * leg];
                foot_ref(1 + 3 * leg,k) = ref[46 + 3 * leg];
                foot_ref(2 + 3 * leg,k) = ref[47 + 3 * leg];
            }
            else
            {
                ref[47 + 3 * leg] = terrain_height(leg);

                foot_ref(0 + 3 * leg ,k) = ref[45 + 3 * leg];
                foot_ref(1 + 3 * leg ,k) = ref[46 + 3 * leg];
                foot_ref(2 + 3 * leg ,k) = ref[47 + 3 * leg];

            }
        }
        reference.push_back(ref); 
    }
    ocp_.setReference(reference,nx_ref,nu_ref);

    // std::vector<double> x(N_+1);
    // std::vector<double> y(N_+1);
    // std::vector<double> z(N_+1);
    // for(auto i{0};i < N_+1;i++)
    // {
    //     x[i] = foot_ref(0,i);
    //     y[i] = foot_ref(1,i);
    //     z[i] = foot_ref(2,i);
    // }

    // auto l = matplot::scatter3(x, y, z);
    // matplot::show();
    setParameter(foot_ref);
}

void dwmpc::setParameter(const Eigen::MatrixXd &foot_ref)
{   
    //parameter order: contact_state,foot_ref,q_aux,dq_aux,grf_ref,dt
    
    int np{n_leg_+3*n_leg_+n_joint_+n_joint_+3*n_leg_+1};

    std::vector<std::vector<double>> parameter;
    std::vector<double> param;

    // contact_state
    for (int leg{0};leg < n_leg_;leg++)
    {
        param.push_back(contact_seq_(leg,0));
    }
    // foot_ref
    for (int leg{0};leg < n_leg_;leg++)
    {
        param.push_back(foot_ref(0 + 3 * leg,0));
        param.push_back(foot_ref(1 + 3 * leg,0));
        param.push_back(foot_ref(2 + 3 * leg,0));
    
    }
    // q_aux
    for (int joint{0}; joint < n_joint_; joint++)
    {
        param.push_back(q_[0][joint]);
    }
    // dq_aux
    for (int joint{0}; joint < n_joint_; joint++)
    {
        param.push_back(dq_[0][joint]);
    }
    // grf_ref
    for (int leg{0}; leg < n_leg_; leg++)
    {
        param.push_back(grf_[0][leg*3+0]);
        param.push_back(grf_[0][leg*3+1]);
        param.push_back(grf_[0][leg*3+2]);
    }
    // dt
    param.push_back(dt_);
    parameter.push_back(param);

    for (int k{1}; k < N_; k++)
    {
        // contact_state
        for (int leg{0};leg < n_leg_;leg++)
        {
            param[leg] = contact_seq_(leg,k);
        }
        // foot_ref
        for (int leg{0};leg < n_leg_;leg++)
        {
            param[n_leg_+leg*3+0] = foot_ref(0 + 3 * leg,k);
            param[n_leg_+leg*3+1] = foot_ref(1 + 3 * leg,k);
            param[n_leg_+leg*3+2] = foot_ref(2 + 3 * leg,k);
        }
        // q_aux
        for (int joint{0}; joint < n_joint_; joint++)
        {
            param[n_leg_+3*n_leg_+joint] = q_[k][joint];
        }
        // dq_aux
        for (int joint{0}; joint < n_joint_; joint++)
        {
            param[n_leg_+3*n_leg_+n_joint_+joint+ 3*n_leg_] = dq_[k][joint];
        }
        // grf_ref
        for (int leg{0}; leg < n_leg_; leg++)
        {
            param[n_leg_+3*n_leg_+2*n_joint_+leg*3+0] = grf_[k][leg*3+0];
            param[n_leg_+3*n_leg_+2*n_joint_+leg*3+1] = grf_[k][leg*3+1];
            param[n_leg_+3*n_leg_+2*n_joint_+leg*3+2] = grf_[k][leg*3+2];
        }
        // dt
        param[n_leg_+3*n_leg_+3*n_joint_+3*n_leg_] = dt_; //change dt to handle the switch in the contact state
        parameter.push_back(param);
    }
    ocp_.setParameter(parameter,np);
}
void dwmpc::solve(std::vector<std::vector<double>> &torque)
{
    if(first_)
    {
        ocp_.prepare();
        first_ = false;
    }
    ocp_.solve();
    ocp_.getControl(torque,q_,dq_,grf_);
}
void dwmpc::prepare()
{
    ocp_.prepare();
}