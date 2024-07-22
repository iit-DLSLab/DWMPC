#include "controllers/dwmpc/dwmpc.hpp"

namespace controllers
{
    Dwmpc::Dwmpc() : ocp_(), config(YAML::LoadFile("/usr/include/dls2/controllers/dwmpc/config/config.yaml"))
    {}
    Dwmpc::~Dwmpc()
    { }
    void Dwmpc::setWeight(const std::map<std::string,std::vector<double>> &weight_vec)
    {
        weight_vec_ = weight_vec;
    }
    void Dwmpc::reorder_contact(Eigen::MatrixXd &contact)
    {
        Eigen::MatrixXd c;
        c.setZero(3,n_contact_wb_);
        c.col(0) = contact.col(1);
        c.col(1) = contact.col(0);
        c.col(2) = contact.col(3);
        c.col(3) = contact.col(2);
        contact = c;
    }
    void Dwmpc::reorder_contact(std::vector<double> &contact)
    {
        std::vector<double> c;
        c.push_back(contact[1]);
        c.push_back(contact[0]);
        c.push_back(contact[3]);
        c.push_back(contact[2]);
        contact = c;
    }
    void Dwmpc::reorder_joints(Eigen::VectorXd &joint,const  bool in)
    {
        Eigen::VectorXd j;
        j.setZero(n_joint_wb_);
        j.segment<3>(0) = joint.segment<3>(3);
        j.segment<3>(3) = joint.segment<3>(0);
        j.segment<3>(6) = joint.segment<3>(9);
        j.segment<3>(9) = joint.segment<3>(6);
        if(in)
        {
            j[3] = -j[3];
            j[9] = -j[9];
        }
        else
        {
            j[0] = -j[0];
            j[6] = -j[6];
        }
        joint = j;
    }
    void Dwmpc::reorder_joints(std::vector<double> &joint,const bool in)
    {
        std::vector<double> j;
        j.push_back(joint[3]);
        j.push_back(joint[4]);
        j.push_back(joint[5]);
        j.push_back(joint[0]);
        j.push_back(joint[1]);
        j.push_back(joint[2]);
        j.push_back(joint[9]);
        j.push_back(joint[10]);
        j.push_back(joint[11]);
        j.push_back(joint[6]);
        j.push_back(joint[7]);
        j.push_back(joint[8]);
        if(in)
        {
            j[3] = -j[3];
            j[9] = -j[9];
        }
        else
        {
            j[0] = -j[0];
            j[6] = -j[6];
        }
        joint = j;
    }
    void Dwmpc::init()
    {   
        N_ = config["N_step"].as<int>();
        
        n_joint_wb_ = config["n_joint_wb"].as<int>();
        
        n_contact_wb_ = config["n_contact_wb"].as<int>();
        
        for(int i{0};i < n_contact_wb_;i++)
        {
            terrain_height_.push_back(0); // terrain height for each leg
        }  

        q0_ = config["q0"].as<std::vector<double>>();

        foot0_ = config["foot0"].as<std::vector<double>>();

        do_init_ = true;
       
        parameter solver_param{};

        solver_param.max_iteration = config["max_iteration"].as<int>();
        
        solver_param.n_problem = config["n_problem"].as<int>();
        
        solver_param.receding_horizon = config["receding_horizon"].as<bool>();
        
        solver_param.subsystems_name = config["subsystems_name"].as<std::vector<std::string>>();

        for(int subsystem{0};subsystem < solver_param.subsystems_name.size();subsystem++)
        {
            solver_param.subsystems_map_joint[solver_param.subsystems_name[subsystem]] = config["subsystems_map_joint"][solver_param.subsystems_name[subsystem]].as<std::vector<int>>();
            solver_param.subsystems_map_contact[solver_param.subsystems_name[subsystem]] = config["subsystems_map_contact"][solver_param.subsystems_name[subsystem]].as<std::vector<int>>();
        }
        
        solver_param.N_legs = config["n_legs"].as<int>();
        
        solver_param.N_ =  config["N_step"].as<int>();

        solver_param.n_ineq_0 = config["n_ineq_0"].as<int>();

        solver_param.n_ineq = config["n_ineq"].as<int>();
        
        for(auto constraint : config["constraint"]["lh0"].as<std::vector<std::string>>())
        {   
            for(auto value : config["constraint"][constraint].as<std::vector<double>>())
            {
                solver_param.lh0.push_back(value);
            }
        }

        for(auto constraint : config["constraint"]["lh"].as<std::vector<std::string>>())
        {   
            for(auto value : config["constraint"][constraint].as<std::vector<double>>())
            {
                solver_param.lh.push_back(value);
            }
        }

        ocp_.init(solver_param);

        // set the desired to default values

        desired_["robot_height"] = config["robot_height"].as<std::vector<double>>();
        
        desired_["step_height"] = config["step_height"].as<std::vector<double>>();
        
        desired_["quat"] = config["quat"].as<std::vector<double>>();
        
        desired_["dp"] = config["dp"].as<std::vector<double>>();
        
        desired_["omega"] = config["omega"].as<std::vector<double>>();

        weight_vec_["p"] = config["weight_p"].as<std::vector<double>>();
        
        weight_vec_["quat"] = config["weight_quat"].as<std::vector<double>>();
        
        weight_vec_["q"] = config["weight_q"].as<std::vector<double>>();
        
        weight_vec_["dp"] = config["weight_dp"].as<std::vector<double>>();
        
        weight_vec_["omega"] = config["weight_omega"].as<std::vector<double>>();
        
        weight_vec_["dq"] = config["weight_dq"].as<std::vector<double>>();
        
        weight_vec_["tau"] = config["weight_tau"].as<std::vector<double>>();
        
        weight_vec_["grf"] = config["weight_grf"].as<std::vector<double>>();
        
        weight_vec_["foot_stance"] = config["weight_foot_stance"].as<std::vector<double>>();
        
        weight_vec_["foot_swing"] = config["weight_foot_swing"].as<std::vector<double>>();
        
        weight_vec_["consensus"] = config["weight_consensus"].as<std::vector<double>>();
        
        timer_.setDelta(config["delta"].as<std::vector<double>>());
        timer_.setParam(config["duty_factor"].as<double>(),config["step_freq"].as<double>());
        timer_.set({0,0,0,0},{true,true,true,true}); 

    }
    void Dwmpc::run(const Eigen::VectorXd &p,
                        const Eigen::Quaterniond &quat,
                        const Eigen::VectorXd &q_op,
                        const Eigen::VectorXd &dp,
                        const Eigen::VectorXd &omega,
                        const Eigen::VectorXd &dq_op,
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
                        std::vector<double> &des_dq)
    {   
        // build the initial condition map
        std::map<std::string,std::vector<double>> initial_condition;
        initial_condition["p"] = {p[0],p[1],p[2]};
        initial_condition["quat"] = {quat.x(),quat.y(),quat.z(),quat.w()};
        initial_condition["dp"] = {dp[0],dp[1],dp[2]};
        initial_condition["omega"] = {omega[0],omega[1],omega[2]};

        //initialize the timer
        std::vector<double> contact0 {timer_.run(loop_dt)};
        des_contact = contact0;
        // reorder_contact(des_contact);
        time_ += loop_dt;
        //save t and init values for next iteration
        std::vector <double> t{};
        std::vector <bool> init{};

        
        timer_.get(t,init);
        
        Eigen::VectorXd q = q_op;
        Eigen::VectorXd dq = dq_op;
        
        // reorder_joints(q,true);
        // reorder_joints(dq,true);

        for (auto i{0};i < n_joint_wb_;i++)
        {
            initial_condition["q"].push_back(q[i]);
            initial_condition["dq"].push_back(dq[i]);
        }

        Eigen::MatrixXd foot = foot_op;
        // reorder_contact(foot);

        upate_terrain_height(contact0,foot);
        // update the desired values
    
        //get the yaw from the quaternion
        double yaw = std::atan2(2*(quat.w()*quat.z() + quat.x()*quat.y()),1-2*(quat.y()*quat.y() + quat.z()*quat.z()));
        //turn the desired orientation by the yaw
        Eigen::Quaterniond yaw_rotation(Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()));
        Eigen::Quaterniond rotated_desired_orientation = yaw_rotation * desired_orientation;

        desired_["quat"][0] = rotated_desired_orientation.x();
        desired_["quat"][1] = rotated_desired_orientation.y();
        desired_["quat"][2] = rotated_desired_orientation.z();
        desired_["quat"][3] = rotated_desired_orientation.w();

        desired_["dp"][0] = cos(yaw)*desired_linear_speed[0] - sin(yaw)*desired_linear_speed[1];
        desired_["dp"][1] = cos(yaw)*desired_linear_speed[1] + sin(yaw)*desired_linear_speed[0];
        desired_["dp"][2] = desired_linear_speed[2];

        desired_["omega"][0] = cos(yaw)*desired_angular_speed[0] - sin(yaw)*desired_angular_speed[1];
        desired_["omega"][1] = cos(yaw)*desired_angular_speed[1] + sin(yaw)*desired_angular_speed[0];
        desired_["omega"][2] = desired_angular_speed[2];

        std::map<std::string,std::vector<std::vector<double>>> ref;
        std::map<std::string,std::vector<std::vector<double>>> param;

        setDesiredAndParameter(contact0,foot,initial_condition,ref,param);
        if(do_sine_wave_)
        {
            sineWave(ref,param);   
        }
        // solve the ocp
        ocp_.solve(do_init_,
               initial_condition,
               ref,
               param,
               weight_vec_);

        //FILL the VIS message
        //ref values

        Eigen::Vector3d _pos{0,0,0};
        double _radius{0.003};
        //opti value
        std::map<std::string,pdata> data;
        getFullPrediction(data);
        for(int k{0};k<N_;k++)
        {   
            //ref values
            _pos[0] = ref["p"][k][0];
            _pos[1] = ref["p"][k][1];
            _pos[2] = ref["p"][k][2];
            sphere_pos.push_back(_pos);
            sphere_color.push_back({1, 0, 0, static_cast<double>((N_ - k)) / static_cast<double>(2*N_)+0.3});
            sphere_radius.push_back(_radius);
            for (int leg{0};leg < n_contact_wb_;leg++)
            {   
                _pos[0] = ref["foot"][k][3*leg];
                _pos[1] = ref["foot"][k][3*leg+1];
                _pos[2] = ref["foot"][k][3*leg+2];

                sphere_pos.push_back(_pos);
                if(leg == 0)
                {
                    sphere_color.push_back({0, 1, 0, static_cast<double>((N_ - k)) / static_cast<double>(2*N_)+0.3});
                }
                else if (leg == 1)
                {
                    sphere_color.push_back({1, 0, 0, static_cast<double>((N_ - k)) / static_cast<double>(2*N_)+0.3});
                }
                else if (leg == 2)
                {
                    sphere_color.push_back({1,0,1, static_cast<double>((N_ - k)) / static_cast<double>(2*N_)+0.3});
                }
                else if (leg == 3)
                {
                    sphere_color.push_back({0,0,1, static_cast<double>((N_ - k)) / static_cast<double>(2*N_)+0.3});
                }
                sphere_radius.push_back(_radius);
            }
            _pos[0] = data["front"].p[k][0];
            _pos[1] = data["front"].p[k][1];
            _pos[2] = data["front"].p[k][2];
            sphere_pos.push_back(_pos);
            sphere_color.push_back({0.25, 0.25, 0.25, static_cast<double>((N_ - k)) / static_cast<double>(2*N_)+0.3});
            sphere_radius.push_back(_radius);
        
            _pos[0] = data["back"].p[k][0];
            _pos[1] = data["back"].p[k][1];
            _pos[2] = data["back"].p[k][2];
            sphere_pos.push_back(_pos);
            sphere_color.push_back({0.25, 0.25, 0.25, static_cast<double>((N_ - k)) / static_cast<double>(2*N_)+0.3});
            sphere_radius.push_back(_radius);
    
        }
        
        Eigen::MatrixXd grf;
        grf.setZero(3,n_contact_wb_);
       
        for (int leg{0};leg < n_contact_wb_;leg++)
        {   
            grf.col(leg) << data["wb"].grf[0][3*leg], data["wb"].grf[0][3*leg+1], data["wb"].grf[0][3*leg+2];
        }
        for (int leg{0};leg < n_contact_wb_;leg++)
        {   
            arrow_pos.push_back(foot.col(leg));
            arrow_length.push_back(grf.col(leg).norm()/220);
            if(leg == 0)
                {
                    arrow_color.push_back({0, 1, 0, 1});
                }
                else if (leg == 1)
                {
                    arrow_color.push_back({1, 0, 0, 1});
                }
                else if (leg == 2)
                {
                    arrow_color.push_back({1, 0, 1, 1});
                }
                else if (leg == 3)
                {
                    arrow_color.push_back({0, 0, 1, 1});
                }
            //set the orrientation of the arrow depending on the grf
            Eigen::Vector3d _dir = grf.col(leg)/grf.col(leg).norm();
            double angle = std::acos(_dir[2]);
            Eigen::Quaterniond _quat(Eigen::AngleAxisd(angle, _dir));
            arrow_quat.push_back({_quat.x(),_quat.y(),_quat.z(),_quat.w()});
           
        }
        // set the timer state coherently with the wall clock
        timer_.set(t,init);
        // update desired torque, joint angle and joint velocity
        ocp_.getControl(des_q,des_dq,des_tau);   
        // reorder the joint values
        // reorder_joints(des_q,false);
        // reorder_joints(des_dq,false);
        // reorder_joints(des_tau,false);  

    }
    void Dwmpc::upate_terrain_height(const std::vector<double> &contact0, const Eigen::MatrixXd &foot_op)
    {
        for(int leg{0};leg < n_contact_wb_;leg++)
        {
            if(contact0[leg] > 0)
            {
                // terrain_height_[leg] = -0.33;
                terrain_height_[leg] = foot_op(2,leg);
            }
        }
    }
    double Dwmpc::proprioHeight(double desired_height)
    {   
        double height{0};
        for(int leg{0};leg < n_contact_wb_;leg++)
        {
            height += terrain_height_[leg]/n_contact_wb_;

        }
        return height + desired_height;
    }
    void Dwmpc::setGaitParam(const double duty_factor, const double step_freq, const std::vector<double> &delta)
    {
        timer_.setDelta(delta);
        timer_.setParam(duty_factor,step_freq);
    }
    void Dwmpc::updateTimer(const std::vector<double> &t, const std::vector<bool> &init)
    {
        timer_.set(t,init);
    }
    void Dwmpc::setDesiredAndParameter(const std::vector<double> &contact0,
                                       const Eigen::MatrixXd &foot_op,
                                       const std::map<std::string,std::vector<double>> initial_condition,
                                       std::map<std::string,std::vector<std::vector<double>>> &ref,
                                       std::map<std::string,std::vector<std::vector<double>>> &param)
    {   
        //bezier courve for the foot trajectory
        std::vector<bezier_curves_t> bcs(n_contact_wb_);
        
        //initialize the value vector of the data in the reference 
        std::vector<std::vector<double>> p;
        std::vector<std::vector<double>> quat;
        std::vector<std::vector<double>> q;
        std::vector<std::vector<double>> dp;
        std::vector<std::vector<double>> omega;
        std::vector<std::vector<double>> dq;
        std::vector<std::vector<double>> tau;
        std::vector<std::vector<double>> grf;
        std::vector<std::vector<double>> foot;
        std::vector<std::vector<double>> dt_vec;

        std::vector<double> p_k;
        std::vector<double> tau_k;
        std::vector<double> grf_k;
        std::vector<double> foot_k;

        //number of leg in contact at this step
        double n_contact{std::accumulate(contact0.begin(),contact0.end(),0.0)};
        std::vector<bool> fix_swing{};

        //initialize the first value of the reference
        // to the initial condition x,y
        p_k = initial_condition.at("p");
        
        //use proprioceptive height
        p_k[2] = proprioHeight(desired_.at("robot_height")[0]);
        
        p.push_back(p_k);

        //set to the desired_ angle
        quat.push_back(desired_.at("quat"));
        // home position quaterion for normalization
        q.push_back(q0_);
        //set to the desired_ speed linear and angular
        dp.push_back(desired_.at("dp"));
        omega.push_back(desired_.at("omega"));
        //set to the desired_ joint angle to 0
        dq.push_back(std::vector<double>(n_joint_wb_,0));

        //set the desired_ foot position to the initial foot position
        for(int idx{0};idx<n_contact_wb_;idx++)
        {
            foot_k.push_back(foot_op(0,idx));
            foot_k.push_back(foot_op(1,idx));
            foot_k.push_back(foot_op(2,idx));
            // foot_k.push_back(foot0_[0+3*idx]);
            // foot_k.push_back(foot0_[1+3*idx]);
            // foot_k.push_back(-0.33);
            
            fix_swing.push_back(contact0[idx] < 1);
        }
        // foot.push_back(foot_k);

        //set the desired_ torque to 0

        for(int idx{0};idx<n_joint_wb_;idx++)
        {
            tau_k.push_back(0);
        }
        tau.push_back(std::vector<double>(n_joint_wb_,0));

        //set the desired_ ground reaction force to the grf for gravity compensation

        for(int idx{0};idx<n_contact_wb_;idx++)
        {
            grf_k.push_back(0);
            grf_k.push_back(0);
            grf_k.push_back(220/std::max(1.0,n_contact)*contact0[idx]); //TODO change this to a more general model 220 is the weight of aliengo
        }
        grf.push_back(grf_k);

        //contact sequence
        std::vector<std::vector<double>> contact_seq;
        //current contact state
        std::vector<double> contact;
        //set the initial contact state
        for(int idx{0};idx<n_contact_wb_;idx++)
        {
            contact.push_back(contact0[idx]);
        }
        contact_seq.push_back(contact);

        std::vector<double> dt{0.01}; //integrazion time step
        std::vector<double> t_leg;
        std::vector<bool> val;
        timer_.get(t_leg,val);

        for(int k {0}; k < N_ + 1; k++)
        {   
            //set dt
            // if(k<2)
            // {
            //     dt[0] = 0.01;
            // }
            // else
            // {
            //     dt[0] = 0.03;
            // }    
            dt_vec.push_back(dt);
            // update the contact state base on the timer
            contact = timer_.run(dt[0]); // at time k+1
            //append the contact state to the contact sequence
            contact_seq.push_back(contact);

            //number of leg in contact at this step
            n_contact = 0;
            for (int i = 0; i < contact.size(); i++) {
                n_contact += contact[i]; 
            }         

            //integrate using the desired_ speed 
            // p
            p_k[0] = ((p[k][0] + desired_.at("dp")[0]*dt[0]));
            p_k[1] = ((p[k][1] + desired_.at("dp")[1]*dt[0]));
            p_k[2] = ((p[k][2] + desired_.at("dp")[2]*dt[0]));
            p.push_back(p_k);

            // quat
            quat.push_back(desired_.at("quat"));
            // q
            q.push_back(q0_);
            // dp
            dp.push_back(desired_.at("dp"));
            // omega
            omega.push_back(desired_.at("omega"));
            // dq
            dq.push_back(std::vector<double>(n_joint_wb_,0));

            if (k < N_)
            // foot
            {
                for (int leg{0};leg < n_contact_wb_;leg++)
                {    
                    
                    // if((contact_seq[k+1][leg] == 0 && contact_seq[k][leg]) > 0 || (contact_seq[k][leg] == 0  && k == 0)) // lift off or already on swing
                    if((contact_seq[k+1][leg] == 0 && contact_seq[k][leg]) > 0) // lift off or already on swing
                    {   
                        //get the yaw from the quaternion
                        double yaw = std::atan2(2*(desired_.at("quat")[3]*desired_.at("quat")[2] + desired_.at("quat")[0]*desired_.at("quat")[1]), 1 - 2*(desired_.at("quat")[1]*desired_.at("quat")[1] + desired_.at("quat")[2]*desired_.at("quat")[2]));
                        std::vector<double> foothold{cos(yaw)*foot0_[3*leg]-sin(yaw)*foot0_[3*leg+1],cos(yaw)*foot0_[3*leg+1] + sin(yaw)*foot0_[3*leg],terrain_height_[leg]-p_k[2]};
                        
                        // foothold[0] += p_k[0];
                        // foothold[1] += p_k[1];

                        foothold[0] += 0.5*(desired_.at("dp")[0] + desired_.at("omega")[2]*(cos(yaw)*foot0_[3*leg+1] + sin(yaw)*foot0_[3*leg]))*timer_.duty_factor*timer_.step_freq;
                        foothold[1] += 0.5*(desired_.at("dp")[1] + desired_.at("omega")[2]*(cos(yaw)*foot0_[3*leg]-sin(yaw)*foot0_[3*leg+1]))*timer_.duty_factor*timer_.step_freq;
                        
                        std::vector<Eigen::Vector3d> cp{};
                        bezier_curves_t::curve_constraints_t constraints;
                        double scaling_factor{0.7105};
                    
                        // if((t_leg[leg]-timer_.duty_factor)/(1-timer_.duty_factor)<0.5) // maka a bezier curve to the foothold with an apex in the mindle
                        // {     
                            cp.push_back(Eigen::Vector3d(foot_k[3*leg]-p_k[0],foot_k[1+3*leg]-p_k[1],foot_k[2+3*leg]-p_k[2]));
                            cp.push_back(Eigen::Vector3d(foot_k[3*leg]-p_k[0],foot_k[1+3*leg]-p_k[1],foot_k[2+3*leg]-p_k[2]+desired_.at("step_height")[0]/scaling_factor));
                            cp.push_back(Eigen::Vector3d((foot_k[3*leg]+foothold[0]-p_k[0])/2,(foot_k[1+3*leg]-p_k[1]+foothold[1])/2,(foot_k[2+3*leg]+foothold[2]-p_k[2])/2+desired_.at("step_height")[0]/scaling_factor));
                            cp.push_back(Eigen::Vector3d(foothold[0],foothold[1],foothold[2] + desired_.at("step_height")[0]/scaling_factor));
                            cp.push_back(Eigen::Vector3d(foothold[0],foothold[1],foothold[2]));
                            constraints.end_vel = Eigen::Vector3d(0,0,0);
                        // }
                        // else //if we passed the midle time of the step make a bezier curve that goes to the end
                        // {   
                        //     cp.push_back(Eigen::Vector3d(foot_k[3*leg],foot_k[1+3*leg],foot_k[2+3*leg]));
                        //     cp.push_back(Eigen::Vector3d(foothold[0],foothold[1],foot_k[2+3*leg]));
                        //     cp.push_back(Eigen::Vector3d(foothold[0],foothold[1],foothold[2]));
                        //     constraints.end_vel = Eigen::Vector3d(0,0,0);
                        // }
                        bezier_curves_t bc(cp.begin(), cp.end(),constraints,(t_leg[leg]-timer_.duty_factor)/(1-timer_.duty_factor),1);
                        bcs[leg] = bc;
                        if(k <= 1)
                        {
                            bcs_[leg] = bc;
                        }

                    }
                    if (contact_seq[k][leg]==0) // if the leg is in swing
                    {   
                        double _t = t_leg[leg]; 
                        if (_t < timer_.duty_factor)
                        {
                            _t = 0.99;
                        }
                        if(fix_swing[leg])
                        {
                            Eigen::Vector3d foot_position{bcs_[leg](std::min((_t-timer_.duty_factor)/(1-timer_.duty_factor),1.0))};
                            foot_k[3*leg] = foot_position[0] + p_k[0];
                            foot_k[3*leg + 1] = foot_position[1] + p_k[1];
                            foot_k[3*leg + 2] =  foot_position[2] + p_k[2];
                        }
                        else
                        {
                            Eigen::Vector3d foot_position{bcs[leg](std::min((_t-timer_.duty_factor)/(1-timer_.duty_factor),1.0))};
                            foot_k[3*leg] = foot_position[0]+ p_k[0];
                            foot_k[3*leg + 1] = foot_position[1] + p_k[1];
                            foot_k[3*leg + 2] =  foot_position[2] + p_k[2];
                        }

                    }
                    else
                    {
                        foot_k[3*leg + 2] = terrain_height_[leg];
                        if(fix_swing[leg])
                        {
                            fix_swing[leg] = false;
                        }
                    }
                }
                foot.push_back(foot_k);
                // tau
                tau.push_back(std::vector<double>(n_joint_wb_,0));
                // grf
                for(int leg{0};leg<n_contact_wb_;leg++)
                {
                    grf_k[3*leg] = 0;
                    grf_k[3*leg+1] = 0;
                    grf_k[3*leg+2] = 220/std::max(1.0,n_contact)*contact[leg]; //TODO change this to a more general model 220 is the weight of aliengo
                }
                grf.push_back(grf_k);
            }
            else
            {
                // push back the last foot ref 
                foot.push_back(foot_k);
            }
            timer_.get(t_leg,val);
        }

        // set the reference map
        ref["p"] = p;
        ref["quat"] = quat;
        ref["q"] = q;
        ref["dp"] = dp;
        ref["omega"] = omega;
        ref["dq"] = dq;
        ref["tau"] = tau;
        ref["grf"] = grf;
        ref["foot"] = foot;
        // set the parameter map
        param["contact_seq"] = contact_seq;
        param["foot"] = foot;
        param["dt"] = dt_vec;
    }
    void Dwmpc::sineWave(std::map<std::string,std::vector<std::vector<double>>> &ref, std::map<std::string,std::vector<std::vector<double>>> &param)
    {
        double time{time_};
        for(int k{0};k<N_+1;k++)
        {    
            time += param["dt"][k][0];
            ref["p"][k][2] = ref["p"][k][2] + amplitude_*sin(time * 2*M_PI*frequency_);
            ref["dp"][k][2] = 2*M_PI*frequency_ * amplitude_*cos(time * 2*M_PI*frequency_);
        }
    }
    void Dwmpc::setSineParam(double frequency, double amplitude)
    {
        frequency_ = frequency;
        amplitude_ = amplitude;
    }
    void Dwmpc::startSineWave()
    {
        do_sine_wave_ = true;
    }
    void Dwmpc::stopSineWave()
    {
        do_sine_wave_ = false;
    }
    void Dwmpc::reset()
    {
        do_init_ = true;
    }
    void Dwmpc::startWalking()
    {
        timer_.startTimer();
    }
    void Dwmpc::stopWalking()
    {
        timer_.stopTimer();

    }
    void Dwmpc::setGaitParam(const double duty_factor,const double step_freq, const int gait_type)
    {   
        std::vector<double> delta;
        switch (gait_type)
        {
            case 0: //trot
                delta.push_back(0.5);
                delta.push_back(0.0);
                delta.push_back(0.0);
                delta.push_back(0.5);
                std::cout << "Trot gait selected" << std::endl;
                break;
            case 1: //crawl
                delta.push_back(0.25);
                delta.push_back(0.75);
                delta.push_back(0.98);
                delta.push_back(0.5);
                std::cout << "Crawl gait selected" << std::endl;
                break;
            case 2: //pace
                delta.push_back(0.0);
                delta.push_back(0.5);
                delta.push_back(0.0);
                delta.push_back(0.5);
                std::cout << "Pace gait selected" << std::endl;
                break;
            case 3: //jump
                delta.push_back(0.5);
                delta.push_back(0.5);
                delta.push_back(0.5);
                delta.push_back(0.5);
                std::cout << "Jump gait selected" << std::endl;
                break;
            default:
                delta.push_back(0.5);
                delta.push_back(0.0);
                delta.push_back(0.0);
                delta.push_back(0.5);
                std::cout << "Gait type not recognized, trot gait selected" << std::endl;
                break;
        }
        timer_.setDelta(delta);
        timer_.setParam(duty_factor,step_freq);
    }
    void Dwmpc::getFullPrediction(std::map<std::string,pdata> &prediction)
    {
        ocp_.getData(prediction);   
        //reorder q and dq
        // for(int k{0};k<N_+1;k++)
        // {
            // reorder_joints(prediction["wb"].q[k],false);
            // reorder_joints(prediction["wb"].dq[k],false);
        // }      
    }
    void Dwmpc::prepare()
    {
        ocp_.prepare();
    }
    void Dwmpc::goHandStand()
    {
        go_biped_ = true;
    }
    void Dwmpc::stopHandStand()
    {   
        //to do --- THIS NEED THE SAME TREATMENT AS THE GO HAND STAND 
        timer_.stopTimer();
        go_biped_ = false;
        bipedal_walk_ = false;
        stand_up_timer_ = 0.0;
    }
    void Dwmpc::setStepHeight(double step_height)
    {
        desired_["step_height"][0] = step_height;
    }
} //namespace controllers