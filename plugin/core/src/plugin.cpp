#include "controllers/dwmpc/plugin.hpp"

// topics
#include <dls2/topics/topics.hpp> // off-the-shelf topics
#include "controllers/dwmpc/topics.hpp" // custom topics
#include <robotlib/robot_factory.hpp>

#include "dls2/msg_wrappers/signal_writer.hpp"
#include "motion_generators/vis/topics.hpp"

#include <chrono>

namespace controllers 
{
    DwmpcPlugin::DwmpcPlugin (const std::string& ID, const std::shared_ptr<robotlib::RobotBase> pRobot)
    : dls::PeriodicAppPlugin(ID)
    , dwmpc(/*aguments_of_module_constructor*/) // instantiate module
    , console_commands(&dwmpc, &command_manager)
    /*, construct_input_variables*/ // instantiate input
    , input_blind_state(pRobot)
    , input_base_state(pRobot)
    , input_ctrl_comm(pRobot)
    /*, construct_output_variables*/ //instantiate output
    , output_tau(pRobot)
    , output_traj_gen(pRobot)
    , pRobot_(pRobot)
    {
        // Define inputs
        /*this->buildInput<message_wrapper_class>(
            topic_name,
            &input_variable_name
        );*/
        this->buildInput<dls::BlindState>(
            dls::topics::low_level_estimation::blind_state,
            &input_blind_state
        );
        this->buildInput<dls::BaseState>(
            dls::topics::high_level_estimation::base_state,
            &input_base_state
        );
        this->buildInput<dls::ControllerCommand>(
            dls::topics::controller_signal,
            &input_ctrl_comm,
            [](){},
            false
        );
        // Define outputs
        /*this->buildOutput<message_wrapper_class>(
            topic_name,
            &output_variable_name
        );*/
        this->buildOutput<dls::ControlSignal>(
            dls::topics::dwmpc::tau,
            &output_tau
        );
        this->buildOutput<dls::TrajectoryGenerator>(
            dls::topics::trajectory_generator,
            &output_traj_gen
        );
        this->buildOutput<dls::VisMessage>(
            dls::topics::vis::vis,
            &output_vis_message
        );
    }

    DwmpcPlugin::~DwmpcPlugin()
    { }
    
    std::string DwmpcPlugin::where()
    {
        std::stringstream ss;
        ss << "TODO \n";

        return ss.str();
    }
    
    void DwmpcPlugin::run(const std::chrono::system_clock::time_point& time)
    {  
        // Read inputs
        read();
        
        std::vector<double> des_tau{};
        std::vector<double> des_q{};
        std::vector<double> des_dq{};
        std::vector<double> des_contact{};
        
        //Calculate the foot position 
        auto feet_position = pRobot_->makeLegDataMap<Eigen::Vector3d>(Eigen::Vector3d::Zero());

        Eigen::MatrixXd foot_op = Eigen::MatrixXd::Zero(3, 4);

        pRobot_->forwardKinematics(input_blind_state.joints_position_, feet_position);
        Eigen::Vector3d pos_0{0,0,0};
        Eigen::Quaterniond quat_0{};
        quat_0.coeffs() << 0,0,0,1;
        int i{0};
        Eigen::Vector4d current_contact{};
        for (auto leg : *pRobot_->getLegs())
        {   
            // foot_op.col(i) = input_base_state.pose_.toPosition() + input_base_state.pose_.toRotationMatrix().transpose()*feet_position[leg];             
            // current_contact[i] = input_base_state.stance_status_[leg];
            foot_op.col(i) = feet_position[leg];             
            current_contact[i] = 0;
            i += 1;
        }
        
        output_vis_message.sphere_color_.clear();
        output_vis_message.sphere_position_.clear();
        output_vis_message.sphere_radius_.clear();
        output_vis_message.arrow_position_.clear();
        output_vis_message.arrow_color_.clear();
        output_vis_message.arrow_quat_.clear();
        output_vis_message.arrow_length_.clear();

        // auto start = std::chrono::high_resolution_clock::now();
        // Run module
        dwmpc.run(input_base_state.pose_.toPosition(), //p
                  input_base_state.pose_.toQuaternion(),//quat
                  input_blind_state.joints_position_.vec_(),//q
                  input_base_state.velocity_.getLinear(),//dp
                  input_base_state.velocity_.getAngular(),//omega 
                  input_blind_state.joints_velocity_.vec_(), //dq
                  this->getPeriod().count()*1e-6,// period
                  current_contact,//stance_status
                  foot_op,//feet position
                  input_ctrl_comm.base_velocity_HF_.getLinear(),//desired_linear_speed //TODO change from HF to WF
                  input_ctrl_comm.base_velocity_HF_.getAngular(),//desired_angular_speed
                  input_ctrl_comm.base_pose_HF_.toQuaternion(),//desired_orientation
                  output_vis_message.sphere_position_,
                  output_vis_message.sphere_color_,
                  output_vis_message.sphere_radius_,
                  output_vis_message.arrow_position_,
                  output_vis_message.arrow_color_,
                  output_vis_message.arrow_quat_,
                  output_vis_message.arrow_length_,
                  des_contact,//des_contact
                  des_tau,//des_tau
                  des_q,//des_q
                  des_dq//des_dq
                  );
    //test publish vis message
    //    auto robot_jacobian_tmp = this->pRobot_->makeLegDataMap<Eigen::Matrix3d>(Eigen::Matrix3d::Zero());
    //    auto robot_jacobian = this->pRobot_->makeFeetJacobian();
        // Eigen::MatrixXd robot_jacobian_LF;
        // EIgen::MatrixXd robot_jacobian_RF;
        // Eigen::MatrixXd robot_jacobian_LH;
        // Eigen::MatrixXd robot_jacobian_RH;
        
        // this->pRobot_->computeLimbsJacobian(input_blind_state.joints_position_, "lf_foot", robot_jacobian_LF);
        // this->pRobot_->computeLimbsJacobian(input_blind_state.joints_position_, "rf_foot", robot_jacobian_RF);
        // this->pRobot_->computeLimbsJacobian(input_blind_state.joints_position_, "lh_foot", robot_jacobian_LH);
        // this->pRobot_->computeLimbsJacobian(input_blind_state.joints_position_, "rh_foot", robot_jacobian_RH);

        // robot_jacobian_tmp["LF"] = robot_jacobian_LF.block<3,3>(0,0);
        // robot_jacobian_tmp["RF"] = robot_jacobian["RF"].block<3,3>(0,0);
        // robot_jacobian_tmp["LH"] = robot_jacobian["LH"].block<3,3>(0,0);
        // robot_jacobian_tmp["RH"] = robot_jacobian["RH"].block<3,3>(0,0);

        Eigen::MatrixXd grf_m;
        grf_m.setZero(3, 4); 
        int counter_leg = 0;
        // for(auto leg: *this->pRobot_->getLegs())
        // {
        //     grf_m.col(counter_leg) = -input_base_state.pose_.toRotationMatrix().transpose()*robot_jacobian_tmp[leg].transpose().inverse()*input_blind_state.joints_effort_.vec_(leg);
        //     counter_leg += 1;
        // }

         for (int leg{0};leg < this->pRobot_->getNLEGS();leg++)
        {   
            output_vis_message.arrow_position_.push_back(foot_op.col(leg));
            output_vis_message.arrow_length_.push_back(grf_m.col(leg).norm()/220);
            if(leg == 0)
                {
                    output_vis_message.arrow_color_.push_back({0, 1, 0, 0.5});
                }
                else if (leg == 1)
                {
                    output_vis_message.arrow_color_.push_back({1, 0, 0, 0.5});
                }
                else if (leg == 2)
                {
                    output_vis_message.arrow_color_.push_back({1, 0, 1, 0.5});
                }
                else if (leg == 3)
                {
                    output_vis_message.arrow_color_.push_back({0, 0, 1, 0.5});
                }
            //set the orrientation of the arrow depending on the grf
            Eigen::Vector3d _dir = grf_m.col(leg)/grf_m.col(leg).norm();
            double angle = std::acos(_dir[2]);
            Eigen::Quaterniond _quat(Eigen::AngleAxisd(angle, _dir));
            output_vis_message.arrow_quat_.push_back({_quat.x(),_quat.y(),_quat.z(),_quat.w()});
           
        }
        std::map<std::string,pdata> data;
        dwmpc.getFullPrediction(data);
        auto q_k = pRobot_->makeJointState(0);
        Eigen::Vector3d p_k;
        dls::Pose pose_k;
        auto foot_k = pRobot_->makeLegDataMap<Eigen::Vector3d>(Eigen::Vector3d::Zero()); 
        // input_blind_state.joints_position_.print();
        for(int k{0};k < 15;k++) //TODO legth of the prediction is arcoded
        {   
            q_k = data["wb"].q[k];
            pRobot_->forwardKinematics(q_k, foot_k);
            int counter_leg = 0;
            for (auto leg : *pRobot_->getLegs())
            {   
                if(counter_leg < 2)
                {
                    p_k[0] = data["front"].p[k][0];
                    p_k[1] = data["front"].p[k][1];
                    p_k[2] = data["front"].p[k][2];
                    pose_k.set(Eigen::Quaterniond(data["front"].quat[k][3],data["front"].quat[k][0],data["front"].quat[k][1],data["front"].quat[k][2]));

                   
                }
                else
                {
                    p_k[0] = data["back"].p[k][0];
                    p_k[1] = data["back"].p[k][1];
                    p_k[2] = data["back"].p[k][2];
                    pose_k.set(Eigen::Quaterniond(data["back"].quat[k][3],data["back"].quat[k][0],data["back"].quat[k][1],data["back"].quat[k][2]));
                }

                auto foot_Wk = p_k + pose_k.toRotationMatrix().transpose()*foot_k[leg];
                output_vis_message.sphere_position_.push_back(foot_Wk);
                output_vis_message.sphere_color_.push_back({static_cast<double>((27 - k)) / static_cast<double>(27), 0.25, 0.25,static_cast<double>((27 - k)) / static_cast<double>(2*27)+0.3});
                output_vis_message.sphere_radius_.push_back(0.01);
                counter_leg += 1;
               
            }
        }
        // Set outputs
        output_traj_gen.desired_joints_position_ = des_q;
        output_traj_gen.desired_joints_velocity_ = des_dq;
        counter_leg = 0; 
        for (auto leg : *pRobot_->getLegs())
        {
            output_traj_gen.stance_legs_[leg] = des_contact[counter_leg]>0.5;
            counter_leg += 1;
        }
        output_tau.torques_ = des_tau;
        // Publish outputs
        write();
        dwmpc.prepare();
    }
    bool DwmpcPlugin::checkActivation()
    {
        // If module can be activated, initialize the trunk controller
        if(basicActivationChecks())
        {
            read();
            dwmpc.init();
            return true;
        }
        return false;
    }
    bool DwmpcPlugin::deactivation(const std::chrono::system_clock::time_point& time){
        output_tau.torques_.setZero();
        write();
        return true;
    }
    extern "C" PeriodicAppPlugin *create(const std::string& ID, const std::string& robot_name)
    {   
        if (robot_name == "")
        {
            std::string e = "Parameter robot_name is not defined, verify if the parameter server is running";
            throw std::runtime_error(e);
        }

        std::shared_ptr<robotlib::RobotBase> pRobot;
        try
        {
            pRobot = robotlib::RobotFactory::openRobot(robot_name);
        }
        catch (const std::exception &e)
        {
            std::cerr << "child_process: Could not open the robot " << robot_name << std::endl;
            std::cerr << e.what() << std::endl;
        }
        /*call_plugin_constructor*/
        return new DwmpcPlugin(ID,pRobot);
    }

    extern "C" void destroy(PeriodicAppPlugin *p)
    {
            delete p;
    }
} //namespace controllers
