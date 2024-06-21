#ifndef DWMPC_PLUGIN_HPP
#define DWMPC_PLUGIN_HPP

// periodic plugin header
#include <dls2/plugin/periodic_app_plugin.hpp>
#include "controllers/dwmpc/dwmpc.hpp"
#include "controllers/dwmpc/console_commands.hpp"
// wrappers
//#include <dls2/msg_wrappers/msg_wrapper_name.hpp> // off-the-shelf wrapper
#include <dls2/msg_wrappers/trajectory_generator.hpp> // off-the-shelf wrapper
#include <dls2/msg_wrappers/blind_state.hpp> // off-the-shelf wrapper
#include <dls2/msg_wrappers/base_state.hpp> // off-the-shelf wrapper
#include <dls2/msg_wrappers/control_signal.hpp> // off-the-shelf wrapper
#include <dls2/msg_wrappers/controller_command.hpp> // off-the-shelf wrapper
//#include "controllers/dwmpc/msg_wrapper_name.hpp" // custom wrapper
#include "motion_generators/vis/vis_data.hpp"


namespace controllers
{
    class DwmpcPlugin : public dls::PeriodicAppPlugin
    {
    public:
        DwmpcPlugin (const std::string& ID, const std::shared_ptr<robotlib::RobotBase> pRobot);

        ~DwmpcPlugin();

        void run(const std::chrono::system_clock::time_point &time) override;
        
        bool deactivation(const std::chrono::system_clock::time_point& time);

        std::string where() override;

        AppStatus eStop() override { return getStatus(); }

    private:
        Dwmpc dwmpc;
        DwmpcConsoleCommands console_commands;
        std::shared_ptr<robotlib::RobotBase> pRobot_;
        bool checkActivation();
        
        /*define_inputs*/
        dls::BlindState input_blind_state;
		dls::BaseState input_base_state;
        dls::ControllerCommand input_ctrl_comm;
        /*define_output*/
        dls::ControlSignal output_tau;
        dls::TrajectoryGenerator output_traj_gen;
        dls::VisMessage output_vis_message;

        
    };
} // namespace controllers

#endif // end of include guard: DWMPC_PLUGIN_HPP
