#include "controllers/dwmpc/console_commands.hpp"

#include <sstream>

namespace controllers
{
	DwmpcConsoleCommands::DwmpcConsoleCommands (Dwmpc* const ptr, dls::CommandManager* const command_manager_ptr) :ptr(ptr)
	{
        // Add console commands
        /*command_manager_ptr->addCommand(  "command_name",
                                            "comment_of_the_command",
                                            &DwmpcConsoleCommands::function_name, this, {}, true);
        */
        command_manager_ptr->addCommand(  "startWalking",
                                            "start walking",
                                            &DwmpcConsoleCommands::startWalking, this, {}, true);
        command_manager_ptr->addCommand(  "stopWalking",
                                            "stop walking",
                                            &DwmpcConsoleCommands::stopWalking, this, {}, true);
         command_manager_ptr->addCommand(  "setGaitParameter",
                                            "setGaitParameter",
                                            &DwmpcConsoleCommands::setGaitParameter, this, {}, true);
        command_manager_ptr->addCommand(  "startSine",
                                            "start sine",
                                            &DwmpcConsoleCommands::startSine, this, {}, true);
        command_manager_ptr->addCommand(  "stopSine",
                                            "stop sine",
                                            &DwmpcConsoleCommands::stopSine, this, {}, true);
        command_manager_ptr->addCommand(  "setSineParameter",
                                            "setSineParameter",
                                            &DwmpcConsoleCommands::setSineParameter, this, {}, true);
        command_manager_ptr->addCommand(  "setWeight",
                                            "set weight",
                                            &DwmpcConsoleCommands::setWeight, this, {}, true);
    }

	DwmpcConsoleCommands::~DwmpcConsoleCommands()
    {}

    /*
	bool DwmpcConsoleCommands::function_name()
    {
        // Define function here, using ptr to access to module object's state
        
        return true;
    }
    */
    bool DwmpcConsoleCommands::startWalking()
    {
        ptr->startWalking();
        return true;
    }
    bool DwmpcConsoleCommands::stopWalking()
    {
        ptr->stopWalking();
        return true;
    } 
    bool DwmpcConsoleCommands::setGaitParameter()
    {   
        double duty_factor{0.0};
        double step_frequency{0.0};
        int gait_type{0};
        bool result_duty_factor = dls::CommandHelper::readValue<double>("Duty Factor", duty_factor);
        bool result_step_frequency = dls::CommandHelper::readValue<double>("Step Frequency", step_frequency);
        bool result_gait_type = dls::CommandHelper::readValue<int>("Gait Type", gait_type);
        if(result_duty_factor && result_step_frequency && result_gait_type)
        {
            ptr->setGaitParam(duty_factor,step_frequency,gait_type);
        }
        return result_duty_factor && result_step_frequency && result_gait_type;
    } 
    bool DwmpcConsoleCommands::startSine()
    {
        ptr->startSineWave();
        return true;
    }
    bool DwmpcConsoleCommands::stopSine()
    {
        ptr->stopSineWave();
        return true;
    }
    bool DwmpcConsoleCommands::setSineParameter()
    {
        double frequency{0.0};
        double amplitude{0.0};
        bool result_frequency = dls::CommandHelper::readValue<double>("Frequency", frequency);
        bool result_amplitude = dls::CommandHelper::readValue<double>("Amplitude", amplitude);
        if(result_frequency && result_amplitude)
        {
            ptr->setSineParam(frequency,amplitude);
        }
        return result_frequency && result_amplitude;
    }
    bool DwmpcConsoleCommands::setWeight()
    {
        std::map<std::string,std::vector<double>> weight_vec{ptr->getWeight()};
        std::cout << "Weight names: ";
        for(auto &it : weight_vec)
        {
            std::string name = it.first;
            std::cout << name << " -- ";
        }
        std::cout << std::endl;
        std::string name;
        // loop over the names of string in the map
        bool safety_check{true};
        bool result_name = dls::CommandHelper::readValue<std::string>("Name of the weight",name);
        if(result_name)
        {
            try
            {
                std::vector<double> weight = weight_vec.at(name);
                std::cout << "Weight for " << name << std::endl;
                std::cout << "Current value: ";
                for (auto &w : weight)
                {
                    std::cout << w << " ";
                }
                int counter = 0;
                std::cout << std::endl;
                for(auto &w : weight)
                {
                    std::cout << counter << "";
                    bool result = dls::CommandHelper::readValue<double>(" ",w);
                    safety_check = safety_check && result; 
                    counter ++;
                }
                if (safety_check)
                {
                    weight_vec[name] = weight;
                }
            }
            catch(const std::exception& e)
            {
               std::cout << "Weight name not found" << std::endl;
                return false;
            }
             
        }
       
        if(safety_check)
        {
            ptr->setWeight(weight_vec);
        }
        return safety_check;
    }
} //namespace controllers
