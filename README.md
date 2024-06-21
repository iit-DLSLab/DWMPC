# Periodic plugin

## Introduction
This project skeleton represents the starting point of anyone that wants to create a C++ plugin to interface a software module with DLS2.

This project has been created with the command

        create_periodic_plugin

that requests you to provide a *Plugin Type* and a *Plugin Name* (which is also the name of the module).

The project structure is made by the following main folders:
* module: where you develop your module
* plugin/core: where you develop the plugin
* plugin/console_commands: where you develop your console commands
* plugin/messages: where you develop custom messages
* plugin/topics: where you develop custom topics

The Plugin, module and console commands classes are automatically created, together with some suggestions on how to customize your plugin.

Notice that the include directories has a subdirectory following this convention

        include/<plugin_type>/<plugin_name>

where *\<plugin_name>* is the name of the plugin, and *\<plugin_type>* is the plugin type that can be: *hardwares, estimators, controllers, motion_generators*. This include structure helps with the inclusion of the headers in other libraries.

Don't worry, you are lucky: we will guide you step-by-step with the customization of your plugin.

## Build and install the project
THe first step is to install dls2_deploy
* cd dls2_deploy
* mkdir build
* cd build
* ccmake ..
* make
* sudo make install

With the `ccmake ..` command you can choose what to install.

Once dls2_deploy is installed you can build and install the project

* cd <path_to_your_project>
* mkdir build
* cd build
* ccmake ..
* make
* sudo make install

As for dls2_deploy, with the `ccmake ..` command you can choose what to build and install:
* the software module (*<plugin_name>_module*)
* the plugin (*<plugin_name>_plugin/core*)
* the console commands library (*<plugin_name>_plugin/console_comm*)
* the messages library (*<plugin_name>_plugin/messages*)
* the topics library (*<plugin_name>_plugin/topics*)

In this way, you can build and install separately each software part.

With the build and install steps, you have created the following libraries:
* *<plugin_name>*: plugin library
* *<plugin_name>_module*: module library
* *<plugin_name>_console_commands*: console commands library
* *<plugin_name>_msgs*: messages library
* *<plugin_name>_msgs_wrappers*: messages wrappers library
* *<plugin_name>_topics*: topics library

For example, when creating a *stance_detection* plugin, you will have:
* *stance_detection*
* *stance_detection_module*
* *stance_detection_console_commands*
* *stance_detection_msgs*
* *stance_detection_msgs_wrappers*
* *stance_detection_topics*

Notice that the installation is a necessary step in order to load at run-time your plugin.

## Hands-on
In this section we will see how to customize the project. In the project files, there are comments that suggest you what to do. Some of them are straightforward. For others instead, it is provided an example to clarify what to do. Most of the examples consider that an user has created a plugin for a stance detection module. You will see that the procedure is longer in the explanation, but it is easy and fast in the implementation.

### Outermost CMakeLists.txt
Let's start with the outermost CMakeLists.txt.

You can change the name of the plugin here

      # set configuration variables
      set(PLUGIN_NAME <plugin_name>)

  This will be the name of the plugin library too.  Notice that the names of the other libraries are extracted from the plugin name.

### Create the software module
In the module folder you can customize your software module.
####  module/CMakeLists.txt
In the module/CMakeLists.txt the module library is created. Here you can add other source files, include directories and libraries to be linked.

Notice that to include in the project external libraries to be compiled with yours, we can follow the convetion that they are put in the *module/third-party* folder. For example, if ndcurves is an external library and you want to compile it each time, you will have 

      target_include_directories(${MODULE_LIBRARY_NAME}
            PUBLIC
                  include
            # add other include directories here
                  third-party/ndcurves/include
      )

#### <module_name>.hpp and <module_name>.cpp
As we said, the module class has been already created. However, you can change the class according to your needs. In particular, in the <module_name>.hpp you have to:

* add the arguments of the module constuctor. For example

      StanceDetection(/*aguments_of_module_constructor*/);
  becames

      StanceDetection(std::shared_ptr<robotlib::RobotBase> pRobot);
  In this particular example, remember to add *#include "robotlib/robot_factory.hpp"* and to link the *robotlib* library to the module library.
* add the inputs and outputs to the run function. For example

      void run(/*input_arguments, output_arguments*/);
  becames
  
      void run(robotlib::LegDataMap<bool> &stance_sensors_status, const Eigen::Matrix3d& w_R_b, const robotlib::JointState& q, const robotlib::JointState& qd, const robotlib::JointState& qdd, const robotlib::JointState& tau);
  Rember that the order of the arguments is: *inputs* THEN *outputs*. Writing the run arguments in this way allows your module to be independent from the DLS2 messages and messages' wrappers.

* In the class it is also defined a YAML::Node, that it is used to read the module/config/config.yaml file. In this file you can add configurations for your modules, for example

      aliengo_th:
            foot_LF_contact_force_th : 5
            foot_RF_contact_force_th : 5
            foot_LH_contact_force_th : 5
            foot_RH_contact_force_th : 5
* when choosing what to build and install with the *ccmake ..* command, set to *ON* the *<plugin_name>_module* option

The changes of the class constructor and run function signatures has to be imported in the module/src/<module_name>.cpp as well. In this file, you then have to provide the implementation of the run function. Remember that this is the function storing the "module logic", and it the one called periodically.
### Create the plugin
In the plugin folder you can define the inputs and outputs of your plugin.

#### plugin/core
Here you define the plugin. The plugin stores internally an instance of both the module and console commands classes. To instantiate the module instance, you need to change the plugin constructor, adding the arguments of the module constructor. For example

      StanceDetectionPlugin (
            std::string &ID
            /*, aguments_of_module_constructor*/);
becames


      StanceDetectionPlugin(std::string& ID, const std::shared_ptr<robotlib::RobotBase> robot);
                
This change has to be done as well in the plugin.cpp, by passing such arguments to the module constructor. For example

      StanceDetectionPlugin::StanceDetectionPlugin (std::string& ID/*, aguments_of_module_constructor*/) 
    : dls::PeriodicAppPlugin(ID)
    , stance_detection(/*aguments_of_module_constructor*/) // instantiate module
becames
    
    StanceDetectionPlugin::StanceDetectionPlugin (std::string& ID, const std::shared_ptr<robotlib::RobotBase> robot) 
    : dls::PeriodicAppPlugin(ID)
    , stance_detection(robot) // instantiate module

The console commands instance is automatically instantiated.

Notice that you need to change the *PeriodicAppPlugin \*create(const std::string& ID, const std::string& robot_name)* function too, according to the constructor arguments. This function is called when the plugin is loaded at run-time, and it is responsible for the creation of a plugin instance, by calling the plugin constructor. For example, if the plugin takes as input a robotlib::RobotBase argument

      /*call_plugin_constructor*/
        return new StanceDetectionPlugin(ID/*, aguments_of_module_constructor*/);
  becames

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
      
      return new StanceDetectionPlugin(ID, pRobot);

Now, you have to define the inputs and the outputs. This is done in three steps:

* define member variables storing inputs and outputs
* define inputs and outputs topics
* build inputs and outputs, i.e., data reader and data writer, according to the topics

To define input/output variables:
* includes the headers of your inputs and outputs in plugin.hpp. For example

      //#include <dls2/msg_wrappers/msg_wrapper_name.hpp> // off-the-shelf wrapper
      //#include "estimators/stance_detection/msg_wrapper_name.hpp" // custom wrapper
  becames

      #include <dls2/msg_wrappers/blind_state.hpp> // input, off-the-shelf
      #include <dls2/msg_wrappers/base_state.hpp> //input, off-the-shelf
      #include "estimators/stance_detection/stance_status.hpp" //output, custom
  As you can see, the types of such variables correspond to a message wrapper. You can see also how to include either already provided messages, or custom ones. [Here](#create-custom-messages) you can see how to create a custom message.
* declare the variables in plugin.hpp. For example

      /*define_inputs*/
      /*define_output*/
  becames

      BlindState blind_state; //input
      BaseState base_state; //input
      StanceStatus stance_status; //output
* initialize the variables in plugin.cpp. For example

      /*, construct_input_variables*/ // instantiate input
      /*, construct_output_variables*/ //instantiate output

  becames

      , blind_state(robot)
      , base_state(robot)
      , stance_status(robot)
  Notice here that we are using the robot variable to create the wrappers. If your module does not need a robot object to be created, you still have to add the robot object to the plugin constructor, to be passed to the wrappers constructor. At the same time, there might be wrappers that does not need a robot object in their constructor.

To define the topics you can include them in the plugin.cpp. If you use already existing topics, decomment the follwing line

      //#include <dls2/topics/topics.hpp> // off-the-shelf topics
and in the plugin/core/CMakeLists.txt, add in the target_link_libraries command, under PUBLIC, the library *dls_topics*.

If you use instead custom topics, you have to include your topics.hpp file and change the plugin/core/CMakeLists.txt. For example, you need to decomment

      //#include "estimators/stance_detection/topics.hpp" // custom topics
and in the plugin/core/CMakeLists.txt also decomment 
      
      #${TOPICS_LIBRARY_NAME}
You can of course have both custom and off-the-shelf topics.

You can now build the inputs and outputs. For example

      // Define inputs
        /*this->buildInput<message_wrapper_class>(
            topic_name,
            &input_variable_name
        );*/

        // Define outputs
        /*this->buildOutput<message_wrapper_class>(
            topic_name,
            &output_variable_name
        );*/
becames
        
        // Define inputs
        this->buildInput<BlindState>(
            dls::topics::low_level_estimation::blind_state,
            &blind_state
        );
        this->buildInput<BaseState>(
            dls::topics::high_level_estimation::base_state,
            &base_state
        );
        // Define outputs
        this->buildInput<StanceStatus>(
            topics::stance_detection::stance_status,
            &stance_status
        );

With this functions, we created the data readers and data writers of the plugin, associated to off-the-shelf and custom topics. Moreover, since we are passing the reference to our input/output variables when building the inputs/outputs:
 * the read() function automatically updates all the inputs
 * the write() function automatically take the output variables and publish them. It also automatically fills the timestamp, if the variable has one; so you do not need to bother about setting it

There are two last steps to be done:
* in the run function of the plugin, add the correct inputs and outputs to the run function of the module. For example 

      stance_detection.run(/*input_arguments, output_arguments*/);

  becames (according to the run function we have defined in the previous example)

      stance_detection.run(   blind_state.feet_contact_,
                              base_state.pose_.toRotationMatrix.transpose(),
                              blind_state.joints_position_,
                              blind_state.joints_velocity_,
                              blind_state.joints_acceleration_,
                              blind_state.joints_effort_
                              stance_status.stance_status_);

* when choosing what to build and install with the *ccmake ..* command, set to *ON* the *<plugin_name>_plugin/core* and *<plugin_name>_plugin/console_comm* options. If you are using custom messages and/or custom topics set to *ON*, respectively, *<plugin_name>_plugin/messages* and *<plugin_name>_plugin/topics* options.

Congratulations! You have created your fist periodic plugin for dls2!

#### How to set the scheduler properties
In the plugin/config folder you have the possibility to set the properties of the scheduler.

### Create custom console commands
Creating console commmands in plugin/console_commands is quite easy. The console commands are functions, called from the console, that change the status of your running module. Since the console is implemented as a DLS2 layer, to keep the module independent from how to interact with the console, a separated console function class is defined that links the plugin with the module. This is done by
* defining functions, to be used in the console, that changes the status of the running module
* adding such functions to the command manager of the plugin as console commands

To do that:
* declare the console functions in console_functions.hpp. For example

      /* bool function_name();*/
  becames

      bool setStanceDetectionMethod();

* implement the console functions in console_functions.cpp. For example

      /*
	      bool StanceDetectionConsoleCommands::function_name()
      {
        // Define function here, using ptr to access to module object's state
        
        return true;
      }
      */

  becames

      bool StanceDetectionConsoleCommands::setStanceDetectionMethod()
      {
        std::cout   << "Stance detection methods:\n"
                    <<  "Use sensor data: " << StanceDetectionMethod::use_sensor_data << "\n"
                    <<  "Use estimated grf: " << StanceDetectionMethod::use_estimated_grf << "\n";
        int stance_method{-1};
        if(CommandHelper::readValue<int>("Stance method", stance_method, ptr->stance_detection_method_))
        {
            ptr_->stance_detection_method_ = stance_method;
        }
      }

      return true;

  As you can see from this example, the state of the module instance is changed through the ptr pointer. Moreover, to get the data from the command line, you can use the *CommandHelper::readValue<value_type>* function. This function takes as inputs:
  
  * a comment to be displayed
  * the variable to be filled with the command line value; this variable is of *value_type* type which has to be equal to the command line value type
  * an optional value corresponding to the current value that you want to change. If it is provided, it is displayed; otherwise it is not

  It returns true if the console input is not empty: in this case you can then update the ptr state.

* add the console functions to the command manager. For example

        /*command_manager_ptr->addCommand(  "command_name",
                                            "comment_of_the_command",
                                            &StanceDetectionConsoleCommands::function_name, this, {}, true);
        */
  becames
      
      // Add console commands
      command_manager_ptr->addCommand("setStanceDetectionMethod",
                                        "Set stance detector method",
                                        &StanceDetectionConsoleCommands::setStanceDetectionMethod, this, {}, true);
  The *command_manager_ptr* is a pointer to the command manager object of the plugin creating an instance of the console commands class. See [here](https://gitlab.advr.iit.it/dls-lab/dls2/-/tree/clear_inputs_outputs/modules%2Fcommand#how-to-define-a-command) for how to create a command.
  
* when choosing what to build and install with the *ccmake ..* command, set to *ON* the *<plugin_name>_plugin/console_comm* option

### Create custom messages
In plugin/messages you can define custom messages. To create a message:
* add its idl file in plugin/messages/idls. For example, the *message.idl* file can be renamed to *stance_status.idl* and

      struct <MessageName>Msg{}
  becames

      struct StanceStatusMsg
      {
            // Header
            string frame_id;
            unsigned long sequence_id;
            double timestamp;

            // Stance status
            double stance_status[4];
      };
  By convetion, please ends the struct name with *Msg*.
* create a message wrapper
      
      TODO
* In plugin/messages/CMakeLists.txt, call the function *dls_add_message* with the message idl file name as argument. For example,

      dls_add_message(message) 	# generate message
  becames
 
      dls_add_message(stance_status) 	# generate message

* when choosing what to build and install with the *ccmake ..* command, set to *ON* the *<plugin_name>_plugin/messages* option

So far, you have create a library for custom messages. To use the messages library in the custom topics, in plugin/topics/CMakeLists.txt uncomment the following lines

      #${MSGS_LIBRARY_NAME}
and 
      
      #${CMAKE_CURRENT_BINARY_DIR}/../messages/include

To link instead the library of the custom messages' wrappers to your plugin, in plugin/core/CMakeLists.txt uncomment the following line

      #${MSGS_WRAPPERS_LIBRARY_NAME}

### Create custom topics
In plugin/topics you have the possiblity to create custom topics. For each topic you have to define a topic name and a topic message. If at least one of the custom topic is defined using off-the-shelf message, decomment 

      #dls_messages

 in plugin/topics/CMakeLists.txt. If instead custom messages are used, decomment in the same file 
      
      #${CMAKE_CURRENT_BINARY_DIR}/../messages/include
and 

      ${MSGS_LIBRARY_NAME}
which is the library of the custom message. Both of them can be uncommented if you have a mixed configuration.

To create a topic
* in topics.hpp, declare the topic. For example

      //extern dls::topicType topic_variable_name;
  becames

      extern dls::topicType stance_status;
* in topics.cpp, include the [TypeSupport](https://fast-dds.docs.eprosima.com/en/latest/fastdds/dds_layer/topic/typeSupport/typeSupport.html?highlight=TopicDataType#definition-of-data-types) of each topic. Thanks to [Fast DDS-Gen](https://fast-dds.docs.eprosima.com/en/latest/fastdds/dds_layer/topic/fastddsgen/fastddsgen.html#fast-dds-gen-for-data-types-source-code-generation), the TypeSupport of each message is automatically created from the corresponding idl file, when building the project. For example, if you have a custom idl file in plugin/messages/idls called stance_status.idl, you have that

      // Include the TypeSupport of each message associated to each topic
      //#include <dls_messages/dds/<idl_file_name>PubSubTypes.h> // # off-the-shelf message
      //#include "dls_messages/dds/<idl_file_name>PubSubTypes.h" // # custom message

  becames

      #include "dls_messages/dds/stance_statusPubSubTypes.h"

  If your module instead needs to create another topic from a off-the-shelf message, you can include it, for example, in this way

      #include "dls_messages/dds/control_signalPubSubTypes.h"
  where *control_signal* is the name of the idl file of the control signal message.
* in topics.cpp, define the topic. For example

      //dls::topicType topic_variable_name = dls::topicType("topic_name", new <message_name>PubSubType());
  becames

      dls::topicType stance_status = dls::topicType("stance_status", new StanceStatusMsgPubSubType());
* when choosing what to build and install with the *ccmake ..* command, set to *ON* the *<plugin_name>_plugin/topics* option. Remember that if at least one of your topics is using custom messages, set to *ON* the *<plugin_name>_plugin/messages* option too.

So far you have created the topic. In order to link the topic library to the plugin, in plugin/core/CMakeLists.txt decomment the following line

      #${TOPICS_LIBRARY_NAME}
Remember that if at least one of your topics is using custom messages,

      #${MSGS_WRAPPERS_LIBRARY_NAME}

should be uncommented too.

Notice that if you are creating a plugin of *controllers* type, to make the control layer using the output of the controller, the output topic should have the same name of the plugin library name. 
#### How to include a custom topic in an external project
To include the set of custom topics of your plugin in another external project
* include the topics.hpp file in this way
      #include "dls2/<plugin_type>/<plugin_name>/topics.hpp"
  where <plugin_type> is the type of plugin  and <plugin_name> is its name. For example

      #include "dls2/estimators/stance_detection/topics.hpp"
* link the topics library name to the external project, adding the library

      <topics_library_name>
  to target_link_library (under PUBLIC keyword), where <topics_library_name> is the name of the topics library. For example

      stance_detection_topics
### Customize debian packaging
At the end of the outermost CMakeLists.txt, add the debian dependencies here

      set(CPACK_DEBIAN_PERIODIC_PACKAGE_DEPENDS       "dls2-runtime" CACHE INTERNAL "") # add here package dependencies

and here

      set(CPACK_DEBIAN_PERIODIC_DEV_PACKAGE_DEPENDS   "dls2-dev, dls-${PLUGIN_NAME}" CACHE INTERNAL "") # add here package dependencies
For example you can have

      set(CPACK_DEBIAN_PERIODIC_PACKAGE_DEPENDS       "dls2-runtime, dls-state-estimator" CACHE INTERNAL "")
and

      set(CPACK_DEBIAN_PERIODIC_DEV_PACKAGE_DEPENDS   "dls2-dev, dls-${PLUGIN_NAME}, dls-stance-detection, dls-state-estimator-dev" CACHE INTERNAL "")