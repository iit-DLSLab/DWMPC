#ifndef DWMPC_CONSOLE_COMMANDS_HPP
#define DWMPC_CONSOLE_COMMANDS_HPP

// module header
#include "controllers/dwmpc/dwmpc.hpp"

// command manager
#include "dls2/command/command_manager.hpp"

namespace controllers
{
	class DwmpcConsoleCommands
	{
	public:
		DwmpcConsoleCommands
		(
			Dwmpc*  const ptr, // module object pointer
			dls::CommandManager*  const command_manager_ptr //command manager ptr, when console functions are registered
		);

		~DwmpcConsoleCommands();

	private:
		//! Module object pointer
		Dwmpc * const ptr;

		// Console functions
		/* bool function_name();*/
		bool startWalking();
		bool stopWalking();
		bool setGaitParameter();
		bool startSine();
		bool stopSine();
		bool setSineParameter();
		bool setWeight();
		bool goHandStand();
		bool stopHandStand();
		bool setStepHeight();
	};
}
#endif /* end of include guard: DWMPC_CONSOLE_COMMANDS_HPP */
