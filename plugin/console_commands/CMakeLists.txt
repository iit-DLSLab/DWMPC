# Create the library for the console functions of the module
add_library(${CONSOLE_COMMANDS_LIBRARY_NAME}
	SHARED
	src/console_commands.cpp
)
target_include_directories(${CONSOLE_COMMANDS_LIBRARY_NAME}
PUBLIC
	include
)
target_link_directories(${CONSOLE_COMMANDS_LIBRARY_NAME}
    PRIVATE
        /usr/lib/dls2
)
target_link_libraries(${CONSOLE_COMMANDS_LIBRARY_NAME}
	PUBLIC
		dls_command
		${MODULE_LIBRARY_NAME}
)

# ================================== Install =================================
install(
	DIRECTORY   include/
	DESTINATION /usr/include/dls2
	COMPONENT   ${CONSOLE_COMMANDS_LIBRARY_NAME}_dev
)
install(
	TARGETS             ${CONSOLE_COMMANDS_LIBRARY_NAME}
	LIBRARY DESTINATION /usr/lib/dls2/controllers/${PLUGIN_NAME}
	COMPONENT           ${CONSOLE_COMMANDS_LIBRARY_NAME}
)
