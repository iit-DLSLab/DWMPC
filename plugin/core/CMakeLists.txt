# Create plugin library
add_library(${PROJECT_NAME} SHARED
	src/plugin.cpp
)
target_include_directories(${PROJECT_NAME}
	PUBLIC
		include
		# add other include directories here
)
target_link_libraries(${PROJECT_NAME}
	PUBLIC
		periodic_app_plugin
		${MODULE_LIBRARY_NAME}
		${CONSOLE_COMMANDS_LIBRARY_NAME}
		#${MSGS_WRAPPERS_LIBRARY_NAME}
		${TOPICS_LIBRARY_NAME}
		vis_topics
		vis_msgs_wrappers
		# add other linked libraries here
)
target_link_directories(${PROJECT_NAME}
	PRIVATE
        /usr/lib/dls2
		/usr/lib/dls2/controllers/${PLUGIN_NAME}
		/usr/lib/dls2/motion_generators/vis
)

# ================================== Install =================================

install(
	DIRECTORY   include/
	DESTINATION /usr/include/dls2
	COMPONENT   ${PROJECT_NAME}_dev
)

install(
	FILES		config/scheduler.yaml
	DESTINATION /usr/include/dls2/schedulers/${PLUGIN_NAME}
	COMPONENT   ${PROJECT_NAME}
)

install(
	TARGETS             ${PROJECT_NAME}
	LIBRARY DESTINATION /usr/lib/dls2/controllers
	COMPONENT           ${PROJECT_NAME}
)
