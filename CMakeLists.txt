cmake_minimum_required(VERSION 3.15)   
# set configuration variables
set(PLUGIN_NAME dwmpc)
set(MODULE_LIBRARY_NAME ${PLUGIN_NAME}_module)
set(CONSOLE_COMMANDS_LIBRARY_NAME ${PLUGIN_NAME}_console_commands)
set(MSGS_LIBRARY_NAME ${PLUGIN_NAME}_msgs)
set(MSGS_WRAPPERS_LIBRARY_NAME ${PLUGIN_NAME}_msgs_wrappers)
set(TOPICS_LIBRARY_NAME ${PLUGIN_NAME}_topics)

project(${PLUGIN_NAME})

add_compile_options("-std=c++2a")

set(CMAKE_CXX_FLAGS "-O3")

# add_compile_options("-std=c++2a")

find_package(Eigen3 3.3 REQUIRED NO_MODULE)
find_package(pybind11 CONFIG REQUIRED)
find_package(Python3 REQUIRED COMPONENTS Development)
add_subdirectory(third_party/acados)

# Create the library for the module
add_library(${MODULE_LIBRARY_NAME} SHARED
	src/${PLUGIN_NAME}.cpp
	src/acadosInterface.cpp
	src/timer.cpp
	src/distributed_solver.cpp
	# add other source files here
)
target_include_directories(${MODULE_LIBRARY_NAME}
	PUBLIC
		include
		${CMAKE_CURRENT_SOURCE_DIR}/c_generated_code
		/opt/openrobots/include
	# add other include directories here
)
target_link_directories(${MODULE_LIBRARY_NAME}
    PUBLIC
		# ${CMAKE_CURRENT_SOURCE_DIR}/c_generated_code
		/opt/openrobots/lib
		/usr/lib/dls2/controllers/${PLUGIN_NAME}
)
target_link_libraries(${MODULE_LIBRARY_NAME}
	PUBLIC
	acados
	${CMAKE_CURRENT_SOURCE_DIR}/c_generated_code/libacados_ocp_solver_front.so
	${CMAKE_CURRENT_SOURCE_DIR}/c_generated_code/libacados_ocp_solver_back.so
	yaml-cpp
	Eigen3::Eigen
	# add other linked libraries here
)
#Add test.cpp as an executable

pybind11_add_module(pydwmpc src/py_dwmpc.cpp)
target_link_libraries(pydwmpc
	PUBLIC
	${MODULE_LIBRARY_NAME}
)

# ================================== Install =================================
install(
	DIRECTORY   include/
	DESTINATION /usr/include/dls2
	COMPONENT   ${MODULE_LIBRARY_NAME}_dev
)

install(
	DIRECTORY   config
	DESTINATION /usr/include/dls2/controllers/${PLUGIN_NAME}
	COMPONENT   ${MODULE_LIBRARY_NAME}
)

install(
	TARGETS             ${MODULE_LIBRARY_NAME}
	LIBRARY DESTINATION /usr/lib/dls2/controllers/${PLUGIN_NAME}
	COMPONENT           ${MODULE_LIBRARY_NAME}
)

install(
	FILES       ${CMAKE_CURRENT_SOURCE_DIR}/c_generated_code/libacados_ocp_solver_front.so
				${CMAKE_CURRENT_SOURCE_DIR}/c_generated_code/libacados_ocp_solver_back.so
	COMPONENT   ${MODULE_LIBRARY_NAME}
	DESTINATION /usr/lib/dls2/controllers/${PLUGIN_NAME}
)
install(TARGETS pydwmpc DESTINATION /usr/lib/dls2/controllers/${PLUGIN_NAME})