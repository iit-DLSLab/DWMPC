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
# add_executable(test
# 	src/test.cpp
# )

# target_link_libraries(test
# 	PUBLIC
# 	${MODULE_LIBRARY_NAME}
# 	vis_module
# )
# target_link_directories( test
# 	PUBLIC
# 	/usr/lib/dls2/motion_generators/vis
# )

pybind11_add_module(pydwmpc src/py_dwmpc.cpp)
target_link_libraries(pydwmpc
	PUBLIC
	${MODULE_LIBRARY_NAME}
	vis_module
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
# install(
# 	TARGETS test 
# 	RUNTIME DESTINATION /usr/lib/dls2/controllers/${PLUGIN_NAME}
# 	COMPONENT ${MODULE_LIBRARY_NAME} )
install(TARGETS pydwmpc DESTINATION /usr/lib/dls2/controllers/${PLUGIN_NAME})