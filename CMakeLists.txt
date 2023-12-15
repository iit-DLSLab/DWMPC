cmake_minimum_required(VERSION 3.16.2)
project(dwmpc)

set(CMAKE_CXX_FLAGS "-O3")

find_package(Eigen3 3.3 REQUIRED NO_MODULE)
# find_package(Matplot++ REQUIRED)

include_directories(
    PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}/include
        /opt/openrobots/include
)

link_directories(/opt/openrobots/lib)

# DWOPC library
add_library(dwocp SHARED src/dwocp.cpp)

#target_include_directories(dwocp
#    PUBLIC
#        $ENV{HOME}/robot-code/acados/include
#        $ENV{HOME}/robot-code/acados/include/blasfeo/include
#        $ENV{HOME}/robot-code/acados/include/hpipm/include
#        ${CMAKE_CURRENT_SOURCE_DIR}/../c_generated_code
#)

#target_link_directories(dwocp PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/../c_generated_code)

#target_link_libraries(dwocp
#    PUBLIC
#        libacados_ocp_solver_FL.so
#        libacados_ocp_solver_FR.so
#        libacados_ocp_solver_RL.so
#        libacados_ocp_solver_RR.so
#)

# DWMPC library
add_library(${PROJECT_NAME} src/dwmpc.cpp)# src/timer.cpp)

#target_include_directories(${PROJECT_NAME} PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/third-party/ndcurves/include)

## target_link_libraries(${PROJECT_NAME} PUBLIC dwocp Eigen3::Eigen ndcurves Matplot++::matplot)
#target_link_libraries(${PROJECT_NAME} PUBLIC dwocp Eigen3::Eigen ndcurves)

## Test executable
#add_executable(test test/test.cpp)

#target_link_libraries(test PUBLIC ${PROJECT_NAME})