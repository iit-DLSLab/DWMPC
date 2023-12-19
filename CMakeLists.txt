cmake_minimum_required(VERSION 3.16.2)
project(dwmpc)

set(CMAKE_CXX_FLAGS "-O3")

find_package(Eigen3 3.3 REQUIRED NO_MODULE)

add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/third_party/acados)

include_directories(PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)

# DWOPC library
add_library(dwocp SHARED src/dwocp.cpp)

target_include_directories(dwocp
    PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}/c_generated_code
)

target_link_directories(dwocp
    PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}/c_generated_code
)

target_link_libraries(dwocp
    PUBLIC
        acados
        libacados_ocp_solver_FL.so
        libacados_ocp_solver_FR.so
        libacados_ocp_solver_RL.so
        libacados_ocp_solver_RR.so
)

# DWMPC library
add_library(${PROJECT_NAME} src/dwmpc.cpp src/timer.cpp)

target_include_directories(${PROJECT_NAME} PUBLIC /opt/openrobots/include)

target_link_directories(${PROJECT_NAME} PUBLIC /opt/openrobots/lib)

target_link_libraries(${PROJECT_NAME} PUBLIC dwocp Eigen3::Eigen)

# Test executable
add_executable(test test/test.cpp)

target_link_libraries(test PUBLIC ${PROJECT_NAME})