# Install script for directory: /home/iit.local/lamatucci/DWMPC

# Set the install prefix
if(NOT DEFINED CMAKE_INSTALL_PREFIX)
  set(CMAKE_INSTALL_PREFIX "/home/iit.local/lamatucci/DWMPC/third_party/acados")
endif()
string(REGEX REPLACE "/$" "" CMAKE_INSTALL_PREFIX "${CMAKE_INSTALL_PREFIX}")

# Set the install configuration name.
if(NOT DEFINED CMAKE_INSTALL_CONFIG_NAME)
  if(BUILD_TYPE)
    string(REGEX REPLACE "^[^A-Za-z0-9_]+" ""
           CMAKE_INSTALL_CONFIG_NAME "${BUILD_TYPE}")
  else()
    set(CMAKE_INSTALL_CONFIG_NAME "Release")
  endif()
  message(STATUS "Install configuration: \"${CMAKE_INSTALL_CONFIG_NAME}\"")
endif()

# Set the component getting installed.
if(NOT CMAKE_INSTALL_COMPONENT)
  if(COMPONENT)
    message(STATUS "Install component: \"${COMPONENT}\"")
    set(CMAKE_INSTALL_COMPONENT "${COMPONENT}")
  else()
    set(CMAKE_INSTALL_COMPONENT)
  endif()
endif()

# Install shared libraries without execute permission?
if(NOT DEFINED CMAKE_INSTALL_SO_NO_EXE)
  set(CMAKE_INSTALL_SO_NO_EXE "1")
endif()

# Is this installation the result of a crosscompile?
if(NOT DEFINED CMAKE_CROSSCOMPILING)
  set(CMAKE_CROSSCOMPILING "FALSE")
endif()

# Set default install directory permissions.
if(NOT DEFINED CMAKE_OBJDUMP)
  set(CMAKE_OBJDUMP "/usr/bin/objdump")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("/home/iit.local/lamatucci/DWMPC/build/third_party/acados/cmake_install.cmake")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xdwmpc_module_devx" OR NOT CMAKE_INSTALL_COMPONENT)
  list(APPEND CMAKE_ABSOLUTE_DESTINATION_FILES
   "/usr/include/dls2/")
  if(CMAKE_WARN_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(WARNING "ABSOLUTE path INSTALL DESTINATION : ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
  if(CMAKE_ERROR_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(FATAL_ERROR "ABSOLUTE path INSTALL DESTINATION forbidden (by caller): ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
  file(INSTALL DESTINATION "/usr/include/dls2" TYPE DIRECTORY FILES "/home/iit.local/lamatucci/DWMPC/include/")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xdwmpc_modulex" OR NOT CMAKE_INSTALL_COMPONENT)
  list(APPEND CMAKE_ABSOLUTE_DESTINATION_FILES
   "/usr/include/dls2/controllers/dwmpc/config")
  if(CMAKE_WARN_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(WARNING "ABSOLUTE path INSTALL DESTINATION : ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
  if(CMAKE_ERROR_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(FATAL_ERROR "ABSOLUTE path INSTALL DESTINATION forbidden (by caller): ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
  file(INSTALL DESTINATION "/usr/include/dls2/controllers/dwmpc" TYPE DIRECTORY FILES "/home/iit.local/lamatucci/DWMPC/config")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xdwmpc_modulex" OR NOT CMAKE_INSTALL_COMPONENT)
  if(EXISTS "$ENV{DESTDIR}/usr/lib/dls2/controllers/dwmpc/libdwmpc_module.so" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}/usr/lib/dls2/controllers/dwmpc/libdwmpc_module.so")
    file(RPATH_CHECK
         FILE "$ENV{DESTDIR}/usr/lib/dls2/controllers/dwmpc/libdwmpc_module.so"
         RPATH "")
  endif()
  list(APPEND CMAKE_ABSOLUTE_DESTINATION_FILES
   "/usr/lib/dls2/controllers/dwmpc/libdwmpc_module.so")
  if(CMAKE_WARN_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(WARNING "ABSOLUTE path INSTALL DESTINATION : ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
  if(CMAKE_ERROR_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(FATAL_ERROR "ABSOLUTE path INSTALL DESTINATION forbidden (by caller): ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
  file(INSTALL DESTINATION "/usr/lib/dls2/controllers/dwmpc" TYPE SHARED_LIBRARY FILES "/home/iit.local/lamatucci/DWMPC/build/libdwmpc_module.so")
  if(EXISTS "$ENV{DESTDIR}/usr/lib/dls2/controllers/dwmpc/libdwmpc_module.so" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}/usr/lib/dls2/controllers/dwmpc/libdwmpc_module.so")
    file(RPATH_CHANGE
         FILE "$ENV{DESTDIR}/usr/lib/dls2/controllers/dwmpc/libdwmpc_module.so"
         OLD_RPATH "/opt/openrobots/lib:/home/iit.local/lamatucci/DWMPC/c_generated_code:/usr/lib/dls2/controllers/dwmpc:/home/iit.local/lamatucci/DWMPC/build/third_party/acados/acados:/home/iit.local/lamatucci/DWMPC/build/third_party/acados/external/hpipm:/home/iit.local/lamatucci/DWMPC/build/third_party/acados/external/blasfeo:"
         NEW_RPATH "")
    if(CMAKE_INSTALL_DO_STRIP)
      execute_process(COMMAND "/usr/bin/strip" "$ENV{DESTDIR}/usr/lib/dls2/controllers/dwmpc/libdwmpc_module.so")
    endif()
  endif()
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xdwmpc_modulex" OR NOT CMAKE_INSTALL_COMPONENT)
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xdwmpc_modulex" OR NOT CMAKE_INSTALL_COMPONENT)
  list(APPEND CMAKE_ABSOLUTE_DESTINATION_FILES
   "/usr/lib/dls2/controllers/dwmpc/libacados_ocp_solver_front.so;/usr/lib/dls2/controllers/dwmpc/libacados_ocp_solver_back.so")
  if(CMAKE_WARN_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(WARNING "ABSOLUTE path INSTALL DESTINATION : ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
  if(CMAKE_ERROR_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(FATAL_ERROR "ABSOLUTE path INSTALL DESTINATION forbidden (by caller): ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
  file(INSTALL DESTINATION "/usr/lib/dls2/controllers/dwmpc" TYPE FILE FILES
    "/home/iit.local/lamatucci/DWMPC/c_generated_code/libacados_ocp_solver_front.so"
    "/home/iit.local/lamatucci/DWMPC/c_generated_code/libacados_ocp_solver_back.so"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  if(EXISTS "$ENV{DESTDIR}/usr/lib/dls2/controllers/dwmpc/pydwmpc.cpython-310-x86_64-linux-gnu.so" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}/usr/lib/dls2/controllers/dwmpc/pydwmpc.cpython-310-x86_64-linux-gnu.so")
    file(RPATH_CHECK
         FILE "$ENV{DESTDIR}/usr/lib/dls2/controllers/dwmpc/pydwmpc.cpython-310-x86_64-linux-gnu.so"
         RPATH "")
  endif()
  list(APPEND CMAKE_ABSOLUTE_DESTINATION_FILES
   "/usr/lib/dls2/controllers/dwmpc/pydwmpc.cpython-310-x86_64-linux-gnu.so")
  if(CMAKE_WARN_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(WARNING "ABSOLUTE path INSTALL DESTINATION : ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
  if(CMAKE_ERROR_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(FATAL_ERROR "ABSOLUTE path INSTALL DESTINATION forbidden (by caller): ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
  file(INSTALL DESTINATION "/usr/lib/dls2/controllers/dwmpc" TYPE MODULE FILES "/home/iit.local/lamatucci/DWMPC/build/pydwmpc.cpython-310-x86_64-linux-gnu.so")
  if(EXISTS "$ENV{DESTDIR}/usr/lib/dls2/controllers/dwmpc/pydwmpc.cpython-310-x86_64-linux-gnu.so" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}/usr/lib/dls2/controllers/dwmpc/pydwmpc.cpython-310-x86_64-linux-gnu.so")
    file(RPATH_CHANGE
         FILE "$ENV{DESTDIR}/usr/lib/dls2/controllers/dwmpc/pydwmpc.cpython-310-x86_64-linux-gnu.so"
         OLD_RPATH "/opt/openrobots/lib:/home/iit.local/lamatucci/DWMPC/build:/home/iit.local/lamatucci/DWMPC/c_generated_code:/usr/lib/dls2/controllers/dwmpc:/home/iit.local/lamatucci/DWMPC/build/third_party/acados/acados:/home/iit.local/lamatucci/DWMPC/build/third_party/acados/external/hpipm:/home/iit.local/lamatucci/DWMPC/build/third_party/acados/external/blasfeo:"
         NEW_RPATH "")
    if(CMAKE_INSTALL_DO_STRIP)
      execute_process(COMMAND "/usr/bin/strip" "$ENV{DESTDIR}/usr/lib/dls2/controllers/dwmpc/pydwmpc.cpython-310-x86_64-linux-gnu.so")
    endif()
  endif()
endif()

if(CMAKE_INSTALL_COMPONENT)
  set(CMAKE_INSTALL_MANIFEST "install_manifest_${CMAKE_INSTALL_COMPONENT}.txt")
else()
  set(CMAKE_INSTALL_MANIFEST "install_manifest.txt")
endif()

string(REPLACE ";" "\n" CMAKE_INSTALL_MANIFEST_CONTENT
       "${CMAKE_INSTALL_MANIFEST_FILES}")
file(WRITE "/home/iit.local/lamatucci/DWMPC/build/${CMAKE_INSTALL_MANIFEST}"
     "${CMAKE_INSTALL_MANIFEST_CONTENT}")
