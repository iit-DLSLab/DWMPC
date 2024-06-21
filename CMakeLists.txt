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

# ==============================================================================
# Calculate correct project version number
# ==============================================================================
# Read in the current version
file(READ version.txt PROJECT_VERSION)

# create array of version strings
string(REPLACE "." ";" VERSION_ARRAY ${PROJECT_VERSION})
# ------------------------------------------------------------------------------
# Major, Minor, Patch numbers
# ------------------------------------------------------------------------------
# set corresponding cmake version variables
list(POP_FRONT VERSION_ARRAY
	PROJECT_VERSION_MAJOR
	PROJECT_VERSION_MINOR
	PROJECT_VERSION_PATCH
)
# Remove the trailing newline from the patch number
string(REPLACE "\n" "" PROJECT_VERSION_PATCH ${PROJECT_VERSION_PATCH})

# ------------------------------------------------------------------------------
# Full version number
# ------------------------------------------------------------------------------
# Rebuild the project version including the current commit hash
string(JOIN "." PROJECT_VERSION
	${PROJECT_VERSION_MAJOR}
	${PROJECT_VERSION_MINOR}
	${PROJECT_VERSION_PATCH}
)
unset(VERSION_ARRAY) # delete aray of version strings
message(STATUS "SLIP DETECTION PROJECT VERSION ${PROJECT_VERSION}")

# ==============================================================================
# Setup the repository
# ==============================================================================
# This will perfom task such as setting up the git hooks for anyone who uses
# this project
execute_process(
	COMMAND           ./setup.sh
	WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
)

# Set RELEASE as the default build type
if(NOT CMAKE_BUILD_TYPE)
	set(CMAKE_BUILD_TYPE Release CACHE STRING "Build type" FORCE)
endif()

# use RPATH for the build tree
set(CMAKE_SKIP_BUILD_RPATH FALSE)

# the install RPATH is not used when building
set(CMAKE_BUILD_WITH_INSTALL_RPATH FALSE)

# add to RPATH automatically determined paths for the install tree
set(CMAKE_INSTALL_RPATH_USE_LINK_PATH TRUE)

# ================================== Library config ==========================
macro(conditional_deploy_plugin TARGET_NAME VALUE)
	set(${PLUGIN_NAME}_${TARGET_NAME} ${VALUE}
		CACHE BOOL "Whether to package ${TARGET_NAME}"
	)

	if(${PLUGIN_NAME}_${TARGET_NAME})
		add_subdirectory(${TARGET_NAME})
	endif()
endmacro()
conditional_deploy_plugin(plugin/core OFF)
conditional_deploy_plugin(plugin/console_commands OFF)
conditional_deploy_plugin(plugin/messages OFF)
conditional_deploy_plugin(plugin/topics OFF)
conditional_deploy_plugin(module OFF)

# ================================= Packaging =================================
set(CPACK_PACKAGE_VERSION_MAJOR             ${PROJECT_VERSION_MAJOR})
set(CPACK_PACKAGE_VERSION_MINOR             ${PROJECT_VERSION_MINOR})
set(CPACK_PACKAGE_VERSION_PATCH             ${PROJECT_VERSION_PATCH})
set(CPACK_DEBIAN_PACKAGE_VERSION            ${PROJECT_VERSION})
set(CPACK_PACKAGE_VENDOR                    "dlslab")
set(CPACK_PACKAGE_CONTACT                   "dlslab")
set(CPACK_DEBIAN_PACKAGE_MAINTAINER         "dlslab")
set(CPACK_VERBATIM_VARIABLES                ON)
set(CPACK_GENERATOR                         "DEB")
set(CPACK_DEB_COMPONENT_INSTALL             ON)
set(CPACK_DEBIAN_PACKAGE_SHLIBDEPS          ON)

set(CPACK_DEBIAN_${PLUGIN_NAME}_PACKAGE_NAME          "dls-${PLUGIN_NAME}" CACHE INTERNAL "")
set(CPACK_DEBIAN_${PLUGIN_NAME}_FILE_NAME             "dls-${PLUGIN_NAME}-${PROJECT_VERSION}.deb" CACHE INTERNAL "")  
set(CPACK_DEBIAN_${PLUGIN_NAME}_PACKAGE_DEPENDS       "dls2-runtime" CACHE INTERNAL "") # add here package dependencies

set(CPACK_DEBIAN_${PLUGIN_NAME}_DEV_PACKAGE_NAME      "dls-${PLUGIN_NAME}-dev" CACHE INTERNAL "")
set(CPACK_DEBIAN_${PLUGIN_NAME}_DEV_FILE_NAME         "dls-${PLUGIN_NAME}-dev-${PROJECT_VERSION}.deb" CACHE INTERNAL "")  
set(CPACK_DEBIAN_${PLUGIN_NAME}_DEV_PACKAGE_DEPENDS   "dls2-dev, dls-${PLUGIN_NAME}" CACHE INTERNAL "") # add here package dependencies

get_cmake_property(CPACK_COMPONENTS_ALL COMPONENTS)
list(REMOVE_ITEM CPACK_COMPONENTS_ALL "Unspecified")

include(CPack)
