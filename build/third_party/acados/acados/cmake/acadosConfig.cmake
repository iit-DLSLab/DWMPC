#
# Copyright (c) The acados authors.
#
# This file is part of acados.
#
# The 2-Clause BSD License
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
# 1. Redistributions of source code must retain the above copyright notice,
# this list of conditions and the following disclaimer.
#
# 2. Redistributions in binary form must reproduce the above copyright notice,
# this list of conditions and the following disclaimer in the documentation
# and/or other materials provided with the distribution.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.;
#

# Enabled external modules
set(ACADOS_WITH_OPENMP OFF)
set(ACADOS_WITH_HPMPC OFF)
set(ACADOS_WITH_QORE OFF)
set(ACADOS_WITH_QPOASES OFF)
set(ACADOS_WITH_QPDUNES OFF)
set(ACADOS_WITH_OSQP OFF)
set(ACADOS_WITH_OOQP OFF)
set(ACADOS_WITH_DAQP OFF)

# Add acados CMake folder to CMake prefix and module path
set(CMAKE_MODULE_PATH_save "${CMAKE_MODULE_PATH}")
list(APPEND CMAKE_PREFIX_PATH "${CMAKE_CURRENT_LIST_DIR}/../") # for *Config.cmake
list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_LIST_DIR}")     # for FindOpenBLAS, FindFortranLibs

include(CMakeFindDependencyMacro)

# Find the enabled external modules
find_dependency(blasfeo)
find_dependency(hpipm)

if (ACADOS_WITH_OPENMP)
    find_dependency(OpenMP)
endif()

if(ACADOS_WITH_QPOASES)
    find_dependency(qpOASES_e)
endif()

if(ACADOS_WITH_HPMPC)
    find_dependency(hpmpc)
endif()

if(ACADOS_WITH_QPDUNES)
    find_dependency(qpdunes)
endif()

if(ACADOS_WITH_QORE)
    find_dependency(qore)
endif()

if(ACADOS_WITH_OOQP)
    find_dependency(ooqp)

    find_dependency(OpenBLAS)
    if (NOT TARGET openblas)
        add_library(openblas UNKNOWN IMPORTED)
        set_property(TARGET openblas PROPERTY IMPORTED_LOCATION ${OpenBLAS_LIB})
    endif()

    find_dependency(FortranLibs)
    if (NOT TARGET gfortran)
        add_library(gfortran UNKNOWN IMPORTED)
        set_property(TARGET gfortran PROPERTY IMPORTED_LOCATION ${FORTRAN_LIBRARY})
    endif()
endif()

if(ACADOS_WITH_DAQP)
    find_package(daqp)
endif()

if(ACADOS_WITH_OSQP)
    find_package(osqp)
endif()

include("${CMAKE_CURRENT_LIST_DIR}/acadosTargets.cmake")

set(CMAKE_MODULE_PATH "${CMAKE_MODULE_PATH_save}")
unset(CMAKE_MODULE_PATH_save)
