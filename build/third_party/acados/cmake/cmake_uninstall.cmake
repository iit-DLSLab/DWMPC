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


# Remove everything from install_manifest
IF(NOT EXISTS "/home/iit.local/lamatucci/DWMPC/build/third_party/acados/install_manifest.txt")
    MESSAGE(FATAL_ERROR "Cannot find install manifest: '/home/iit.local/lamatucci/DWMPC/build/third_party/acados/install_manifest.txt'")
ENDIF()

FILE(READ "/home/iit.local/lamatucci/DWMPC/build/third_party/acados/install_manifest.txt" files)
STRING(REGEX REPLACE "\n" ";" files "${files}")
list(REVERSE files)
FOREACH(file ${files})
    MESSAGE(STATUS "Uninstalling $ENV{DESTDIR}${file}")
    IF(EXISTS "$ENV{DESTDIR}${file}" OR IS_SYMLINK "$ENV{DESTDIR}${file}")
        EXEC_PROGRAM(
        "/usr/bin/cmake" ARGS "-E remove -f $ENV{DESTDIR}${file}"
        OUTPUT_VARIABLE rm_out
        RETURN_VALUE rm_retval)
        IF(NOT "${rm_retval}" STREQUAL 0)
            MESSAGE(FATAL_ERROR "Problem when removing $ENV{DESTDIR}${file}")
        ENDIF()
    ELSE()
        MESSAGE(STATUS "File $ENV{DESTDIR}${file} does not exist.")
    ENDIF()
ENDFOREACH()

# Remove directories
execute_process(COMMAND ${CMAKE_COMMAND} -E remove_directory /home/iit.local/lamatucci/DWMPC/third_party/acados/include/acados)
execute_process(COMMAND ${CMAKE_COMMAND} -E remove_directory /home/iit.local/lamatucci/DWMPC/third_party/acados/include/hpmpc)
execute_process(COMMAND ${CMAKE_COMMAND} -E remove_directory /home/iit.local/lamatucci/DWMPC/third_party/acados/include/ooqp)
execute_process(COMMAND ${CMAKE_COMMAND} -E remove_directory /home/iit.local/lamatucci/DWMPC/third_party/acados/include/qpdunes)
execute_process(COMMAND ${CMAKE_COMMAND} -E remove_directory /home/iit.local/lamatucci/DWMPC/third_party/acados/include/qpOASES_e)
execute_process(COMMAND ${CMAKE_COMMAND} -E remove_directory /home/iit.local/lamatucci/DWMPC/third_party/acados/lib/+acados)
