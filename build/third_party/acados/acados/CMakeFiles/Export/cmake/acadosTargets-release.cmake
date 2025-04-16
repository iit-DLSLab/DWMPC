#----------------------------------------------------------------
# Generated CMake target import file for configuration "Release".
#----------------------------------------------------------------

# Commands may need to know the format version.
set(CMAKE_IMPORT_FILE_VERSION 1)

# Import target "acados" for configuration "Release"
set_property(TARGET acados APPEND PROPERTY IMPORTED_CONFIGURATIONS RELEASE)
set_target_properties(acados PROPERTIES
  IMPORTED_LOCATION_RELEASE "${_IMPORT_PREFIX}/lib/libacados.so"
  IMPORTED_SONAME_RELEASE "libacados.so"
  )

list(APPEND _IMPORT_CHECK_TARGETS acados )
list(APPEND _IMPORT_CHECK_FILES_FOR_acados "${_IMPORT_PREFIX}/lib/libacados.so" )

# Commands beyond this point should not need to know the version.
set(CMAKE_IMPORT_FILE_VERSION)
