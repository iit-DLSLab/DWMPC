#----------------------------------------------------------------
# Generated CMake target import file for configuration "Release".
#----------------------------------------------------------------

# Commands may need to know the format version.
set(CMAKE_IMPORT_FILE_VERSION 1)

# Import target "hpipm" for configuration "Release"
set_property(TARGET hpipm APPEND PROPERTY IMPORTED_CONFIGURATIONS RELEASE)
set_target_properties(hpipm PROPERTIES
  IMPORTED_LINK_INTERFACE_LIBRARIES_RELEASE "blasfeo"
  IMPORTED_LOCATION_RELEASE "${_IMPORT_PREFIX}/lib/libhpipm.so"
  IMPORTED_SONAME_RELEASE "libhpipm.so"
  )

list(APPEND _IMPORT_CHECK_TARGETS hpipm )
list(APPEND _IMPORT_CHECK_FILES_FOR_hpipm "${_IMPORT_PREFIX}/lib/libhpipm.so" )

# Commands beyond this point should not need to know the version.
set(CMAKE_IMPORT_FILE_VERSION)
