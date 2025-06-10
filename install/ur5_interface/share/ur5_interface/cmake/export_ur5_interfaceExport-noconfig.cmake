#----------------------------------------------------------------
# Generated CMake target import file.
#----------------------------------------------------------------

# Commands may need to know the format version.
set(CMAKE_IMPORT_FILE_VERSION 1)

# Import target "ur5_interface::ur5_interface" for configuration ""
set_property(TARGET ur5_interface::ur5_interface APPEND PROPERTY IMPORTED_CONFIGURATIONS NOCONFIG)
set_target_properties(ur5_interface::ur5_interface PROPERTIES
  IMPORTED_LOCATION_NOCONFIG "${_IMPORT_PREFIX}/lib/libur5_interface.so"
  IMPORTED_SONAME_NOCONFIG "libur5_interface.so"
  )

list(APPEND _IMPORT_CHECK_TARGETS ur5_interface::ur5_interface )
list(APPEND _IMPORT_CHECK_FILES_FOR_ur5_interface::ur5_interface "${_IMPORT_PREFIX}/lib/libur5_interface.so" )

# Commands beyond this point should not need to know the version.
set(CMAKE_IMPORT_FILE_VERSION)
