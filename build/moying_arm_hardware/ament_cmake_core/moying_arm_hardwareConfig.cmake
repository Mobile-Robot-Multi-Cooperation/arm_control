# generated from ament/cmake/core/templates/nameConfig.cmake.in

# prevent multiple inclusion
if(_moying_arm_hardware_CONFIG_INCLUDED)
  # ensure to keep the found flag the same
  if(NOT DEFINED moying_arm_hardware_FOUND)
    # explicitly set it to FALSE, otherwise CMake will set it to TRUE
    set(moying_arm_hardware_FOUND FALSE)
  elseif(NOT moying_arm_hardware_FOUND)
    # use separate condition to avoid uninitialized variable warning
    set(moying_arm_hardware_FOUND FALSE)
  endif()
  return()
endif()
set(_moying_arm_hardware_CONFIG_INCLUDED TRUE)

# output package information
if(NOT moying_arm_hardware_FIND_QUIETLY)
  message(STATUS "Found moying_arm_hardware: 0.0.0 (${moying_arm_hardware_DIR})")
endif()

# warn when using a deprecated package
if(NOT "" STREQUAL "")
  set(_msg "Package 'moying_arm_hardware' is deprecated")
  # append custom deprecation text if available
  if(NOT "" STREQUAL "TRUE")
    set(_msg "${_msg} ()")
  endif()
  # optionally quiet the deprecation message
  if(NOT ${moying_arm_hardware_DEPRECATED_QUIET})
    message(DEPRECATION "${_msg}")
  endif()
endif()

# flag package as ament-based to distinguish it after being find_package()-ed
set(moying_arm_hardware_FOUND_AMENT_PACKAGE TRUE)

# include all config extra files
set(_extras "ament_cmake_export_include_directories-extras.cmake;ament_cmake_export_libraries-extras.cmake;ament_cmake_export_dependencies-extras.cmake")
foreach(_extra ${_extras})
  include("${moying_arm_hardware_DIR}/${_extra}")
endforeach()
