# generated from ament/cmake/core/templates/nameConfig.cmake.in

# prevent multiple inclusion
if(_clearpath_simulator_CONFIG_INCLUDED)
  # ensure to keep the found flag the same
  if(NOT DEFINED clearpath_simulator_FOUND)
    # explicitly set it to FALSE, otherwise CMake will set it to TRUE
    set(clearpath_simulator_FOUND FALSE)
  elseif(NOT clearpath_simulator_FOUND)
    # use separate condition to avoid uninitialized variable warning
    set(clearpath_simulator_FOUND FALSE)
  endif()
  return()
endif()
set(_clearpath_simulator_CONFIG_INCLUDED TRUE)

# output package information
if(NOT clearpath_simulator_FIND_QUIETLY)
  message(STATUS "Found clearpath_simulator: 2.7.0 (${clearpath_simulator_DIR})")
endif()

# warn when using a deprecated package
if(NOT "" STREQUAL "")
  set(_msg "Package 'clearpath_simulator' is deprecated")
  # append custom deprecation text if available
  if(NOT "" STREQUAL "TRUE")
    set(_msg "${_msg} ()")
  endif()
  # optionally quiet the deprecation message
  if(NOT clearpath_simulator_DEPRECATED_QUIET)
    message(DEPRECATION "${_msg}")
  endif()
endif()

# flag package as ament-based to distinguish it after being find_package()-ed
set(clearpath_simulator_FOUND_AMENT_PACKAGE TRUE)

# include all config extra files
set(_extras "")
foreach(_extra ${_extras})
  include("${clearpath_simulator_DIR}/${_extra}")
endforeach()
