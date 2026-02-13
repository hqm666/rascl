# generated from ament/cmake/core/templates/nameConfig.cmake.in

# prevent multiple inclusion
if(_motorctl_cpp_CONFIG_INCLUDED)
  # ensure to keep the found flag the same
  if(NOT DEFINED motorctl_cpp_FOUND)
    # explicitly set it to FALSE, otherwise CMake will set it to TRUE
    set(motorctl_cpp_FOUND FALSE)
  elseif(NOT motorctl_cpp_FOUND)
    # use separate condition to avoid uninitialized variable warning
    set(motorctl_cpp_FOUND FALSE)
  endif()
  return()
endif()
set(_motorctl_cpp_CONFIG_INCLUDED TRUE)

# output package information
if(NOT motorctl_cpp_FIND_QUIETLY)
  message(STATUS "Found motorctl_cpp: 0.0.0 (${motorctl_cpp_DIR})")
endif()

# warn when using a deprecated package
if(NOT "" STREQUAL "")
  set(_msg "Package 'motorctl_cpp' is deprecated")
  # append custom deprecation text if available
  if(NOT "" STREQUAL "TRUE")
    set(_msg "${_msg} ()")
  endif()
  # optionally quiet the deprecation message
  if(NOT motorctl_cpp_DEPRECATED_QUIET)
    message(DEPRECATION "${_msg}")
  endif()
endif()

# flag package as ament-based to distinguish it after being find_package()-ed
set(motorctl_cpp_FOUND_AMENT_PACKAGE TRUE)

# include all config extra files
set(_extras "")
foreach(_extra ${_extras})
  include("${motorctl_cpp_DIR}/${_extra}")
endforeach()
