# generated from ament/cmake/core/templates/nameConfig.cmake.in

# prevent multiple inclusion
if(_plumbing_meta_func_pkg_CONFIG_INCLUDED)
  # ensure to keep the found flag the same
  if(NOT DEFINED plumbing_meta_func_pkg_FOUND)
    # explicitly set it to FALSE, otherwise CMake will set it to TRUE
    set(plumbing_meta_func_pkg_FOUND FALSE)
  elseif(NOT plumbing_meta_func_pkg_FOUND)
    # use separate condition to avoid uninitialized variable warning
    set(plumbing_meta_func_pkg_FOUND FALSE)
  endif()
  return()
endif()
set(_plumbing_meta_func_pkg_CONFIG_INCLUDED TRUE)

# output package information
if(NOT plumbing_meta_func_pkg_FIND_QUIETLY)
  message(STATUS "Found plumbing_meta_func_pkg: 0.0.0 (${plumbing_meta_func_pkg_DIR})")
endif()

# warn when using a deprecated package
if(NOT "" STREQUAL "")
  set(_msg "Package 'plumbing_meta_func_pkg' is deprecated")
  # append custom deprecation text if available
  if(NOT "" STREQUAL "TRUE")
    set(_msg "${_msg} ()")
  endif()
  # optionally quiet the deprecation message
  if(NOT ${plumbing_meta_func_pkg_DEPRECATED_QUIET})
    message(DEPRECATION "${_msg}")
  endif()
endif()

# flag package as ament-based to distinguish it after being find_package()-ed
set(plumbing_meta_func_pkg_FOUND_AMENT_PACKAGE TRUE)

# include all config extra files
set(_extras "")
foreach(_extra ${_extras})
  include("${plumbing_meta_func_pkg_DIR}/${_extra}")
endforeach()
