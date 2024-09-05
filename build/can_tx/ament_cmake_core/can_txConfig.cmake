# generated from ament/cmake/core/templates/nameConfig.cmake.in

# prevent multiple inclusion
if(_can_tx_CONFIG_INCLUDED)
  # ensure to keep the found flag the same
  if(NOT DEFINED can_tx_FOUND)
    # explicitly set it to FALSE, otherwise CMake will set it to TRUE
    set(can_tx_FOUND FALSE)
  elseif(NOT can_tx_FOUND)
    # use separate condition to avoid uninitialized variable warning
    set(can_tx_FOUND FALSE)
  endif()
  return()
endif()
set(_can_tx_CONFIG_INCLUDED TRUE)

# output package information
if(NOT can_tx_FIND_QUIETLY)
  message(STATUS "Found can_tx: 0.1.0 (${can_tx_DIR})")
endif()

# warn when using a deprecated package
if(NOT "" STREQUAL "")
  set(_msg "Package 'can_tx' is deprecated")
  # append custom deprecation text if available
  if(NOT "" STREQUAL "TRUE")
    set(_msg "${_msg} ()")
  endif()
  # optionally quiet the deprecation message
  if(NOT ${can_tx_DEPRECATED_QUIET})
    message(DEPRECATION "${_msg}")
  endif()
endif()

# flag package as ament-based to distinguish it after being find_package()-ed
set(can_tx_FOUND_AMENT_PACKAGE TRUE)

# include all config extra files
set(_extras "ament_cmake_export_dependencies-extras.cmake;ament_cmake_export_include_directories-extras.cmake")
foreach(_extra ${_extras})
  include("${can_tx_DIR}/${_extra}")
endforeach()
