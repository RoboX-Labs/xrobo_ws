# generated from ament/cmake/core/templates/nameConfig.cmake.in

# prevent multiple inclusion
if(_robocon_desktop_CONFIG_INCLUDED)
  # ensure to keep the found flag the same
  if(NOT DEFINED robocon_desktop_FOUND)
    # explicitly set it to FALSE, otherwise CMake will set it to TRUE
    set(robocon_desktop_FOUND FALSE)
  elseif(NOT robocon_desktop_FOUND)
    # use separate condition to avoid uninitialized variable warning
    set(robocon_desktop_FOUND FALSE)
  endif()
  return()
endif()
set(_robocon_desktop_CONFIG_INCLUDED TRUE)

# output package information
if(NOT robocon_desktop_FIND_QUIETLY)
  message(STATUS "Found robocon_desktop: 0.0.0 (${robocon_desktop_DIR})")
endif()

# warn when using a deprecated package
if(NOT "" STREQUAL "")
  set(_msg "Package 'robocon_desktop' is deprecated")
  # append custom deprecation text if available
  if(NOT "" STREQUAL "TRUE")
    set(_msg "${_msg} ()")
  endif()
  # optionally quiet the deprecation message
  if(NOT ${robocon_desktop_DEPRECATED_QUIET})
    message(DEPRECATION "${_msg}")
  endif()
endif()

# flag package as ament-based to distinguish it after being find_package()-ed
set(robocon_desktop_FOUND_AMENT_PACKAGE TRUE)

# include all config extra files
set(_extras "")
foreach(_extra ${_extras})
  include("${robocon_desktop_DIR}/${_extra}")
endforeach()
