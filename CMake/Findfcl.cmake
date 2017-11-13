# Copyright (c) 2015-2016, Humanoid Lab, Georgia Tech Research Corporation
# Copyright (c) 2015-2017, Graphics Lab, Georgia Tech Research Corporation
# Copyright (c) 2016-2017, Personal Robotics Lab, Carnegie Mellon University
# This file is provided under the "BSD-style" License

# Find FCL
#
# This sets the following variables:
# FCL_FOUND
# FCL_INCLUDE_DIRS
# FCL_LIBRARIES
# FCL_VERSION

find_package(PkgConfig QUIET)

set(FCL_ROOT_DIR "../IPPP_third_party/" CACHE PATH "FCL root dir")

# Check to see if pkgconfig is installed.
pkg_check_modules(PC_FCL FCL)

# Include directories
find_path(FCL_INCLUDE_DIRS
        NAMES fcl/collision.h
        HINTS ${PC_FCL_INCLUDEDIR}
        PATHS "${CMAKE_INSTALL_PREFIX}/include" "${FCL_ROOT_DIR}/include"
        PATH_SUFFIXES
        include)

# Libraries
find_library(FCL_LIBRARY_RELEASE
        NAMES fcl
        HINTS ${PC_FCL_LIBDIR}
        PATHS "${CMAKE_INSTALL_PREFIX}" "${FCL_ROOT_DIR}"
        PATH_SUFFIXES
        lib)

find_library(FCL_LIBRARY_DEBUG
        NAMES fcl
        HINTS ${PC_FCL_LIBDIR}
        PATHS "${CMAKE_INSTALL_PREFIX}" "${FCL_ROOT_DIR}"
        PATH_SUFFIXES
        debug/lib)

set(FCL_LIBRARIES
        optimized ${FCL_LIBRARY_RELEASE}
        debug ${FCL_LIBRARY_DEBUG})

# Version
set(FCL_VERSION ${PC_FCL_VERSION})

# Set (NAME)_FOUND if all the variables and the version are satisfied.
include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(FCL
        FAIL_MESSAGE  DEFAULT_MSG
        REQUIRED_VARS FCL_INCLUDE_DIRS FCL_LIBRARIES
        VERSION_VAR FCL_VERSION)