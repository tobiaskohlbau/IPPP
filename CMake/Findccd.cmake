# Copyright (c) 2015-2016, Humanoid Lab, Georgia Tech Research Corporation
# Copyright (c) 2015-2017, Graphics Lab, Georgia Tech Research Corporation
# Copyright (c) 2016-2017, Personal Robotics Lab, Carnegie Mellon University
# This file is provided under the "BSD-style" License

# Find CCD
#
# This sets the following variables:
# CCD_FOUND
# CCD_INCLUDE_DIRS
# CCD_LIBRARIES
# CCD_VERSION

set(CCD_ROOT_DIR "../IPPP_third_party/" CACHE PATH "ccd root dir")

find_package(PkgConfig QUIET)

# Check to see if pkgconfig is installed.
pkg_check_modules(PC_CCD ccd QUIET)

find_path(CCD_INCLUDE_DIR
        NAMES ccd/ccd.h
        HINTS ${PC_CCD_INCLUDEDIR}
        PATHS ${CCD_ROOT_DIR}/include
        PATH_SUFFIXES
        include)

find_library(CCD_LIBRARY_DEBUG
        NAMES ccd
        HINTS ${PC_CCD_LIBDIR}
        PATHS ${CCD_ROOT_DIR}
        PATH_SUFFIXES
        debug/lib)

find_library(CCD_LIBRARY_RELEASE
        NAMES ccd
        HINTS ${PC_CCD_LIBDIR}
        PATHS ${CCD_ROOT_DIR}
        PATH_SUFFIXES
        lib)

set(CCD_LIBRARY
        debug ${FCL_LIBRARY_DEBUG}
        optimized ${CCD_LIBRARY_RELEASE})

# Version
set(CCD_VERSION ${PC_CCD_VERSION})

# Set (NAME)_FOUND if all the variables and the version are satisfied.
include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(CCD
        FAIL_MESSAGE  DEFAULT_MSG
        REQUIRED_VARS CCD_INCLUDE_DIR CCD_LIBRARY
        VERSION_VAR CCD_VERSION)

if(CCD_FOUND)
    add_library(ccd::ccd UNKNOWN IMPORTED ${CCD_LIBRARY})
    set_target_properties(ccd::ccd PROPERTIES INTERFACE_INCLUDE_DIRECTORIES "${CCD_INCLUDE_DIR}")
    if(EXISTS "${CCD_LIBRARY}")
        set_target_properties(ccd::ccd PROPERTIES IMPORTED_LOCATION "${CCD_LIBRARY}")
    endif()
    if(EXISTS "${CCD_LIBRARY_DEBUG}")
        set_target_properties(ccd::ccd PROPERTIES IMPORTED_LOCATION_DEBUG "${CCD_LIBRARY_DEBUG}")
    endif()
    if(EXISTS "${CCD_LIBRARY_RELEASE}")
        set_target_properties(ccd::ccd PROPERTIES IMPORTED_LOCATION_RELEASE "${CCD_LIBRARY_RELEASE}")
    endif()
endif()    
