include(FindPackageHandleStandardArgs)

set(PQP_ROOT_DIR "../IPPP_third_party/" CACHE PATH "Folder containing PQP")

find_path(PQP_INCLUDE_DIR PQP.h
    PATHS ${PQP_ROOT_DIR}
    PATH_SUFFIXES
    include)

find_library(PQP_LIBRARY PQP
    PATHS ${PQP_ROOT_DIR}
    PATH_SUFFIXES
    lib)

include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(PQP
        FAIL_MESSAGE  DEFAULT_MSG
        REQUIRED_VARS PQP_INCLUDE_DIR PQP_LIBRARY
        VERSION_VAR FCL_VERSION)

if(PQP_FOUND)
    add_library(pqp::pqp UNKNOWN IMPORTED ${PQP_LIBRARY})
    set_target_properties(pqp::pqp PROPERTIES INTERFACE_INCLUDE_DIRECTORIES "${PQP_INCLUDE_DIR}")
    if(EXISTS "${PQP_LIBRARY}")
        set_target_properties(pqp::pqp PROPERTIES IMPORTED_LOCATION "${PQP_LIBRARY}")
    endif()
endif()    
