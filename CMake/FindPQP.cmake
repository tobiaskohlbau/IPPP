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

find_package_handle_standard_args(PQP DEFAULT_MSG
    PQP_INCLUDE_DIR PQP_LIBRARY)

if(PQP_FOUND)
    set(PQP ${PQP_INCLUDE_DIR})
    set(PQP_LIBRARIES ${PQP_LIBRARY})
endif()    