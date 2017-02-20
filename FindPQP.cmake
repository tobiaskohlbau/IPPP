include(FindPackageHandleStandardArgs)

set(PQP_ROOT_DIR "" CACHE PATH "Folder containing PQP")

find_path(PQP_INCLUDE_DIR include/PQP.h
    PATHS ${PQP_ROOT_DIR}
    PATH_SUFFIXES
    include)

find_library(PQP_LIBRARY pqp
    PATHS ${PQP_ROOT_DIR}
    PATH_SUFFIXES
    lib)

find_package_handle_standard_args(PQP DEFAULT_MSG
    PQP_INCLUDE_DIR PQP_LIBRARY)

if(PQP_FOUND)
    set(PQP ${PQP_INCLUDE_DIR})
    set(PQP_LIBRARIES ${PQP_LIBRARY})
endif()    