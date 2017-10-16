include(FindPackageHandleStandardArgs)

set(NLOHMANN_JSON_ROOT_DIR "" CACHE PATH "Folder containing nlohmann json")

find_path(NLOHMANN_JSON_INCLUDE_DIR json.hpp
    PATHS ${NLOHMANN_JSON_ROOT_DIR}
    PATH_SUFFIXES
    include)

find_package_handle_standard_args(NLOHMANN_JSON DEFAULT_MSG
    NLOHMANN_JSON_INCLUDE_DIR)

if(NLOHMAN_JSON_FOUND)
    set(nlohmann-json ${NLOHMANN_JSON_INCLUDE_DIR})
endif()    