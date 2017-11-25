add_library(ui STATIC BenchmarkReader.cpp)
target_link_libraries(ui
    PRIVATE
        IPPP)
set_target_properties(ui PROPERTIES INTERFACE_INCLUDE_DIRECTORIES "${CMAKE_CURRENT_LIST_DIR}")
