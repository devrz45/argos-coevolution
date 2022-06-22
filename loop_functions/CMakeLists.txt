include_directories(${CMAKE_SOURCE_DIR})

# If GALIB was found, also descend into galib_phototaxis_loop_functions
if(GALIB_FOUND)
  add_subdirectory(co_evolution_loop_functions)
endif(GALIB_FOUND)
