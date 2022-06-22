add_executable(co_evolution main.cpp)

target_link_libraries(co_evolution
  ${GALIB_LIBRARIES}
  prey_nn
  predator_nn
  co_evolution_loop_functions
  argos3core_simulator)
