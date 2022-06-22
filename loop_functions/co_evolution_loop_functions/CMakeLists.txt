link_directories(${CMAKE_BINARY_DIR}/controllers/prey_nn)
link_directories(${CMAKE_BINARY_DIR}/controllers/predator_nn)
add_library(co_evolution_loop_functions SHARED co_evolution_loop_functions.cpp)

target_link_libraries(co_evolution_loop_functions
  prey_nn
  predator_nn
  argos3core_simulator
  argos3plugin_simulator_dynamics2d
  argos3plugin_simulator_entities
  argos3plugin_simulator_footbot
  argos3plugin_simulator_genericrobot
  argos3plugin_simulator_media
  ${GALIB_LIBRARIES})
