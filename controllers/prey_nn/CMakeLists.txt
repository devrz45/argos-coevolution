add_library(prey_nn SHARED
  nn/neural_network.h
  nn/neural_network.cpp
  nn/perceptron.h
  nn/perceptron.cpp
  nn/ctrnn_multilayer.h
  nn/ctrnn_multilayer.cpp
  prey_nn_controller.h
  prey_nn_controller.cpp
)
target_link_libraries(prey_nn
  argos3core_simulator
  argos3plugin_simulator_footbot
  argos3plugin_simulator_genericrobot)
