add_library(predator_nn SHARED
  nn/neural_network.h
  nn/neural_network.cpp
  nn/perceptron.h
  nn/perceptron.cpp
  nn/ctrnn_multilayer.h
  nn/ctrnn_multilayer.cpp
  predator_nn_controller.h
  predator_nn_controller.cpp
)
target_link_libraries(predator_nn
  argos3core_simulator
  argos3plugin_simulator_footbot
  argos3plugin_simulator_genericrobot)
