#ifndef PREDATOR_NN_CONTROLLER
#define PREDATOR_NN_CONTROLLER

/*
 * Include some necessary headers.
 */
/* Definition of the CCI_Controller class. */
#include <argos3/core/control_interface/ci_controller.h>
/* Definition of the differential steering actuator */
#include <argos3/plugins/robots/generic/control_interface/ci_differential_steering_actuator.h>
/* Definition of the foot-bot proximity sensor */
#include <argos3/plugins/robots/foot-bot/control_interface/ci_footbot_proximity_sensor.h>
/* Definition of the omnidirectional camera sensor */
#include <argos3/plugins/robots/generic/control_interface/ci_colored_blob_omnidirectional_camera_sensor.h>
/* Definition of the LEDs actuator */
#include <argos3/plugins/robots/generic/control_interface/ci_leds_actuator.h>
/* Definition of the perceptron */
#include "nn/perceptron.h"

#define MAXDIST_PREDATOR 170.0f



/*
 * All the ARGoS stuff in the 'argos' namespace.
 * With this statement, you save typing argos:: every time.
 */
using namespace argos;

/*
 * A controller is simply an implementation of the CCI_Controller class.
 * In this case, we also inherit from the CPerceptron class. We use
 * virtual inheritance so that matching methods in the CCI_Controller
 * and CPerceptron don't get messed up.
 */
class CPredatorNNController : public CCI_Controller {

private:
   const static uint8_t COLORS = 1;
   const static uint8_t SECTORS = 5;
   Real distances[SECTORS];
   Real mapped[SECTORS*COLORS];
public:

   CPredatorNNController();
   virtual ~CPredatorNNController();

   void Init(TConfigurationNode& t_node);
   void ControlStep();
   void Reset();
   void Destroy();

   // inline CPerceptron& GetPerceptron() {
   //    return m_neuralNet;
   // }

private:

      /* Pointer to the differential steering actuator */
   CCI_DifferentialSteeringActuator* m_pcWheels;
   /* Pointer to the foot-bot proximity sensor */
   CCI_FootBotProximitySensor* m_pcProximity;
   /* Pointer to the foot-bot light sensor */
   CCI_ColoredBlobOmnidirectionalCameraSensor* m_pcCamera;
   /* Pointer to the LEDs actuator */
   CCI_LEDsActuator* m_pcLEDs;
   /* The perceptron neural network */
   //CPerceptron m_neuralNet;
   /* Wheel speeds */
   Real m_fLeftSpeed, m_fRightSpeed;
   /* Base Velocity */
   Real m_fWheelVelocity;
   /* Front angle */
   CDegrees frontAngle;
   /* Mid angle */
   CDegrees  midAngle;
   /* Far angle */
   CDegrees farAngle;
   /* Central neuron angle range from the camera */
   CRange<CRadians> m_centralAngleRange;
   /* Left mid Peripheral neuron angle range from the camera */
   CRange<CRadians> m_lMidPerAngleRange;
   /* Left far Peripheral neuron angle range from the camera */
   CRange<CRadians> m_lFarPerAngleRange;
   /* Right mid Peripheral neuron angle range from the camera */
   CRange<CRadians> m_rMidPerAngleRange;
   /* Right far Peripheral neuron angle range from the camera */
   CRange<CRadians> m_rFarPerAngleRange;
   /* Offset for wheel motion*/
   Real offset;

   bool leftWall;
   bool rightWall ;
   uint8_t counter;
};


#endif
