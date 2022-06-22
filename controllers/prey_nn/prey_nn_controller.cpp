#include "prey_nn_controller.h"

/****************************************/
/****************************************/

static CRange<Real> NN_OUTPUT_RANGE(0.0f, 1.0f);
static CRange<Real> WHEEL_ACTUATION_RANGE(-5.0f, 5.0f);
static CRange<Real> CAMERA_DISTANCE_RANGE(0.0f, MAXDIST_PREY);
static CRange<Real> NN_INPUT_RANGE(0.00f, 1.00f);
/****************************************/
/****************************************/

CPreyNNController::CPreyNNController():
        m_pcWheels(NULL),
        m_pcProximity(NULL),
        m_pcCamera(NULL),
        m_pcLEDs(NULL),
        m_fWheelVelocity(5.0f),
        Life(1),
        leftLife(Life),
        Dead(false),
        frontAngle(20),
        midAngle(60),
        farAngle(130),
        threshold(0.3f) {}

/****************************************/
/*************************************/

CPreyNNController::~CPreyNNController() {
}

/****************************************/
/****************************************/

void CPreyNNController::Init(TConfigurationNode& t_node) {
   /*
    * Get sensor/actuator handles
    */
   try {
      m_pcWheels = GetActuator<CCI_DifferentialSteeringActuator>("differential_steering");
      m_pcProximity = GetSensor<CCI_FootBotProximitySensor>("footbot_proximity");
      m_pcCamera = GetSensor<CCI_ColoredBlobOmnidirectionalCameraSensor>("colored_blob_omnidirectional_camera");
      m_pcLEDs = GetActuator<CCI_LEDsActuator>("leds");
   }
   catch(CARGoSException& ex) {
      THROW_ARGOSEXCEPTION_NESTED("Error initializing sensors/actuators", ex);
   }

   /*
    * Get/Set Parameter values
    */

   try {
      GetNodeAttributeOrDefault(t_node, "life", Life, Life);
      GetNodeAttributeOrDefault(t_node, "front", frontAngle, frontAngle);
      GetNodeAttributeOrDefault(t_node, "mid", midAngle, midAngle);
      GetNodeAttributeOrDefault(t_node, "far", farAngle, farAngle);
      GetNodeAttributeOrDefault(t_node, "velocity", m_fWheelVelocity, m_fWheelVelocity);
      GetNodeAttributeOrDefault(t_node, "threshold", threshold, threshold);

      m_centralAngleRange.Set(-ToRadians(frontAngle), ToRadians(frontAngle));
      m_lMidPerAngleRange.Set(ToRadians(frontAngle), ToRadians(midAngle));
      m_lFarPerAngleRange.Set(ToRadians(midAngle), ToRadians(farAngle));
      m_rMidPerAngleRange.Set(-ToRadians(midAngle), -ToRadians(frontAngle));
      m_rFarPerAngleRange.Set(-ToRadians(farAngle), -ToRadians(midAngle));

      WHEEL_ACTUATION_RANGE.Set(-m_fWheelVelocity,m_fWheelVelocity);
      //LOG<<"Life "<<Life<<std::endl;
      leftLife = Life;
   }
   catch(CARGoSException& ex) {
      THROW_ARGOSEXCEPTION_NESTED("Error initializing parameters", ex);
   }

   /* Initialize the perceptron */
   try {
      m_neuralNet.Init(t_node);
   }
   catch(CARGoSException& ex) {
      THROW_ARGOSEXCEPTION_NESTED("Error initializing the perceptron network", ex);
   }

   // Turn the led on for the prey
   m_pcLEDs->SetSingleColor(12,CColor::GREEN);
   // Turn on the camera
   m_pcCamera->Enable();
   // Initialize sector wise data  
   for(uint8_t i=0 ; i<SECTORS ; i++)
   {
      for(uint8_t j=0 ; j<COLORS ; j++)
          distances[i][j] = MAXDIST_PREY;
   }
}

/****************************************/
/****************************************/

void CPreyNNController::ControlStep() {
  //LOG<<leftLife<<std::endl;
  if(!Dead)
  {
   /* Get sensory data */
   const CCI_FootBotProximitySensor::TReadings& tProx = m_pcProximity->GetReadings();
   const CCI_ColoredBlobOmnidirectionalCameraSensor::SReadings& sCamera = m_pcCamera->GetReadings();
   CCI_ColoredBlobOmnidirectionalCameraSensor::SBlob* cblob;
   /* Reset the reading vectors*/
   for(uint8_t i=0 ; i<SECTORS ; i++)
   {
      for(uint8_t j=0 ; j<COLORS ; j++)
          distances[i][j] = MAXDIST_PREY;
   }
   /* Structure the readings from omnidirectional camera */
   if(! sCamera.BlobList.empty())
   {
      for(size_t i = 0; i < sCamera.BlobList.size(); ++i) {
        cblob = sCamera.BlobList[i];
        //LOG<<"Blob:"<<i<<" Color: "<<cblob->Color<<" distance: "<<cblob->Distance<<" angle: "<<cblob->Angle.GetValue()<<std::endl;
        if(cblob->Distance<MAXDIST_PREY)
        {
        uint8_t sector=5;
        if(cblob->Color==CColor::RED && cblob->Distance<18.0f)
          leftLife=leftLife-1;

        if(m_centralAngleRange.WithinMinBoundIncludedMaxBoundIncluded(cblob->Angle))
          sector = 0;
        
        else if(m_lMidPerAngleRange.WithinMinBoundExcludedMaxBoundIncluded(cblob->Angle))
          sector = 1;
        
        else if(m_lFarPerAngleRange.WithinMinBoundExcludedMaxBoundIncluded(cblob->Angle))
          sector = 2;
        
        else if(m_rMidPerAngleRange.WithinMinBoundExcludedMaxBoundIncluded(cblob->Angle))
          sector = 3;
        
        else if(m_rFarPerAngleRange.WithinMinBoundExcludedMaxBoundIncluded(cblob->Angle))
          sector = 4;

        if( sector!=5)
        {
          if (cblob->Color==CColor::CYAN && (distances[sector][CYAN]>cblob->Distance))
            distances[sector][CYAN] = cblob->Distance;
          
          else if (cblob->Color==CColor::YELLOW && (distances[sector][YELLOW]>cblob->Distance))
            distances[sector][YELLOW] = cblob->Distance;
          
          else if (cblob->Color==CColor::GREEN && (distances[sector][GREEN]>cblob->Distance))
            distances[sector][GREEN] = cblob->Distance;
          
          else if (cblob->Color==CColor::RED && (distances[sector][RED]>cblob->Distance))
            distances[sector][RED] = cblob->Distance;
        }
       }
      }
    }


   /* Fill NN inputs from omnidirection camera */
   uint16_t k=0;
   for(uint8_t i=0 ; i<SECTORS ; i++)
   {
      for(uint8_t j=0 ; j<COLORS ; j++)
      {
        CAMERA_DISTANCE_RANGE.MapValueIntoRange(mapped[k],distances[i][j],NN_INPUT_RANGE);
        //LOG<<mapped[k]<<" ";
        m_neuralNet.SetInput(k,1.0f - mapped[k]);
        ++k;    
      }
   }
   //LOG<<std::endl;

   /* Fill NN inputs from proximity sensor */
   for(uint8_t i = 0; i < tProx.size(); i+=3) {
      m_neuralNet.SetInput(k, tProx[i].Value);
   }
   /* Compute NN outputs */
   m_neuralNet.ComputeOutputs();
   /*
    * Apply NN outputs to actuation
    */
   NN_OUTPUT_RANGE.MapValueIntoRange(
      m_fLeftSpeed,               
      m_neuralNet.GetOutput(0), 
      WHEEL_ACTUATION_RANGE       
      );
   NN_OUTPUT_RANGE.MapValueIntoRange(
      m_fRightSpeed,              
      m_neuralNet.GetOutput(1), 
      WHEEL_ACTUATION_RANGE       
      );

   /* Set the signalling leds with required color if the neurons fire above a certain threshold */

   Real cyan = m_neuralNet.GetOutput(2);
   Real yellow = m_neuralNet.GetOutput(3);

   if(cyan>threshold || yellow>threshold)
   {
        if(cyan>=yellow) {
            m_pcLEDs->SetSingleColor(0,CColor::CYAN);
            m_pcLEDs->SetSingleColor(4,CColor::CYAN);
            m_pcLEDs->SetSingleColor(8,CColor::CYAN);
          }
        else {
            m_pcLEDs->SetSingleColor(0,CColor::YELLOW);
            m_pcLEDs->SetSingleColor(4,CColor::YELLOW);
            m_pcLEDs->SetSingleColor(8,CColor::YELLOW);
          }
   }
   else
   {
      m_pcLEDs->SetAllColors(CColor::BLACK);
      m_pcLEDs->SetSingleColor(12,CColor::GREEN);
   }

   m_pcWheels->SetLinearVelocity(
      m_fLeftSpeed,
      m_fRightSpeed);
 }
}

/****************************************/
/****************************************/

void CPreyNNController::Reset() {
   m_neuralNet.Reset();
   leftLife = Life;
   m_pcLEDs->SetSingleColor(12,CColor::GREEN);
   // Initialize sector wise data  
   for(uint8_t i=0 ; i<SECTORS ; i++)
   {
      for(uint8_t j=0 ; j<COLORS ; j++)
          distances[i][j] = MAXDIST_PREY;
   }
   
}

/****************************************/
/****************************************/

void CPreyNNController::Destroy() {
   m_neuralNet.Destroy();
}

void CPreyNNController::Stop() {
  m_pcWheels->SetLinearVelocity(0.0f,0.0f);
  //LOG<<"I was called"<<std::endl;
}

/****************************************/
/****************************************/

REGISTER_CONTROLLER(CPreyNNController, "prey_nn_controller")
