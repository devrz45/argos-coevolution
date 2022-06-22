#include "predator_nn_controller.h"

/****************************************/
/****************************************/

static CRange<Real> NN_OUTPUT_RANGE(0.0f, 1.0f);
static CRange<Real> WHEEL_ACTUATION_RANGE(-5.0f, 5.0f);
static CRange<Real> CAMERA_DISTANCE_RANGE(0.0f, MAXDIST_PREDATOR);
static CRange<Real> NN_INPUT_RANGE(0.00f, 1.00f);

/****************************************/
/****************************************/

CPredatorNNController::CPredatorNNController(): 
        m_pcWheels(NULL),
        m_pcProximity(NULL),
        m_pcCamera(NULL),
        m_pcLEDs(NULL),
        m_fWheelVelocity(5.0f),
        offset(2),
        frontAngle(6),
        midAngle(17),
        farAngle(29),
        leftWall(false),
        rightWall(false),
        counter(0) {}

/****************************************/
/****************************************/

CPredatorNNController::~CPredatorNNController() {
}

/****************************************/
/****************************************/

void CPredatorNNController::Init(TConfigurationNode& t_node) {
   /*
    * Get sensor/actuator handles
    */
   try {
      m_pcWheels    = GetActuator<CCI_DifferentialSteeringActuator>("differential_steering");
      m_pcProximity = GetSensor  <CCI_FootBotProximitySensor      >("footbot_proximity"    );
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
      GetNodeAttributeOrDefault(t_node, "front", frontAngle, frontAngle);
      GetNodeAttributeOrDefault(t_node, "mid", midAngle, midAngle);
      GetNodeAttributeOrDefault(t_node, "far", farAngle, farAngle);
      GetNodeAttributeOrDefault(t_node, "velocity", m_fWheelVelocity, m_fWheelVelocity);
      GetNodeAttributeOrDefault(t_node, "offset", offset, offset);

      m_centralAngleRange.Set(-ToRadians(frontAngle), ToRadians(frontAngle));
      m_lMidPerAngleRange.Set(ToRadians(frontAngle), ToRadians(midAngle));
      m_lFarPerAngleRange.Set(ToRadians(midAngle), ToRadians(farAngle));
      m_rMidPerAngleRange.Set(-ToRadians(midAngle), -ToRadians(frontAngle));
      m_rFarPerAngleRange.Set(-ToRadians(farAngle), -ToRadians(midAngle));

      WHEEL_ACTUATION_RANGE.Set(-m_fWheelVelocity,m_fWheelVelocity);
   }
   catch(CARGoSException& ex) {
      THROW_ARGOSEXCEPTION_NESTED("Error initializing parameters", ex);
   }

   /* Initialize the perceptron */
   // try {
   //    m_neuralNet.Init(t_node);
   // }
   // catch(CARGoSException& ex) {
   //    THROW_ARGOSEXCEPTION_NESTED("Error initializing the perceptron network", ex);
   // }

   // Turn the led on for the predator
   m_pcLEDs->SetSingleColor(12,CColor::RED);
   // Turn on the camera
   m_pcCamera->Enable();
   // Initialize sector wise data  
   for(uint8_t i=0 ; i<SECTORS ; i++)
          distances[i] = MAXDIST_PREDATOR;
   //LOG<<"AngleRange:"<<m_centralAngleRange.GetMin().GetValue()<<"<->"<<m_centralAngleRange.GetMax().GetValue()<<std::endl;
   //LOG<<"WithinRange"<<m_rFarPerAngleRange.WithinMinBoundIncludedMaxBoundIncluded(CRadians(-0.41302))<<std::endl;
   //LOG<<"WithinRange"<<m_rMidPerAngleRange.WithinMinBoundIncludedMaxBoundIncluded(CRadians(-0.106775))<<std::endl;
   //LOG<<"AngleRange:"<<m_lMidPerAngleRange.GetMin().GetValue()<<"<->"<<m_lMidPerAngleRange.GetMax().GetValue()<<std::endl;
   //LOG<<"AngleRange:"<<m_lFarPerAngleRange.GetMin().GetValue()<<"<->"<<m_lFarPerAngleRange.GetMax().GetValue()<<std::endl;
   //LOG<<"AngleRange:"<<m_rMidPerAngleRange.GetMin().GetValue()<<"<->"<<m_rMidPerAngleRange.GetMax().GetValue()<<std::endl;
   //LOG<<"AngleRange:"<<m_rFarPerAngleRange.GetMin().GetValue()<<"<->"<<m_rFarPerAngleRange.GetMax().GetValue()<<std::endl;
}

/****************************************/
/****************************************/

void CPredatorNNController::ControlStep() {
   /* Get sensory data */
   const CCI_FootBotProximitySensor::TReadings& tProx = m_pcProximity->GetReadings();
   const CCI_ColoredBlobOmnidirectionalCameraSensor::SReadings& sCamera = m_pcCamera->GetReadings();
   
   Real minDistance = MAXDIST_PREDATOR;
   uint8_t minSector = 5;
   CCI_ColoredBlobOmnidirectionalCameraSensor::SBlob* cblob;
   /* Structure the readings from omnidirectional camera */
   if(! sCamera.BlobList.empty())
   {
      for(size_t i = 0; i < sCamera.BlobList.size(); ++i) {
        cblob = sCamera.BlobList[i];
        //LOG<<"Blob:"<<i<<" Color: "<<cblob->Color<<" distance: "<<cblob->Distance<<" angle: "<<cblob->Angle.GetValue()<<std::endl;
        uint8_t sector=5;

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

        if(sector!=5)
        {
          if (cblob->Color==CColor::GREEN && (distances[sector]>cblob->Distance))
          { 
             distances[sector] = cblob->Distance;
             if(cblob->Distance<minDistance)
              {
                minSector = sector;
                minDistance = cblob->Distance;
              }
          }
          //LOG<<"sector:"<<sector;
        }
      }
    }
  for(uint8_t i=0 ; i<SECTORS ; i++)
    distances[i] = MAXDIST_PREDATOR;

   /* Fill NN inputs from omnidirection camera */
   // uint16_t k=0;
   // for(uint8_t i=0 ; i<SECTORS ; i++)
   // {
   //      CAMERA_DISTANCE_RANGE.MapValueIntoRange(mapped[k],distances[i],NN_INPUT_RANGE);
   //      m_neuralNet.SetInput(k,1.0f - mapped[k]);
   //      ++k;    
   // }

   // /* Fill NN inputs from proximity sensor */
   // for(uint8_t i = 0; i < tProx.size(); i+=3) {
   //    m_neuralNet.SetInput(k, tProx[i].Value);
   // }
   //  Compute NN outputs 
   // m_neuralNet.ComputeOutputs();
   // /*
   //  * Apply NN outputs to actuation
   //  */
   // NN_OUTPUT_RANGE.MapValueIntoRange(
   //    m_fLeftSpeed,               
   //    m_neuralNet.GetOutput(0), 
   //    WHEEL_ACTUATION_RANGE       
   //    );
   // NN_OUTPUT_RANGE.MapValueIntoRange(
   //    m_fRightSpeed,              
   //    m_neuralNet.GetOutput(1), 
   //    WHEEL_ACTUATION_RANGE       
   //    );
   //LOG<<" Min Sector:"<<minSector<<std::endl;
   switch(minSector)
   {
    case 0:
      m_pcWheels->SetLinearVelocity(m_fWheelVelocity,m_fWheelVelocity);
      break;
    case 1:
      m_pcWheels->SetLinearVelocity(m_fWheelVelocity - offset ,m_fWheelVelocity);
      break;
    case 2:
      m_pcWheels->SetLinearVelocity(m_fWheelVelocity - offset*2,m_fWheelVelocity);
      break;
    case 3:
      m_pcWheels->SetLinearVelocity(m_fWheelVelocity ,m_fWheelVelocity - offset);
      break; 
    case 4:
      m_pcWheels->SetLinearVelocity(m_fWheelVelocity,m_fWheelVelocity - offset*2);
    case 5:
      if(counter!=0)
      {
        m_pcWheels->SetLinearVelocity(-m_fWheelVelocity,m_fWheelVelocity);
        counter -=1;
      }
      else
      {
      if(tProx[0].Value==1 || tProx[1].Value==1 || tProx[2].Value==1 || tProx[3].Value==1 ) 
        leftWall =true;
      else
        leftWall = false;
      if( tProx[23].Value==1 || tProx[22].Value==1 || tProx[21].Value==1 || tProx[23].Value==1)
        rightWall = true; 
      else
        rightWall = false;
      if(leftWall && rightWall)
      {  counter=20;
        //LOG<<"Im reached"<<std::endl; 
        break;
      }
      else if(rightWall)
         m_pcWheels->SetLinearVelocity(-m_fWheelVelocity,m_fWheelVelocity); 
      else if(leftWall) 
         m_pcWheels->SetLinearVelocity(m_fWheelVelocity,-m_fWheelVelocity);
      else
        m_pcWheels->SetLinearVelocity(m_fWheelVelocity,m_fWheelVelocity);
      }
      break;
   }
   // m_pcWheels->SetLinearVelocity(
   //    m_fLeftSpeed,
   //    m_fRightSpeed);
}

/****************************************/
/****************************************/

void CPredatorNNController::Reset() {
   //m_neuralNet.Reset();
   // Turn the led on for the prey
   m_pcLEDs->SetSingleColor(12,CColor::RED);
   // Initialize sector wise data
   for(uint8_t i=0 ; i<SECTORS ; i++)
          distances[i] = MAXDIST_PREDATOR;
}

/****************************************/
/****************************************/

void CPredatorNNController::Destroy() {
   //m_neuralNet.Destroy();
}

/****************************************/
/****************************************/

REGISTER_CONTROLLER(CPredatorNNController, "predator_nn_controller")
