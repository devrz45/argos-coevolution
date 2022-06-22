#include "co_evolution_loop_functions.h"

/****************************************/
/****************************************/

CCoEvolutionLoopFunctions::CCoEvolutionLoopFunctions() :
   m_unCurrentTrial(0),
   m_pfControllerParams(new Real[PREY_GENOME_SIZE]),
   m_pcRNG(NULL),
   countKilled(0) {}

/****************************************/
/****************************************/

CCoEvolutionLoopFunctions::~CCoEvolutionLoopFunctions() {
   delete[] m_pfControllerParams;
}

/****************************************/
/****************************************/

void CCoEvolutionLoopFunctions::Init(TConfigurationNode& t_node) {
   /*
    * Create the random number generator
    */
   m_pcRNG = CRandom::CreateRNG("argos");

   /*
    * Find the foot-bot and get a reference to its controller
    */
   CSpace::TMapPerType& m_cFootbots = GetSpace().GetEntitiesByType("foot-bot");

   for(CSpace::TMapPerType::iterator it = m_cFootbots.begin(); it != m_cFootbots.end(); ++it) {
      /* Get handle to foot-bot entity and controller */
      CFootBotEntity& cFootBot = *any_cast<CFootBotEntity*>(it->second);
      if(cFootBot.GetId().compare(0,8,"predator")==0) {
      		//m_predatorBots.push_back(&cFootBot);
			   m_predatorControllers.push_back(&dynamic_cast<CPredatorNNController&>(cFootBot.GetControllableEntity().GetController()));
	  }
      else {
      		m_preyBots.push_back(&cFootBot);
      		m_preyControllers.push_back(&dynamic_cast<CPreyNNController&>(cFootBot.GetControllableEntity().GetController()));
      }
   }

   Position.Set(-4,-4,0);
   Orientation.FromEulerAngles(CRadians::ZERO,CRadians::ZERO,CRadians::ZERO);
}

/****************************************/
/****************************************/
void CCoEvolutionLoopFunctions::PostStep()
{
	for(size_t i = 0; i < m_preyControllers.size(); i++){
   	  if(m_preyControllers[i]->getLife()==0 && ! m_preyControllers[i]->IsDead()) {
   	  	MoveEntity(m_preyBots[i]->GetEmbodiedEntity(),Position,Orientation);
         m_preyControllers[i]->Stop();
         m_preyControllers[i]->SetDead();
         countKilled++;
         //LOG<<"no.Killed "<<countKilled<<std::endl;
         Position.Set(Position.GetX()+0.20f,-4,0);
        }
	}
}

/****************************************/
/****************************************/

void CCoEvolutionLoopFunctions::Reset() {
   countKilled = 0;
   for(size_t i = 0; i < m_preyControllers.size(); i++){
   	  if(m_preyControllers[i]->IsDead())
   	  	m_preyControllers[i]->SetAlive();
	}
   Position.Set(-4,-4,0);
   //Orientation.FromEulerAngles(CRadians::ZERO,CRadians::ZERO,CRadians::ZERO);
}

/****************************************/
/****************************************/

void CCoEvolutionLoopFunctions::ConfigureFromGenome(const GARealGenome& prey_genome){//,GARealGenome& predator_genome) {
   /* Copy the genes into the NN parameter buffer */
   for(size_t i = 0; i < PREY_GENOME_SIZE; ++i)
      m_pfControllerParams[i] = prey_genome[i];
   /* Set the NN parameters */
   for(size_t i = 0; i < m_preyControllers.size(); i++)
   	  m_preyControllers[i]->GetPerceptron().SetOnlineParameters(PREY_GENOME_SIZE, m_pfControllerParams);
   /* Copy the genes into the NN parameter buffer */
   /*for(size_t i = 0; i < PREDATOR_GENOME_SIZE; ++i)
      m_pfControllerParams[i] = predator_genome[i];
   /* Set the NN parameters */
   //m_predatorController->GetPerceptron().SetOnlineParameters(PREDATOR_GENOME_SIZE, m_pfControllerParams);  
}

/****************************************/
/****************************************/

Real CCoEvolutionLoopFunctions::Performance() {
   /* The performance is simply the no of preys killed */
   //LOG<<"Count:"<<countKilled<<std::endl;
   return countKilled;
}

/****************************************/
/****************************************/

REGISTER_LOOP_FUNCTIONS(CCoEvolutionLoopFunctions, "co_evolution_loop_functions")