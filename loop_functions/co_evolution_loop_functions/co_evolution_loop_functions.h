#ifndef CO_EVOLUTION_LOOP_FUNCTIONS_H
#define CO_EVOLUTION_LOOP_FUNCTIONS_H
/* STL headers*/
#include <deque>
#include <string>
/* The NN controller */
#include <controllers/prey_nn/prey_nn_controller.h>
#include <controllers/predator_nn/predator_nn_controller.h>
/* ARGoS-related headers */
#include <argos3/core/simulator/loop_functions.h>
#include <argos3/core/utility/math/rng.h>
#include <argos3/plugins/robots/foot-bot/simulator/footbot_entity.h>

/* GA-related headers */
#include <ga/ga.h>
#include <ga/GARealGenome.h>
#include <ga/GARealGenome.C> // this is necessary!

/****************************************/
/****************************************/

/*
 * The size of the prey genome.
 * CTRNN : W = L1 ( 25 + 8 + 1 ) * L2 ( 4 ) + L2 ( 4 + 1 ) * L3 ( 5 )  
 * FNN : W = L1 ( 20 + 8 + 1 ) * L2 ( 4 )
 */
static const size_t PREY_GENOME_SIZE = 116;

/*
 * The size of the predator genome.
 * FNN : W = L1 ( 5 + 8 + 1 ) * L2 ( 2 )   
 */
static const size_t PREDATOR_GENOME_SIZE = 28;


/****************************************/
/****************************************/

using namespace argos;

class CCoEvolutionLoopFunctions : public CLoopFunctions {

public:

   CCoEvolutionLoopFunctions();
   virtual ~CCoEvolutionLoopFunctions();

   virtual void Init(TConfigurationNode& t_node);
   virtual void Reset();
   virtual void PostStep();

   /* Called by the evolutionary algorithm to set the current trial */
   inline void SetTrial(size_t un_trial) {
      m_unCurrentTrial = un_trial;
   }

   /* Configures the robot controller from the genome */
   void ConfigureFromGenome(const GARealGenome& prey_genome);//,GARealGenome& predator_genome);

   /* Calculates the performance of the robot in a trial */
   Real Performance();

private:

   
   size_t m_unCurrentTrial;
   std::deque<CFootBotEntity*> m_preyBots;
   CFootBotEntity* m_predatorBot;
   std::deque<CPreyNNController*> m_preyControllers;
   std::deque<CPredatorNNController*> m_predatorControllers;
   Real* m_pfControllerParams;
   CRandom::CRNG* m_pcRNG;
   Real countKilled;
   /* Change these position values if the arena size is ever changed */
   CVector3 Position;
   CQuaternion Orientation;

};

#endif
