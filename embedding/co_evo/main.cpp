/* STL headers */
#include<deque>
/* GA-related headers */
#include <ga/ga.h>

/* ARGoS-related headers */
#include <argos3/core/simulator/simulator.h>
#include <argos3/core/simulator/loop_functions.h>

#include <loop_functions/co_evolution_loop_functions/co_evolution_loop_functions.h>

/****************************************/
/****************************************/
// #ifndef COMPETITORS
// #define COMPETITORS 1
// #endif

#ifndef GENERATIONS
#define GENERATIONS 500
#endif

#ifndef POPULATION
#define POPULATION 20
#endif

/****************************************/
/****************************************/
//bool prey = true;
const static uint8_t GENOME_SIZE_PREY = 116;
//const static uint8_t GENOME_SIZE_PREDATOR = 28;
//Containers for latest competitors
//std::deque<GARealGenome*> prey_competitors;
//std::deque<GARealGenome*> predator_competitors;
//GARealGenome* testGenome;
/*
 * Launch ARGoS to evaluate a genome of prey's population.
 */
float LaunchARGoS(GAGenome& c_genome) {
   //argos::LOG<<" LaunchARGos called by Prey "<<std::endl;//<<prey;

   /* Convert the received genome to the actual genome type */
   GARealGenome& cRealGenome = dynamic_cast<GARealGenome&>(c_genome);
   /* The CSimulator class of ARGoS is a singleton. Therefore, to
    * manipulate an ARGoS experiment, it is enough to get its instance.
    * This variable is declared 'static' so it is created
    * once and then reused at each call of this function.
    * This line would work also without 'static', but written this way
    * it is faster. */
   static argos::CSimulator& cSimulator = argos::CSimulator::GetInstance();
   /* Get a reference to the loop functions */
   static CCoEvolutionLoopFunctions& cLoopFunctions = dynamic_cast<CCoEvolutionLoopFunctions&>(cSimulator.GetLoopFunctions());
   /*
    * Run 5 trials and take the worst performance as final value.
    * Performance in this experiment is defined as the distance from the light.
    * Thus, we keep the max distance found.
    */
   //Real countKill = 0.0f;
   //for(size_t i = 0; i < COMPETITORS; ++i) {
      /* Tell the loop functions to get ready for the i-th trial */
      //cLoopFunctions.SetTrial(i);
      /* Reset the experiment.
       * This internally calls also CEvolutionLoopFunctions::Reset(). */
      cSimulator.Reset();
      /* Configure the controller with the genome */
      //cLoopFunctions.ConfigureFromGenome(cRealGenome);
      //if(prey)
	  	  cLoopFunctions.ConfigureFromGenome(cRealGenome);//,*predator_competitors[i]);
	    //else
	  	//  cLoopFunctions.ConfigureFromGenome(*prey_competitors[i],cRealGenome);
      /* Run the experiment */
      cSimulator.Execute();
      /* Update performance */
      //countKill = Max(countKill, cLoopFunctions.Performance());
   //}
   /* Return the result of the evaluation */
   //return countKill;
    return cLoopFunctions.Performance();
}

/*
 * Flush best individual in current prey generation
 */
void FlushBestPrey(const GARealGenome& c_genome,
               size_t un_generation) {
   std::ostringstream cOSS;
   cOSS << "preyData/best_prey_" << un_generation << ".dat";
   std::ofstream cOFS(cOSS.str().c_str(), std::ios::out | std::ios::trunc);
   cOFS << GENOME_SIZE_PREY // first write the number of values to dump
        << " "
        << c_genome    // then write the actual values
        << std::endl;
}

/*
 * Flush best individual in current predator generation
 */

// void FlushBestPredator(const GARealGenome& c_genome,
//                size_t un_generation) {
//    std::ostringstream cOSS;
//    cOSS << "predatorData/best_predator_" << un_generation << ".dat";
//    std::ofstream cOFS(cOSS.str().c_str(), std::ios::out | std::ios::trunc);
//    cOFS << GENOME_SIZE_PREDATOR // first write the number of values to dump
//         << " "
//         << c_genome    // then write the actual values
//         << std::endl;
// }

/****************************************/
/****************************************/

int main(int argc, char** argv) {
   /*
    * Initialize GALIB for prey
    */
   //argos::LOG<<" YOLO 1 "<<std::endl;
   /* Create an allele whose values can be in the range [-10,10] */
   GAAlleleSet<float> cAlleleSetPrey(-5.0f, 5.0f);
   /* Create a genome with 10 genes, using LaunchARGoS() to evaluate it */
   GARealGenome cGenomePrey(GENOME_SIZE_PREY, cAlleleSetPrey, LaunchARGoS);
   /* Create and configure a basic genetic algorithm using the genome */
   //FlushBestPrey(cGenomePrey,100);
   GASimpleGA preyGA(cGenomePrey);
   preyGA.minimize();                     // minimize the no.of of preys killed.
   preyGA.populationSize(POPULATION);              // population size for each generation
   preyGA.nGenerations(GENERATIONS);              // number of generations
   preyGA.pMutation(0.05f);               // prob of gene mutation
   preyGA.pCrossover(0.15f);              // prob of gene crossover
   preyGA.scoreFilename("evolution_prey.dat"); // filename for the result log
   preyGA.flushFrequency(1);              // log the results every generation




/****************************************/
/****************************************/
  //argos::LOG<<" YOLO 2 "<<std::endl;


   /*
    * Initialize GALIB for predator
    */
   /* Create an allele whose values can be in the range [-5,5] */
   // GAAlleleSet<float> cAlleleSetPredator(-5.0f, 5.0f);
   // /* Create a genome with 10 genes, using LaunchARGoS() to evaluate it */
   // GARealGenome cGenomePredator(GENOME_SIZE_PREDATOR, cAlleleSetPredator, LaunchARGoS);
   // /* Create and configure a basic genetic algorithm using the genome */
   // GASimpleGA predatorGA(cGenomePredator);
   // predatorGA.maximize();                     // maximize the no.of preys killed
   // predatorGA.populationSize(POPULATION);              // population size for each generation
   // predatorGA.nGenerations(GENERATIONS);              // number of generations
   // predatorGA.pMutation(0.05f);               // prob of gene mutation
   // predatorGA.pCrossover(0.15f);              // prob of gene crossover
   // predatorGA.scoreFilename("evolution_predator.dat"); // filename for the result log
   // predatorGA.flushFrequency(1);              // log the results every generation


/****************************************/
/****************************************/

   //argos::LOG<<" YOLO 3"<<std::endl;
   /*
    * Initialize ARGoS
    */
   /* The CSimulator class of ARGoS is a singleton. Therefore, to
    * manipulate an ARGoS experiment, it is enough to get its instance */
   argos::CSimulator& cSimulator = argos::CSimulator::GetInstance();
   /* Set the .argos configuration file
    * This is a relative path which assumed that you launch the executable
    * from argos3-examples (as said also in the README) */
   cSimulator.SetExperimentFileName("experiments/coevo.argos");
   /* Load it to configure ARGoS */
   cSimulator.LoadExperiment();
   
   //argos::argos::LOG<<" starting experiment "<<std::endl;
   //argos::argos::LOG << "Generation #" << preyGA.generation() << "...";
   //argos::argos::LOG << "No. of individuals:"<<preyGA.populationSize()<<std::endl;
   //testGenome = &dynamic_cast<GARealGenome&>(cGA.population().individual(0));
   //argos::argos::LOG << "Best individual:"<<preyGA.population().individual(0).score()<<std::endl;
   
   /*
    * Initialize the Competitors array with 5 random genomes 
	*/
   //argos::LOG<<" YOLO 4"<<std::endl;
  //   for (int i = 0; i < COMPETITORS; ++i){
		// prey_competitors.push_back(&cGenomePrey);
		// predator_competitors.push_back(&cGenomePredator);	
  //   }
    //argos::LOG<<" YOLO 5"<<std::endl;
   /*

    * Launch the evolution, setting the random seed
    */

   preyGA.initialize(12345);
    //argos::LOG<<" YOLO 6"<<std::endl;
   //predatorGA.initialize(12345);
    //argos::LOG<<" YOLO 7"<<std::endl;
   do {
      argos::LOG << "Generation #" << preyGA.generation() << "... Prey Evaluation";
      /* Step over the */
      preyGA.step();
      //set the flag to false
      //prey = false;
      //argos::LOG << "/ Predator Evaluation ";
      //predatorGA.step();
      //prey = true;
      argos::LOG << "done.";
      /*
       *  Update the Competitors array with latest generation's best individual
       */
      //prey_competitors.push_back(&dynamic_cast<GARealGenome&>(preyGA.population().best()));
      //prey_competitors.pop_front();
      //argos::LOG<<prey_competitors.size()<<std::endl;
	    //predator_competitors.push_back(&dynamic_cast<GARealGenome&>(predatorGA.population().best()));
      //predator_competitors.pop_front();  
      //argos::LOG<<predator_competitors.size()<<std::endl;
      /*
       *  Flush the necessary data to the files
       */  
      argos::LOG << "   Flushing...";
      if(preyGA.generation() % preyGA.flushFrequency() == 0) {
         /* Flush scores */
         preyGA.flushScores();
         /* Flush best individual in current generation */
         FlushBestPrey(dynamic_cast<const GARealGenome&>(preyGA.population().best()),
                   preyGA.generation());
      }
      // if(predatorGA.generation() % predatorGA.flushFrequency() == 0) {
      //    /* Flush scores */
      //    predatorGA.flushScores();
      //    /* Flush best individual in current generation */ 
      //    FlushBestPredator(dynamic_cast<const GARealGenome&>(predatorGA.population().best()),
      //              predatorGA.generation());
      // }
      argos::LOG << "done.";
      argos::LOG << std::endl;
      argos::LOG.Flush();
   }
   //while(! (preyGA.done() || predatorGA.done()));
   while(!preyGA.done());
   /*
    * Dispose of ARGoS stuff
    */
   cSimulator.Destroy();

   /* All is OK */
   return 0;
}



/****************************************/
/****************************************/
