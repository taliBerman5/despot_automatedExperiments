#include <despot/planner.h>
#include "noisy_laser_tag.h"

using namespace despot;

class MyPlanner: public Planner {
public:
  MyPlanner() {
  }

DSPOMDP* InitializeModel(option::Option* options) {
    DSPOMDP* model = new NoisyLaserTag() ;
    if (options[E_PARAMS_FILE])
        model = new NoisyLaserTag(options[E_PARAMS_FILE].arg);
    else if (options[E_UNSUCCESSFUL_REWARD])
        model = new NoisyLaserTag(atoi(options[E_UNSUCCESSFUL_REWARD].arg));
    return model;
}

  World* InitializeWorld(std::string&  world_type, DSPOMDP* model, option::Option* options)
  {
      return InitializePOMDPWorld(world_type, model, options);
  }

  void InitializeDefaultParameters() {
    Globals::config.pruning_constant = 0.01;
  }

  std::string ChooseSolver(){
	  return "DESPOT";
  }
};

int main(int argc, char* argv[]) {
  return MyPlanner().RunEvaluation(argc, argv);
}
