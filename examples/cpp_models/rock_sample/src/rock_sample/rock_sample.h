#ifndef ROCKSAMPLE_H
#define ROCKSAMPLE_H

#include <despot/interface/pomdp.h>
#include <despot/core/mdp.h>
#include "base/base_rock_sample.h"
#include <despot/util/coord.h>
#include <despot/util/grid.h>

namespace despot {

/* =============================================================================
 * RockSample class
 * =============================================================================*/

class RockSample: public BaseRockSample {
protected:
    std::vector<double> VI_state_value_;
    std::vector<double> sarsop_state_value_;
    void init_state_value();
public:
	RockSample(std::string map);
	RockSample(int size, int rocks);

	bool Step(State& state, double rand_num, ACT_TYPE action, double& reward,
		OBS_TYPE& obs) const;
	int NumActions() const;
	double ObsProb(OBS_TYPE obs, const State& state, ACT_TYPE action) const;
	void PrintObs(const State& state, OBS_TYPE observation,
		std::ostream& out = std::cout) const;
    double VI_state_value( State* state) const;
    double Sarsop_state_value( State* state) const;
    void InitializeTransitions();

};

} // namespace despot

#endif
