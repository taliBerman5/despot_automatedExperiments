#ifndef TAGNOOPPOBS_H
#define TAGNOOPPOBS_H

#include <despot/interface/pomdp.h>
#include <despot/core/mdp.h>
#include <despot/util/coord.h>
#include <despot/util/floor.h>
#include "base/base_tag.h"

namespace despot {

/* =============================================================================
 * Tag class
 * =============================================================================*/

class TagNoOppObs: public BaseTag {
private:
  std::vector<OBS_TYPE> obs_;
    std::vector<double> VI_state_value_;
    std::vector<double> sarsop_state_value_;
    void init_state_value();
public:
    TagNoOppObs();
    TagNoOppObs(int unsuccessful_tag_reward);
    TagNoOppObs(std::string params_file);

	bool Step(State& state, double random_num, ACT_TYPE action, double& reward,
		OBS_TYPE& obs) const;

    OBS_TYPE getObs(const State& s, ACT_TYPE action) const; //TB
	double ObsProb(OBS_TYPE obs, const State& state, ACT_TYPE action) const;

	Belief* ExactPrior() const;
	Belief* ApproxPrior() const;
	Belief* InitialBelief(const State* start, std::string type = "DEFAULT") const;

	void Observe(const Belief* belief, ACT_TYPE action,
		std::map<OBS_TYPE, double>& obss) const;

	void PrintObs(const State& state, OBS_TYPE obs, std::ostream& out = std::cout) const;

    double VI_state_value( State* state) const;
    double Sarsop_state_value( State* state) const;



};

} // namespace despot

#endif
