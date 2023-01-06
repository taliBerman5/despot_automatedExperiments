#ifndef TAG_H
#define TAG_H

#include <despot/interface/pomdp.h>
#include <despot/core/mdp.h>
#include <despot/util/coord.h>
#include <despot/util/floor.h>
#include "base/base_tag.h"

namespace despot {

/* =============================================================================
 * Tag class
 * =============================================================================*/

class Tag: public BaseTag {
private:
    std::vector<OBS_TYPE> obs_;
    std::vector<double> VI_state_value_;
    std::vector<double> sarsop_state_value_;
    void init_state_value();
    void Insert_state_value_data(std::string file_name, std::vector<double>& state_value);
public:
	Tag();
    Tag(int unsuccessful_tag_reward);
	Tag(std::string params_file);

	bool Step(State& state, double random_num, ACT_TYPE action, double& reward,
		OBS_TYPE& obs) const;

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
