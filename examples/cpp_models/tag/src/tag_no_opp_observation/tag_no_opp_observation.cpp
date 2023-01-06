#include <queue>

#include "tag_no_opp_observation.h"
#include <despot/util/coord.h>
#include <despot/util/floor.h>

using namespace std;

namespace despot {

/* =============================================================================
 * Tag class
 * =============================================================================*/

TagNoOppObs::TagNoOppObs() {
	string map = string("mapSize = 5 10\n") + string("#####...##\n")
		+ string("#####...##\n") + string("#####...##\n")
		+ string("...........\n") + string("...........");
	istringstream iss(map);
	Init(iss);

	same_loc_obs_ = floor_.NumCells();
	obs_.resize(NumStates());
	for (int rob = 0; rob < floor_.NumCells(); rob++) {
		for (int opp = 0; opp < floor_.NumCells(); opp++) {
			int s = RobOppIndicesToStateIndex(rob, opp);
            obs_[s] = rob;
		}
	}
    robot_pos_unknown_ = false;
    init_state_value();
}

TagNoOppObs::TagNoOppObs(int unsuccessful_tag_reward): TagNoOppObs(){
    BaseTag::UNSUCCESSFUL_TAG_REWARD = unsuccessful_tag_reward;
}

TagNoOppObs::TagNoOppObs(string params_file) :
	BaseTag(params_file) {
	same_loc_obs_ = floor_.NumCells();
	obs_.resize(NumStates());
	for (int rob = 0; rob < floor_.NumCells(); rob++) {
		for (int opp = 0; opp < floor_.NumCells(); opp++) {
			int s = RobOppIndicesToStateIndex(rob, opp);
            obs_[s] = rob;
		}
	}
  robot_pos_unknown_ = false;
    init_state_value();
}


void TagNoOppObs::init_state_value() {
    // create sarsop_state_value_
    string sarsop_file_name = "sarsop_noObs.out";
    if (Globals::config.unsuccessful_reward != -1e10)
        sarsop_file_name = "sarsop_noObs" + to_string(Globals::config.unsuccessful_reward) + ".out";

    Insert_state_value_data(sarsop_file_name, sarsop_state_value_);

    // create VI_state_value_
    const_cast<TagNoOppObs*>(this)->ComputeOptimalPolicyUsingVI();  //compute value iteration
    for(int s = 0; s < NumStates(); s++)
        VI_state_value_.push_back(policy_[s].value);
}

bool TagNoOppObs::Step(State& state, double random_num, ACT_TYPE action, double& reward,
	OBS_TYPE& obs) const {
	bool terminal = BaseTag::Step(state, random_num, action, reward);

    obs = getObs(state, action);

	return terminal;
}

OBS_TYPE TagNoOppObs::getObs(const State& s, ACT_TYPE action) const{
    if(action == 4){
        const TagState& state = static_cast<const TagState&>(s);
        int rob = rob_[state.state_id];
        int opp = opp_[state.state_id];
        return (rob == opp ? same_loc_obs_ : rob);
    }
    return obs_[s.state_id];

}

double TagNoOppObs::ObsProb(OBS_TYPE obs, const State& s, ACT_TYPE a) const {
	return obs == getObs(s, a);
}

Belief* TagNoOppObs::ExactPrior() const {
	vector<State*> particles;
	for (int rob = 0; rob < floor_.NumCells(); rob++) {
		for (int opp = 0; opp < floor_.NumCells(); opp++) {
			TagState* state = static_cast<TagState*>(BaseTag::Allocate(
				RobOppIndicesToStateIndex(rob, opp),
				1.0 / floor_.NumCells() / floor_.NumCells()));
			particles.push_back(state);
		}
	}

	TagBelief* belief = new TagBelief(particles, this);
	belief->state_indexer(this);
	return belief;
}

Belief* TagNoOppObs::ApproxPrior() const {
	vector<State*> particles;

	int N = floor_.NumCells();
	double wgt = 1.0 / N / N;
	for (int rob = 0; rob < N; rob++) {
		for (int opp = 0; opp < N; opp++) {
			TagState* state = static_cast<TagState*>(BaseTag::Allocate(
				RobOppIndicesToStateIndex(rob, opp), wgt));
			particles.push_back(state);
		}
	}

	ParticleBelief* belief = new ParticleBelief(particles, this);
	belief->state_indexer(this);
	return belief;
}

Belief* TagNoOppObs::InitialBelief(const State* start, string type) const {
	Belief* prior = NULL;
	if (type == "EXACT") {
		prior = ExactPrior();
	} else if (type == "DEFAULT" || type == "PARTICLE") {
		prior = ApproxPrior();
	} else {
		cerr << "[Tag::InitialBelief] Unsupported belief type: " << type << endl;
		exit(0);
	}

	return prior;
}

void TagNoOppObs::Observe(const Belief* belief, ACT_TYPE action,  //TODO:related to AEMS - didnt change to no loaction observation
	map<OBS_TYPE, double>& obss) const {
	const vector<State*>& particles =
		static_cast<const ParticleBelief*>(belief)->particles();
	for (int i = 0; i < particles.size(); i++) {
		TagState* state = static_cast<TagState*>(particles[i]);
		const vector<State>& distribution = transition_probabilities_[GetIndex(
			state)][action];
		for (int i = 0; i < distribution.size(); i++) {
			const State& next = distribution[i];
			OBS_TYPE obs = obs_[next.state_id];
			double p = state->weight * next.weight;
			obss[obs] += p;
		}
	}
}

void TagNoOppObs::PrintObs(const State& state, OBS_TYPE obs, ostream& out) const {
	if (obs == floor_.NumCells()) {
		out << "On opponent" << endl;
	} else {
		Coord rob = floor_.GetCell(obs);
		out << "Rob at (" << rob.x << ", " << rob.y << ")" << endl;
	}
}

double TagNoOppObs::VI_state_value(State* state) const{
    return this->VI_state_value_[state->state_id];
}

double TagNoOppObs::Sarsop_state_value(State* state) const{
    return this->sarsop_state_value_[state->state_id];
}


} // namespace despot
