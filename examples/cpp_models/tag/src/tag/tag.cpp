#include <queue>

#include "tag.h"
#include <despot/util/coord.h>
#include <despot/util/floor.h>

using namespace std;

namespace despot {

/* =============================================================================
 * Tag class
 * =============================================================================*/

Tag::Tag() {
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
			obs_[s] = (rob == opp ? same_loc_obs_ : rob);
		}
	}
    robot_pos_unknown_ = false;
    init_state_value();
}

Tag::Tag(int unsuccessful_tag_reward): Tag(){
    BaseTag::UNSUCCESSFUL_TAG_REWARD = unsuccessful_tag_reward;
}

Tag::Tag(string params_file) :
	BaseTag(params_file) {
	same_loc_obs_ = floor_.NumCells();
	obs_.resize(NumStates());
	for (int rob = 0; rob < floor_.NumCells(); rob++) {
		for (int opp = 0; opp < floor_.NumCells(); opp++) {
			int s = RobOppIndicesToStateIndex(rob, opp);
			obs_[s] = (rob == opp ? same_loc_obs_ : rob);
		}
	}
    robot_pos_unknown_ = false;
    init_state_value();
}

void Tag::Insert_state_value_data(string file_name, vector<double>& state_value){
    fstream newfile;
    newfile.open(file_name,ios::in);
    if (newfile.is_open()) {   //checking whether the file is open
        string tp;
        vector<double> alpha_vec;
        while (getline(newfile, tp)) {
            std::stringstream iss( tp );
            double number;
            iss >> number;
            state_value.push_back(number);
        }
        newfile.close();
    }
}


void Tag::init_state_value() {
    Insert_state_value_data("sarsop.out", sarsop_state_value_);
    Insert_state_value_data("VI.out", VI_state_value_);
}

bool Tag::Step(State& state, double random_num, ACT_TYPE action, double& reward,
	OBS_TYPE& obs) const {
	bool terminal = BaseTag::Step(state, random_num, action, reward);

	obs = obs_[state.state_id];

	return terminal;
}

double Tag::ObsProb(OBS_TYPE obs, const State& s, ACT_TYPE a) const {
	const TagState& state = static_cast<const TagState&>(s);

	return obs == obs_[state.state_id];
}

Belief* Tag::ExactPrior() const {
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

Belief* Tag::ApproxPrior() const {
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

Belief* Tag::InitialBelief(const State* start, string type) const {
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

void Tag::Observe(const Belief* belief, ACT_TYPE action,
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

void Tag::PrintObs(const State& state, OBS_TYPE obs, ostream& out) const {
	if (obs == floor_.NumCells()) {
		out << "On opponent" << endl;
	} else {
		Coord rob = floor_.GetCell(obs);
		out << "Rob at (" << rob.x << ", " << rob.y << ")" << endl;
	}
}

double Tag::VI_state_value(State* state) const{
    return this->VI_state_value_[state->state_id];


}double Tag::Sarsop_state_value(State* state) const{
    return this->sarsop_state_value_[state->state_id];
}


} // namespace despot
