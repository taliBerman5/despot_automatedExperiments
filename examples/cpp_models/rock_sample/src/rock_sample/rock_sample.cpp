#include "rock_sample.h"

using namespace std;

namespace despot {

/* =============================================================================
 * RockSample class
 * =============================================================================*/

RockSample::RockSample(string map) :
	BaseRockSample(map) {
	half_efficiency_distance_ = 20;
    InitializeTransitions();  //TODO: make sure this is correct
    init_state_value();
}

RockSample::RockSample(int size, int rocks) :
	BaseRockSample(size, rocks) {
	half_efficiency_distance_ = 20;
    InitializeTransitions();
    init_state_value();
}

RockSample::RockSample(int size, int rocks, double unsuccessful_sample_reward) :
        BaseRockSample(size, rocks) {
    BaseRockSample::UNSUCCESSFUL_SAMPLE_REWARD = unsuccessful_sample_reward;
    half_efficiency_distance_ = 20;
    InitializeTransitions();
    init_state_value();
}


void RockSample::init_state_value() {
    Insert_state_value_data("examples/cpp_models/rock_sample/sarsop.out", sarsop_state_value_);

    // create VI_state_value_
    const_cast<RockSample*>(this)->ComputeOptimalPolicyUsingVI();  //compute value iteration
    for(int s = 0; s < NumStates(); s++)
        VI_state_value_.push_back(policy_[s].value);
}

bool RockSample::Step(State& state, double rand_num, ACT_TYPE action, double& reward,
	OBS_TYPE& obs) const {
	RockSampleState& rockstate = static_cast<RockSampleState&>(state);
	reward = 0;
	obs = E_NONE;

	if (action < E_SAMPLE) { // Move
		switch (action) {
		case Compass::EAST:
			if (GetX(&rockstate) + 1 < size_) {
				IncX(&rockstate);
				break;
			} else {
				reward = +10;
				return true;
			}

		case Compass::NORTH:
			if (GetY(&rockstate) + 1 < size_)
				IncY(&rockstate);
			else
				reward = -100;
			break;

		case Compass::SOUTH:
			if (GetY(&rockstate) - 1 >= 0)
				DecY(&rockstate);
			else
				reward = -100;
			break;

		case Compass::WEST:
			if (GetX(&rockstate) - 1 >= 0)
				DecX(&rockstate);
			else
				reward = -100;
			break;
		}
	}

	if (action == E_SAMPLE) { // Sample
		int rock = grid_(GetRobPosIndex(&rockstate));
		if (rock >= 0) {
			if (GetRock(&rockstate, rock))
				reward = +10;
			else
				reward = BaseRockSample::UNSUCCESSFUL_SAMPLE_REWARD;
			SampleRock(&rockstate, rock);
		} else {
			reward = -100;
		}
	}

	if (action > E_SAMPLE) { // Sense
		int rock = action - E_SAMPLE - 1;
		assert(rock < num_rocks_);
		obs = GetObservation(rand_num, rockstate, rock);
	}

	// assert(reward != -100);
	return false;
}

int RockSample::NumActions() const {
	return num_rocks_ + 5;
}

double RockSample::ObsProb(OBS_TYPE obs, const State& state, ACT_TYPE action) const {
	if (action <= E_SAMPLE)
		return obs == E_NONE;

	if (obs != E_GOOD && obs != E_BAD)
		return 0;

	const RockSampleState& rockstate =
		static_cast<const RockSampleState&>(state);

	int rock = action - E_SAMPLE - 1;
	double distance = Coord::EuclideanDistance(GetRobPos(&rockstate),
		rock_pos_[rock]);
	double efficiency = (1 + pow(2, -distance / half_efficiency_distance_))
		* 0.5;

	return
		((GetRock(&rockstate, rock) & 1) == obs) ? efficiency : (1 - efficiency);
}

void RockSample::PrintObs(const State& state, OBS_TYPE observation,
	ostream& out) const {
	switch (observation) {
	case E_NONE:
		out << "None" << endl;
		break;
	case E_GOOD:
		out << "Good" << endl;
		break;
	case E_BAD:
		out << "Bad" << endl;
		break;
	}
}

double RockSample::VI_state_value(State* state) const{
    return this->VI_state_value_[state->state_id];


}double RockSample::Sarsop_state_value(State* state) const{
    return this->sarsop_state_value_[state->state_id];
}


void RockSample::InitializeTransitions() {
    int num_states = NumStates(), num_actions = NumActions();
    transition_probabilities_.resize(num_states);
    for (int s = 0; s < num_states; s++) {
        transition_probabilities_[s].resize(num_actions);
        for (int a = 0; a < num_actions; a++) {
            State state;
            state.state_id = NextState(s, a);
            state.weight = 1.0;
            transition_probabilities_[s][a].push_back(state);
        }
    }
}

} // namespace despot
