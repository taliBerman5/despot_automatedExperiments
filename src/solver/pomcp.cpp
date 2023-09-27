#include <despot/solver/pomcp.h>
#include <despot/util/logging.h>
#include <set>
#include <numeric>

using namespace std;

namespace despot {

/* =============================================================================
 * POMCPPrior class
 * =============================================================================*/

POMCPPrior::POMCPPrior(const DSPOMDP* model) :
	model_(model) {
	exploration_constant_ = (model->GetMaxReward()
		- model->GetBestAction().value);
}

POMCPPrior::~POMCPPrior() {
}

const vector<int>& POMCPPrior::preferred_actions() const {
	return preferred_actions_;
}

const vector<int>& POMCPPrior::legal_actions() const {
	return legal_actions_;
}

ACT_TYPE POMCPPrior::GetAction(const State& state) {
	ComputePreference(state);

	if (preferred_actions_.size() != 0)
		return Random::RANDOM.NextElement(preferred_actions_);

	if (legal_actions_.size() != 0)
		return Random::RANDOM.NextElement(legal_actions_);

	return Random::RANDOM.NextInt(model_->NumActions());
}

/* =============================================================================
 * UniformPOMCPPrior class
 * =============================================================================*/

UniformPOMCPPrior::UniformPOMCPPrior(const DSPOMDP* model) :
	POMCPPrior(model) {
}

UniformPOMCPPrior::~UniformPOMCPPrior() {
}

void UniformPOMCPPrior::ComputePreference(const State& state) {
}

/* =============================================================================
 * POMCP class
 * =============================================================================*/

POMCP::POMCP(const DSPOMDP* model, POMCPPrior* prior, Belief* belief) :
	Solver(model, belief),
	root_(NULL) {
	reuse_ = false;
	prior_ = prior;
	assert(prior_ != NULL);
    geo_.param(geometric_distribution<int>::param_type(1.0 / Globals::config.search_depth));
    search_depth_ = Globals::config.search_depth;

    //set leaf_heuristic function
    double (*rolloutFunc)(State*, int, const DSPOMDP*, POMCPPrior*, int) = Rollout;
    double (*sarsopFunc)(State*, int, const DSPOMDP*, POMCPPrior*, int) = Sarsop_heuristic;
    double (*viFunc)(State*, int, const DSPOMDP*, POMCPPrior*, int) = Value_iteration_heuristic;
    leaf_heuristic_ = Globals::config.leaf_heuristic == "SARSOP" ? sarsopFunc : Globals::config.leaf_heuristic == "VI" ? viFunc : rolloutFunc;

    //set simulate function
    double (*simulate_func)(State*, VNode*, const DSPOMDP*,
                        POMCPPrior*, int, double (State*, int, const DSPOMDP*, POMCPPrior*, int)) = Simulate;
    double (*Check_default_policy_Simulate_func)(State*, VNode*, const DSPOMDP*,
                            POMCPPrior*, int, double (State*, int, const DSPOMDP*, POMCPPrior*, int)) = Check_default_policy_Simulate;
    simulate_ = Globals::config.check_default_policy ? Check_default_policy_Simulate_func : simulate_func;
}

void POMCP::reuse(bool r) {
	reuse_ = r;
}

ValuedAction POMCP::Search(double timeout) {
	double start_cpu = clock(), start_real = get_time_second();

	if (root_ == NULL) {
		State* state = belief_->Sample(1)[0];
		root_ = CreateVNode(0, state, prior_, model_);
		model_->Free(state);
	}

	int hist_size = history_.Size();
	bool done = false;
	int num_sims = 0;
	while (true) {
		vector<State*> particles = belief_->Sample(1000);
		for (int i = 0; i < particles.size(); i++) {
			State* particle = particles[i];
			logd << "[POMCP::Search] Starting simulation " << num_sims << endl;

            if(Globals::config.geometric_search_depth) //sets the search depth according to a sample from geometric distribution
                search_depth_ = geo_(generator_);

            simulate_(particle, root_, model_, prior_, search_depth_, leaf_heuristic_);
 
			num_sims++;
			logd << "[POMCP::Search] " << num_sims << " simulations done" << endl;
			history_.Truncate(hist_size);

			if ((clock() - start_cpu) / CLOCKS_PER_SEC >= timeout) {
				done = true;
				break;
			}
		}

		for (int i = 0; i < particles.size(); i++) {
			model_->Free(particles[i]);
		}

		if (done)
			break;
	}

	ValuedAction astar = OptimalAction(root_);

	logi << "[POMCP::Search] Search statistics" << endl
		<< "OptimalAction = " << astar << endl 
		<< "# Simulations = " << root_->count() << endl
		<< "Time: CPU / Real = " << ((clock() - start_cpu) / CLOCKS_PER_SEC) << " / " << (get_time_second() - start_real) << endl
		<< "# active particles = " << model_->NumActiveParticles() << endl
		<< "Tree size = " << root_->Size() << endl;

	if (astar.action == -1) {
		for (ACT_TYPE action = 0; action < model_->NumActions(); action++) {
			cout << "action " << action << ": " << root_->Child(action)->count()
				<< " " << root_->Child(action)->value() << endl;
		}
	}

	// delete root_;
	return astar;
}


ValuedAction POMCP::simSearch(int num_simulates) {
    double start_cpu = clock(), start_real = get_time_second();

    if (root_ == NULL) {
        State* state = belief_->Sample(1)[0];
        root_ = CreateVNode(0, state, prior_, model_);
        model_->Free(state);
    }

    int hist_size = history_.Size();
    bool done = false;
    int num_sims = 0;
    while (true) {
        vector<State*> particles = belief_->Sample(1000);
        for (int i = 0; i < particles.size(); i++) {
            State* particle = particles[i];
            logd << "[POMCP::Search] Starting simulation " << num_sims << endl;

            if(Globals::config.geometric_search_depth) //sets the search depth according to a sample from geometric distribution
                search_depth_ = geo_(generator_);

            simulate_(particle, root_, model_, prior_, search_depth_, leaf_heuristic_);

            num_sims++;
            logd << "[POMCP::Search] " << num_sims << " simulations done" << endl;
            history_.Truncate(hist_size);

            if (num_sims >= num_simulates) {
                done = true;
                break;
            }
        }

        for (int i = 0; i < particles.size(); i++) {
            model_->Free(particles[i]);
        }

        if (done)
            break;
    }

    ValuedAction astar = OptimalAction(root_);

    logi << "[POMCP::Search] Search statistics" << endl
         << "OptimalAction = " << astar << endl
         << "# Simulations = " << root_->count() << endl
         << "Time: CPU / Real = " << ((clock() - start_cpu) / CLOCKS_PER_SEC) << " / " << (get_time_second() - start_real) << endl
         << "# active particles = " << model_->NumActiveParticles() << endl
         << "Tree size = " << root_->Size() << endl;

    if (astar.action == -1) {
        for (ACT_TYPE action = 0; action < model_->NumActions(); action++) {
            cout << "action " << action << ": " << root_->Child(action)->count()
                 << " " << root_->Child(action)->value() << endl;
        }
    }

    // delete root_;
    return astar;
}

ValuedAction POMCP::Search() {
    if (Globals::config.num_simulates != -1)
        return simSearch(Globals::config.num_simulates); //stop condition is amount of simulates
	return Search(Globals::config.time_per_move); //stop condition is time passed
}

void POMCP::belief(Belief* b) {
	belief_ = b;
	history_.Truncate(0);
  prior_->PopAll();
	delete root_;
	root_ = NULL;
}

void POMCP::BeliefUpdate(ACT_TYPE action, OBS_TYPE obs) {
	double start = get_time_second();

	if (reuse_) {
		VNode* node = root_->Child(action)->Child(obs);
		root_->Child(action)->children().erase(obs);
		delete root_;

		root_ = node;
		if (root_ != NULL) {
			root_->parent(NULL);
		}
	} else {
		delete root_;
		root_ = NULL;
	}

	prior_->Add(action, obs);
	history_.Add(action, obs);
	belief_->Update(action, obs);

	logi << "[POMCP::Update] Updated belief, history and root with action "
		<< action << ", observation " << obs
		<< " in " << (get_time_second() - start) << "s" << endl;
}

ACT_TYPE POMCP::UpperBoundAction(const VNode* vnode, double explore_constant) {
	const vector<QNode*>& qnodes = vnode->children();
	double best_ub = Globals::NEG_INFTY;
	ACT_TYPE best_action = -1;

	/*
	 int total = 0;
	 for (ACT_TYPE action = 0; action < qnodes.size(); action ++) {
	 total += qnodes[action]->count();
	 double ub = qnodes[action]->value() + explore_constant * sqrt(log(vnode->count() + 1) / qnodes[action]->count());
	 cout << action << " " << ub << " " << qnodes[action]->value() << " " << qnodes[action]->count() << " " << vnode->count() << endl;
	 }
	 */

	for (ACT_TYPE action = 0; action < qnodes.size(); action++) {
		if (qnodes[action]->count() == 0)
			return action;

		double ub = qnodes[action]->value()
			+ explore_constant
				* sqrt(log(vnode->count() + 1) / qnodes[action]->count());

		if (ub > best_ub) {
			best_ub = ub;
			best_action = action;
		}
	}
	assert(best_action != -1);
	return best_action;
}

ValuedAction POMCP::OptimalAction(const VNode* vnode) {
	const vector<QNode*>& qnodes = vnode->children();
	ValuedAction astar(-1, Globals::NEG_INFTY);
	for (ACT_TYPE action = 0; action < qnodes.size(); action++) {
		// cout << action << " " << qnodes[action]->value() << " " << qnodes[action]->count() << " " << vnode->count() << endl;
		if (qnodes[action]->value() > astar.value) {
			astar = ValuedAction(action, qnodes[action]->value());
		}
	}
	// assert(atar.action != -1);
	return astar;
}

int POMCP::Count(const VNode* vnode) {
	int count = 0;
	for (ACT_TYPE action = 0; action < vnode->children().size(); action++)
		count += vnode->Child(action)->count();
	return count;
}

VNode* POMCP::CreateVNode(int depth, const State* state, POMCPPrior* prior,
	const DSPOMDP* model) {
	VNode* vnode = new VNode(0, 0.0, depth);

	prior->ComputePreference(*state);

	const vector<int>& preferred_actions = prior->preferred_actions();
	const vector<int>& legal_actions = prior->legal_actions();

	int large_count = 1000000;
	double neg_infty = -1e10;

	if (legal_actions.size() == 0) { // no prior knowledge, all actions are equal
		for (ACT_TYPE action = 0; action < model->NumActions(); action++) {
			QNode* qnode = new QNode(vnode, action);
			qnode->count(0);
			qnode->value(0);

			vnode->children().push_back(qnode);
		}
	} else {
		for (ACT_TYPE action = 0; action < model->NumActions(); action++) {
			QNode* qnode = new QNode(vnode, action);
			qnode->count(large_count);
			qnode->value(neg_infty);

			vnode->children().push_back(qnode);
		}

		for (int a = 0; a < legal_actions.size(); a++) {
			QNode* qnode = vnode->Child(legal_actions[a]);
			qnode->count(0);
			qnode->value(0);
		}

//		for (int a = 0; a < preferred_actions.size(); a++) {
//			ACT_TYPE action = preferred_actions[a];
//			QNode* qnode = vnode->Child(action);
//			qnode->count(prior->SmartCount(action));
//			qnode->value(prior->SmartValue(action));
//		}
	}

	return vnode;
}


double POMCP::Simulate(State* particle, RandomStreams& streams, VNode* vnode,
	const DSPOMDP* model, POMCPPrior* prior, double (*leaf_heuristic)(State*, int, const DSPOMDP*, POMCPPrior*, int)) {
	if (streams.Exhausted())
		return 0;

	double explore_constant = prior->exploration_constant();

	ACT_TYPE action = POMCP::UpperBoundAction(vnode, explore_constant);
	logd << *particle << endl;
	logd << "depth = " << vnode->depth() << "; action = " << action << "; "
		<< particle->scenario_id << endl;

	double reward;
	OBS_TYPE obs;
	bool terminal = model->Step(*particle, streams.Entry(particle->scenario_id),
		action, reward, obs);

	QNode* qnode = vnode->Child(action);
	if (!terminal) {
		prior->Add(action, obs);
		streams.Advance();
		map<OBS_TYPE, VNode*>& vnodes = qnode->children();
		if (vnodes[obs] != NULL) {
			reward += Globals::Discount()
				* Simulate(particle, streams, vnodes[obs], model, prior, leaf_heuristic);
		} else {// Rollout upon encountering a node not in curren tree, then add the node
            vnodes[obs] = CreateVNode(vnode->depth() + 1, particle, prior,
                                      model);

            if (Globals::config.leaf_heuristic == "SARSOP"  or Globals::config.leaf_heuristic == "VI")
                reward += Globals::Discount()
                          * leaf_heuristic(particle, vnode->depth() + 1, model, prior, Globals::config.search_depth);
            else
                reward += Globals::Discount()
            * Rollout(particle, streams, vnode->depth() + 1, model, prior);


        }
		streams.Back();
		prior->PopLast();
	}

	qnode->Add(reward);
	vnode->Add(reward);

	return reward;
}

    double POMCP::Simulate(State* particle, RandomStreams& streams, VNode* vnode,
                           const DSPOMDP* model, POMCPPrior* prior) {
        if (streams.Exhausted())
            return 0;

        double explore_constant = prior->exploration_constant();

        ACT_TYPE action = POMCP::UpperBoundAction(vnode, explore_constant);
        logd << *particle << endl;
        logd << "depth = " << vnode->depth() << "; action = " << action << "; "
             << particle->scenario_id << endl;

        double reward;
        OBS_TYPE obs;
        bool terminal = model->Step(*particle, streams.Entry(particle->scenario_id),
                                    action, reward, obs);

        QNode* qnode = vnode->Child(action);
        if (!terminal) {
            prior->Add(action, obs);
            streams.Advance();
            map<OBS_TYPE, VNode*>& vnodes = qnode->children();
            if (vnodes[obs] != NULL) {
                reward += Globals::Discount()
                          * Simulate(particle, streams, vnodes[obs], model, prior);
            } else { // Rollout upon encountering a node not in curren tree, then add the node
                reward += Globals::Discount()
                          * Rollout(particle, streams, vnode->depth() + 1, model, prior);
                vnodes[obs] = CreateVNode(vnode->depth() + 1, particle, prior,
                                          model);
            }
            streams.Back();
            prior->PopLast();
        }

        qnode->Add(reward);
        vnode->Add(reward);

        return reward;
    }

// static
double POMCP::Simulate(State* particle, VNode* vnode, const DSPOMDP* model,
	POMCPPrior* prior, int search_depth, double (*leaf_heuristic)(State*, int, const DSPOMDP*, POMCPPrior*, int)) {
	assert(vnode != NULL);
	if (vnode->depth() >= search_depth)
		return 0;

	double explore_constant = prior->exploration_constant();

	ACT_TYPE action = UpperBoundAction(vnode, explore_constant);

	double reward;
	OBS_TYPE obs;
	bool terminal = model->Step(*particle, action, reward, obs);

	QNode* qnode = vnode->Child(action);
	if (!terminal) {
		prior->Add(action, obs);
		map<OBS_TYPE, VNode*>& vnodes = qnode->children();
		if (vnodes[obs] != NULL) {
			reward += Globals::Discount()
				* Simulate(particle, vnodes[obs], model, prior, search_depth, leaf_heuristic);
		} else { // Rollout upon encountering a node not in curren tree, then add the node
			vnodes[obs] = CreateVNode(vnode->depth() + 1, particle, prior,
				model);
			reward += Globals::Discount()
				* leaf_heuristic(particle, vnode->depth() + 1, model, prior, search_depth);
		}
		prior->PopLast();
	}

	qnode->Add(reward);
	vnode->Add(reward);

	return reward;
}


/* The action is detriment by the leader
 * the action is performed on the leader and each of the followers.
 * The followers are force to be on the same observation as the leader
 * The followers are weighted according to the observation probability in the follower and the leader*/
vector<double> POMCP::Simulate(vector<State *> &particles, VNode *vnode, const DSPOMDP *model,
                       POMCPPrior *prior, int search_depth,
                       vector<double> (*leaf_heuristic)(vector<State *> &, int, const DSPOMDP *, POMCPPrior *, int)) {

    assert(vnode != NULL);
    vector<double> Reward(particles.size(), 0.0);  //initiation, holds the immediate reward + simulated reward  of the leader and followers
    if (vnode->depth() >= search_depth) {
        return Reward;
    }

    double explore_constant = prior->exploration_constant();

    ACT_TYPE action = UpperBoundAction(vnode, explore_constant);


    OBS_TYPE leader_obs;
    double leaderObsWeight;
    double followerObsWeight;

    // Leader
    bool terminal = model->Step(*particles[0], action, Reward[0], leader_obs);
    leaderObsWeight = model->ObsProb(leader_obs, *particles[0], action);

    // Followers
    for (int i = 1; i < particles.size(); i++) {
        State* follower = particles[i];
        OBS_TYPE obs;
        model->Step(*follower, action, Reward[i], obs);
        followerObsWeight = model->ObsProb(leader_obs, *follower, action); // probability of getting the leader observation in the follower state
        follower->weight = follower->weight * (followerObsWeight / leaderObsWeight); //update the follower weight
    }

    QNode* qnode = vnode->Child(action);
    if (!terminal) {
        prior->Add(action, leader_obs);
        map<OBS_TYPE, VNode*>& vnodes = qnode->children();
        if (vnodes[leader_obs] != NULL) {
            vector <double> simReward = Simulate(particles, vnodes[leader_obs], model, prior, search_depth, leaf_heuristic);
            //for each immediate reward (leader and each follower) add the reward from the recursive simulate multiplied by the discount factor - R + gamma * Simulate(particle)
            transform(simReward.begin(), simReward.end(), Reward.begin(), Reward.begin(), [](double x, double y) { return y + (x * Globals::Discount()); });

        } else { // Rollout upon encountering a node not in current tree, then add the node
            vnodes[leader_obs] = CreateVNode(vnode->depth() + 1, particles[0], prior,
                                             model);
            vector <double> leafReward = leaf_heuristic(particles, vnode->depth() + 1, model, prior, search_depth);
            //for each immediate reward (leader and each follower) add the reward from the leaf_heuristic multiplied by the discount factor - R + gamma * leaf_heuristic(particle)
            transform(leafReward.begin(), leafReward.end(), Reward.begin(), Reward.begin(), [](double x, double y) { return y + (x * Globals::Discount()); });
        }
        prior->PopLast();
    }

    // update the qnode and vnode count and value with the leader and each follower weight and reward
    for(int i = 0; i < Reward.size(); i++) {
        qnode->Add(Reward[i], particles[i]->weight);
        vnode->Add(Reward[i], particles[i]->weight);
    }

//    qnode->Add(reward, weight);
//    vnode->Add(reward, weight);

    return Reward;
}

// Normalize the vector `v` to 1
vector<double> POMCP::normalize(vector<double> v){
    double divisor = accumulate(v.begin(), v.end(), 0.0);
    std::transform(v.begin(), v.end(), v.begin(),
                   [divisor](double element) { return element / divisor; });
    return v;
}

double POMCP::Check_default_policy_Simulate(State* particle, VNode* vnode, const DSPOMDP* model,
                       POMCPPrior* prior, int search_depth, double (*leaf_heuristic)(State*, int, const DSPOMDP*, POMCPPrior*, int)) {
    assert(vnode != NULL);
    if (vnode->depth() >= search_depth)
        return 0;

    double explore_constant = prior->exploration_constant();

    ACT_TYPE action = UpperBoundAction(vnode, explore_constant);

    double reward;
    OBS_TYPE obs;
    bool terminal = model->Step(*particle, action, reward, obs);

    QNode* qnode = vnode->Child(action);
    if (!terminal) {
        prior->Add(action, obs);
        reward += Globals::Discount()
                      * leaf_heuristic(particle, vnode->depth() + 1, model, prior, search_depth);

        prior->PopLast();
    }

    qnode->Add(reward);
    vnode->Add(reward);

    return reward;
}


double POMCP::Check_default_policy_Simulate(State* particle, RandomStreams& streams, VNode* vnode, const DSPOMDP* model,
                                            POMCPPrior* prior, double (*leaf_heuristic)(State*, int, const DSPOMDP*, POMCPPrior*, int)) {
    assert(vnode != NULL);
    if (streams.Exhausted())
        return 0;

    double explore_constant = prior->exploration_constant();

    ACT_TYPE action = UpperBoundAction(vnode, explore_constant);
    logd << *particle << endl;
    logd << "depth = " << vnode->depth() << "; action = " << action << "; "
         << particle->scenario_id << endl;

    double reward;
    OBS_TYPE obs;
    bool terminal = model->Step(*particle, streams.Entry(particle->scenario_id),
                                action, reward, obs);

    QNode* qnode = vnode->Child(action);
    if (!terminal) {
        prior->Add(action, obs);
        streams.Advance();
        if (Globals::config.leaf_heuristic == "SARSOP"  or Globals::config.leaf_heuristic == "VI")
            reward += Globals::Discount()
                      * leaf_heuristic(particle, vnode->depth() + 1, model, prior, Globals::config.search_depth);
        else
            reward += Globals::Discount()
                      * Rollout(particle, streams, vnode->depth() + 1, model, prior);

        streams.Back();
        prior->PopLast();
    }

    qnode->Add(reward);
    vnode->Add(reward);

    return reward;
}

vector<double> POMCP::Check_default_policy_Simulate(vector<State*>&particles, VNode* vnode, const DSPOMDP* model,
                                            POMCPPrior* prior, int search_depth, vector<double> (*leaf_heuristic)(vector<State*>&, int, const DSPOMDP*, POMCPPrior*, int)) {

    assert(vnode != NULL);
    vector<double> Reward(particles.size(), 0.0);

    if (vnode->depth() >= search_depth)
        return Reward;

    double explore_constant = prior->exploration_constant();

    ACT_TYPE action = UpperBoundAction(vnode, explore_constant);

    double reward;
    OBS_TYPE leader_obs;
    double leaderObsWeight;
    double followerObsWeight;

    bool terminal = model->Step(*particles[0], action, Reward[0], leader_obs);
    leaderObsWeight = model->ObsProb(leader_obs, *particles[0], action);

    for (int i = 1; i < particles.size(); i++) {
        State* follower = particles[i];
        OBS_TYPE obs;
        model->Step(*follower, action, Reward[i], obs);
        followerObsWeight = model->ObsProb(leader_obs, *follower, action);
        follower->weight = follower->weight * (followerObsWeight / leaderObsWeight);
    }

    QNode* qnode = vnode->Child(action);
    if (!terminal) {
        prior->Add(action, leader_obs);
        vector<double> leafReward = leaf_heuristic(particles, vnode->depth() + 1, model, prior, search_depth);
        transform(leafReward.begin(), leafReward.end(), Reward.begin(), Reward.begin(), [](double x, double y) { return y + (x * Globals::Discount()); });
        prior->PopLast();
    }

    for(int i = 0; i < particles.size(); i++) {
        qnode->Add(Reward[i], particles[i]->weight);
        vnode->Add(Reward[i], particles[i]->weight);
    }
//    qnode->Add(reward, weight);
//    vnode->Add(reward, weight);

    return Reward;
}

// static
double POMCP::Rollout(State* particle, RandomStreams& streams, int depth,
	const DSPOMDP* model, POMCPPrior* prior) {
	if (streams.Exhausted()) {
		return 0;
	}

	ACT_TYPE action = prior->GetAction(*particle);

	logd << *particle << endl;
	logd << "depth = " << depth << "; action = " << action << endl;

	double reward;
	OBS_TYPE obs;
	bool terminal = model->Step(*particle, streams.Entry(particle->scenario_id),
		action, reward, obs);
	if (!terminal) {
		prior->Add(action, obs);
		streams.Advance();
		reward += Globals::Discount()
			* Rollout(particle, streams, depth + 1, model, prior);
		streams.Back();
		prior->PopLast();
	}

	return reward;
}



// static
double POMCP::Rollout(State* particle, int depth, const DSPOMDP* model,
	POMCPPrior* prior, int search_depth) {
	if (depth >= search_depth) {
		return 0;
	}

	ACT_TYPE action = prior->GetAction(*particle);

	double reward;
	OBS_TYPE obs;
	bool terminal = model->Step(*particle, action, reward, obs);
	if (!terminal) {
		prior->Add(action, obs);
		reward += Globals::Discount() * Rollout(particle, depth + 1, model, prior, search_depth);
		prior->PopLast();
	}

	return reward;
}

// static
/* run a rollout for the leader and each follower */
vector<double> POMCP::Rollout(vector<State *> &particles, int depth, const DSPOMDP* model,
                      POMCPPrior* prior, int search_depth) {

//    vector<int> states(particles.size());
//    std::cout << "Number of unique elements is "
//              << std::set<double>( v.begin(), v.end() ).size()
//              << std::endl;

    vector<double> Reward(particles.size());

    for (int i = 0; i < particles.size(); i++) {  //perform a rollout starting from the leader and each follower
        Reward[i] = Rollout(particles[i], depth, model, prior, search_depth);
//        states[i] = particles[i]->state_id;
    }

//    int different_states = std::set<int>( states.begin(), states.end() ).size();

//    double sum = std::accumulate(Reward.begin(), Reward.end(), 0.0);
//    double mean = sum / Reward.size();
//
//    double sq_sum = std::inner_product(Reward.begin(), Reward.end(), Reward.begin(), 0.0);
//    double stdev = std::sqrt(sq_sum / Reward.size() - mean * mean);
//
//
//    default_out << "Num of different States = " << different_states << endl
//                << "Mean = " << mean << endl
//                << "STD = " << stdev << endl << endl;
    return Reward ;
}



/* run a rollout for the leader and each follower based on the leader actions */
vector<double> POMCP::Rollout_Leader(vector<State *> &particles, int depth, const DSPOMDP* model,
                                  POMCPPrior* prior, int search_depth) {

    vector<double> Reward(particles.size(), 0.0);
    if (depth >= search_depth) {
        return Reward;
    }

    ACT_TYPE action = prior->GetAction(*particles[0]);

    OBS_TYPE leaderObs;
    OBS_TYPE obs;
    bool terminal = model->Step(*particles[0], action, Reward[0], leaderObs);

    for (int i = 1; i < particles.size(); i++) {  //perform a rollout starting from the leader and each follower
        model->Step(*particles[i], action, Reward[i], obs);
    }

    if (!terminal) {
        prior->Add(action, leaderObs);
        vector<double> simReward = Rollout_Leader(particles, depth + 1, model, prior, search_depth);

        //for each immediate reward (leader and each follower) add the reward from the Rollout_Leader multiplied by the discount factor - R + gamma * Rollout_Leader(particle)
        transform(simReward.begin(), simReward.end(), Reward.begin(), Reward.begin(), [](double x, double y) { return y + (x * Globals::Discount()); });
        prior->PopLast();
    }

    return Reward;


}

double POMCP::Sarsop_heuristic(State* particle, int depth, const DSPOMDP* model,
                               POMCPPrior* prior, int search_depth){
    return model->Sarsop_state_value(particle);
}


/*get SARSOP value for the leader and each follower */
vector<double> POMCP::Sarsop_heuristic(vector<State *> &particles, int depth, const DSPOMDP* model,
                               POMCPPrior* prior, int search_depth){
    vector<double> Reward(particles.size());

    for(int i=0; i < particles.size(); i++)
        Reward[i] = model->Sarsop_state_value(particles[i]);

    return Reward;
}

double POMCP::Value_iteration_heuristic(State* particle, int depth, const DSPOMDP* model,
                                        POMCPPrior* prior, int search_depth){
    return model->VI_state_value(particle);
}


/* get VI value for the leader and each follower */
vector<double> POMCP::Value_iteration_heuristic(vector<State *> &particles, int depth, const DSPOMDP* model,
                                        POMCPPrior* prior, int search_depth){
    vector<double> Reward(particles.size());
//    vector<int> states(particles.size());


        for(int i=0; i < particles.size(); i++){
        Reward[i] = model->VI_state_value(particles[i]);
//        states[i] = particles[i]->state_id;
    }

//        int different_states = std::set<int>( states.begin(), states.end() ).size();
//
//        double sum = std::accumulate(Reward.begin(), Reward.end(), 0.0);
//        double mean = sum / Reward.size();
//
//        double sq_sum = std::inner_product(Reward.begin(), Reward.end(), Reward.begin(), 0.0);
//        double stdev = std::sqrt(sq_sum / Reward.size() - mean * mean);
//
//
//        default_out << "Num of different States = " << different_states << endl
//                    << "Mean = " << mean << endl
//                    << "STD = " << stdev << endl << endl;
    return Reward;
}

ValuedAction POMCP::Evaluate(VNode* root, vector<State*>& particles,
	RandomStreams& streams, const DSPOMDP* model, POMCPPrior* prior) {
	double value = 0;

	for (int i = 0; i < particles.size(); i++)
		particles[i]->scenario_id = i;

	for (int i = 0; i < particles.size(); i++) {
		State* particle = particles[i];
		VNode* cur = root;
		State* copy = model->Copy(particle);
		double discount = 1.0;
		double val = 0;
		int steps = 0;

		// Simulate until all random numbers are consumed
		while (!streams.Exhausted()) {
			ACT_TYPE action =
				(cur != NULL) ?
					UpperBoundAction(cur, 0) : prior->GetAction(*particle);

			double reward;
			OBS_TYPE obs;
			bool terminal = model->Step(*copy, streams.Entry(copy->scenario_id),
				action, reward, obs);

			val += discount * reward;
			discount *= Globals::Discount();

			if (!terminal) {
				prior->Add(action, obs);
				streams.Advance();
				steps++;

				if (cur != NULL) {
					QNode* qnode = cur->Child(action);
					map<OBS_TYPE, VNode*>& vnodes = qnode->children();
					cur = vnodes.find(obs) != vnodes.end() ? vnodes[obs] : NULL;
				}
			} else {
				break;
			}
		}

		// Reset random streams and prior
		for (int i = 0; i < steps; i++) {
			streams.Back();
			prior->PopLast();
		}

		model->Free(copy);

		value += val;
	}

	return ValuedAction(UpperBoundAction(root, 0), value / particles.size());
}




/* =============================================================================
 * DPOMCP class
 * =============================================================================*/

DPOMCP::DPOMCP(const DSPOMDP* model, POMCPPrior* prior, Belief* belief) :
	POMCP(model, prior, belief) {
	reuse_ = false;

    /*set simulate function, two options -
      1. default Simulate function
      2. Check_default_policy_Simulate - builds a tree with depth one.
    */
    double (*simulate_func)(State*, RandomStreams&, VNode*, const DSPOMDP*,
                            POMCPPrior*, double (*leaf_heuristic)(State*, int, const DSPOMDP*, POMCPPrior*, int)) = Simulate;
    double (*Check_default_policy_Simulate_func)(State*, RandomStreams& streams, VNode*, const DSPOMDP*,
                                                 POMCPPrior*, double (State*, int, const DSPOMDP*, POMCPPrior*, int)) = Check_default_policy_Simulate;
    simulate_ = Globals::config.check_default_policy ? Check_default_policy_Simulate_func : simulate_func;

    }

void DPOMCP::belief(Belief* b) {
	belief_ = b;
	history_.Truncate(0);
  prior_->PopAll();
}

ValuedAction DPOMCP::Search(double timeout) {
	double start_cpu = clock(), start_real = get_time_second();

	vector<State*> particles = belief_->Sample(Globals::config.num_scenarios);

	RandomStreams streams(Globals::config.num_scenarios,
		Globals::config.search_depth);

	root_ = ConstructTree(particles, streams, model_, prior_, history_, leaf_heuristic_,
		timeout);

	for (int i = 0; i < particles.size(); i++)
		model_->Free(particles[i]);

	logi << "[DPOMCP::Search] Time: CPU / Real = "
		<< ((clock() - start_cpu) / CLOCKS_PER_SEC) << " / "
		<< (get_time_second() - start_real) << endl << "Tree size = "
		<< root_->Size() << endl;

	ValuedAction astar = OptimalAction(root_);
	if (astar.action == -1) {
		for (ACT_TYPE action = 0; action < model_->NumActions(); action++) {
			cout << "action " << action << ": " << root_->Child(action)->count()
				<< " " << root_->Child(action)->value() << endl;
		}
	}

	delete root_;
	return astar;
}

// static
VNode* DPOMCP::ConstructTree(vector<State*>& particles, RandomStreams& streams,
	const DSPOMDP* model, POMCPPrior* prior, History& history, double (*leaf_heuristic)(State*, int, const DSPOMDP* leaf_heuristic, POMCPPrior*, int), double timeout) {
	prior->history(history);
	VNode* root = CreateVNode(0, particles[0], prior, model);

	for (int i = 0; i < particles.size(); i++)
		particles[i]->scenario_id = i;

	logi << "[DPOMCP::ConstructTree] # active particles before search = "
		<< model->NumActiveParticles() << endl;
	double start = clock();
	int num_sims = 0;
	while (true) {
		logd << "Simulation " << num_sims << endl;

		int index = Random::RANDOM.NextInt(particles.size());
		State* particle = model->Copy(particles[index]);


		Simulate(particle, streams, root, model, prior, leaf_heuristic);
		num_sims++;
		model->Free(particle);

		if ((clock() - start) / CLOCKS_PER_SEC >= timeout) {
			break;
		}
	}

	logi << "[DPOMCP::ConstructTree] OptimalAction = " << OptimalAction(root)
		<< endl << "# Simulations = " << root->count() << endl
		<< "# active particles after search = " << model->NumActiveParticles()
		<< endl;

	return root;
}

void DPOMCP::BeliefUpdate(ACT_TYPE action, OBS_TYPE obs) {
	double start = get_time_second();

	history_.Add(action, obs);
	belief_->Update(action, obs);

	logi << "[DPOMCP::Update] Updated belief, history and root with action "
		<< action << ", observation " << obs
		<< " in " << (get_time_second() - start) << "s" << endl;
}

/* =============================================================================
 * LEAFOMCP class
 * =============================================================================*/


LEAFOMCP::LEAFOMCP(const DSPOMDP *model, POMCPPrior *prior, Belief *belief) :
    POMCP(model, prior, belief) {
    reuse_ = false;

    /*set simulate function, two options -
      1. default Simulate function
      2. Check_default_policy_Simulate - builds a tree with depth one.
    */
    vector<double> (*simulate_func)(vector<State*>&, VNode*, const DSPOMDP*,
                            POMCPPrior*, int, vector<double> (*leaf_heuristic)(vector<State*>&, int, const DSPOMDP*, POMCPPrior*, int)) = Simulate;
    vector<double> (*Check_default_policy_Simulate_func)(vector<State*>&, VNode*, const DSPOMDP*,
                                                 POMCPPrior*, int, vector<double> (*leaf_heuristic)(vector<State*>&, int, const DSPOMDP*, POMCPPrior*, int)) = Check_default_policy_Simulate;
    simulate_ = Globals::config.check_default_policy ? Check_default_policy_Simulate_func : simulate_func;

    /*set leaf_heuristic function, three options -
      1. Rollout - follows the default policy till sim_depth or reaching terminal state
      2. VI - value iteration value of the leaf state
      3. SARSOP - sarsop value of the leaf state
    */
    vector<double> (*rolloutFunc)(vector<State*>&, int, const DSPOMDP*, POMCPPrior*, int) = Rollout_Leader;
    vector<double>  (*sarsopFunc)(vector<State*>&, int, const DSPOMDP*, POMCPPrior*, int) = Sarsop_heuristic;
    vector<double>  (*viFunc)(vector<State*>&, int, const DSPOMDP*, POMCPPrior*, int) = Value_iteration_heuristic;
    leaf_heuristic_ = Globals::config.leaf_heuristic == "SARSOP" ? sarsopFunc : Globals::config.leaf_heuristic == "VI" ? viFunc : rolloutFunc;

    }

void LEAFOMCP::belief(Belief* b) {
    belief_ = b;
    history_.Truncate(0);
    prior_->PopAll();
}


/* Search for the best action.
  runs simulates until the time for the search phase is over
  each simulate K particles are sampled from the belief,
  one of the K particles is determined to be the leader and the rest are the followers*/
ValuedAction LEAFOMCP::Search(double timeout) {
    double start_cpu = clock(), start_real = get_time_second();

    if (root_ == NULL) {
        State* state = belief_->Sample(1)[0];
        root_ = CreateVNode(0, state, prior_, model_);
        model_->Free(state);
    }

    int hist_size = history_.Size();
    int num_sims = 0;
    while (true) {
        vector<State*> particles = belief_->Sample(Globals::config.num_scenarios, 1.0); // Sample K particles

            logd << "[POMCP::Search] Starting simulation " << num_sims << endl;

            if(Globals::config.geometric_search_depth) //sets the search depth according to a sample from geometric distribution
                search_depth_ = geo_(generator_);

            simulate_(particles, root_, model_, prior_, search_depth_, leaf_heuristic_);

            num_sims++;
            logd << "[POMCP::Search] " << num_sims << " simulations done" << endl;
            history_.Truncate(hist_size);

        // free all the particles used in this simulation
        for (int i = 0; i < particles.size(); i++)
            model_->Free(particles[i]);


        if ((clock() - start_cpu) / CLOCKS_PER_SEC >= timeout)
            break;
    }

    logi << "[LEAFOMCP::Search] Time: CPU / Real = "
         << ((clock() - start_cpu) / CLOCKS_PER_SEC) << " / "
         << (get_time_second() - start_real) << endl << "Tree size = "
         << root_->Size() << endl;

    ValuedAction astar = OptimalAction(root_);
    if (astar.action == -1) {
        for (ACT_TYPE action = 0; action < model_->NumActions(); action++) {
            cout << "action " << action << ": " << root_->Child(action)->count()
                 << " " << root_->Child(action)->value() << endl;
        }
    }

    delete root_;
    root_ = NULL;
    return astar;
}


/* Search for the best action.
  runs num_simulates simulates
  each simulate K particles are sampled from the belief,
  one of the K particles is determined to be the leader and the rest are the followers*/
ValuedAction LEAFOMCP::simSearch(int num_simulates) {
    double start_cpu = clock(), start_real = get_time_second();

    if (root_ == NULL) {
        State* state = belief_->Sample(1)[0];
        root_ = CreateVNode(0, state, prior_, model_);
        model_->Free(state);
    }

    int hist_size = history_.Size();
    for (int num_sims=0; num_sims< num_simulates; num_sims++) {
        vector<State*> particles = belief_->Sample(Globals::config.num_scenarios, 1.0); // Sample K particles

//        vector<int> states(particles.size());
//
//
//        for(int i=0; i < particles.size(); i++)
//            states[i] = particles[i]->state_id;


//        int different_states = std::set<int>( states.begin(), states.end() ).size();

//        cout << different_states << endl;

        logd << "[POMCP::Search] Starting simulation " << num_sims << endl;

        if(Globals::config.geometric_search_depth) //sets the search depth according to a sample from geometric distribution
            search_depth_ = geo_(generator_);

        simulate_(particles, root_, model_, prior_, search_depth_, leaf_heuristic_);

        logd << "[POMCP::Search] " << num_sims << " simulations done" << endl;
        history_.Truncate(hist_size);

        // free all the particles used in this simulation
        for (int i = 0; i < particles.size(); i++)
            model_->Free(particles[i]);
    }

    logi << "[LEAFOMCP::Search] Time: CPU / Real = "
         << ((clock() - start_cpu) / CLOCKS_PER_SEC) << " / "
         << (get_time_second() - start_real) << endl << "Tree size = "
         << root_->Size() << endl;

    ValuedAction astar = OptimalAction(root_);
    if (astar.action == -1) {
        for (ACT_TYPE action = 0; action < model_->NumActions(); action++) {
            cout << "action " << action << ": " << root_->Child(action)->count()
                 << " " << root_->Child(action)->value() << endl;
        }
    }

    delete root_;
    root_ = NULL;
    return astar;
}


void LEAFOMCP::BeliefUpdate(ACT_TYPE action, OBS_TYPE obs) {
    double start = get_time_second();

    prior_->Add(action, obs);  // TODO: maybe unnecessary
    history_.Add(action, obs);
    belief_->Update(action, obs);

    logi << "[LEAFOMCP::Update] Updated belief, history and root with action "
         << action << ", observation " << obs
         << " in " << (get_time_second() - start) << "s" << endl;
}

} // namespace despot
