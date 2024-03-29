#ifndef POMCP_H
#define POMCP_H

#include <despot/interface/pomdp.h>
#include <despot/core/node.h>
#include <despot/core/globals.h>
#include <random>


namespace despot {

/* =============================================================================
 * POMCPPrior class
 * =============================================================================*/

class POMCPPrior {
protected:
	const DSPOMDP* model_;
	History history_;
	double exploration_constant_;
	std::vector<int> preferred_actions_;
	std::vector<int> legal_actions_;

public:
	POMCPPrior(const DSPOMDP* model);
	virtual ~POMCPPrior();

	inline void exploration_constant(double constant) {
		exploration_constant_ = constant;
	}

	inline double exploration_constant() const {
		return exploration_constant_;
	}

	inline virtual int SmartCount(ACT_TYPE action) const {
		return 10;
	}

	inline virtual double SmartValue(ACT_TYPE action) const {
		return 1;
	}

	inline virtual const History& history() const {
		return history_;
	}

	inline virtual void history(History h) {
		history_ = h;
	}

	inline virtual void Add(ACT_TYPE action, OBS_TYPE obs) {
		history_.Add(action, obs);
	}

	inline virtual void PopLast() {
		history_.RemoveLast();
	}

  inline virtual void PopAll() {
		history_.Truncate(0);
	}

	virtual void ComputePreference(const State& state) = 0;

	const std::vector<int>& preferred_actions() const;
	const std::vector<int>& legal_actions() const;

	ACT_TYPE GetAction(const State& state);
};

/* =============================================================================
 * UniformPOMCPPrior class
 * =============================================================================*/

class UniformPOMCPPrior: public POMCPPrior {
public:
	UniformPOMCPPrior(const DSPOMDP* model);
	virtual ~UniformPOMCPPrior();

	void ComputePreference(const State& state);
};

/* =============================================================================
 * POMCP class
 * =============================================================================*/

class POMCP: public Solver {
protected:
	VNode* root_;
	POMCPPrior* prior_;
	bool reuse_;
    std::default_random_engine generator_;
    std::geometric_distribution<int> geo_;
    int search_depth_;
    double (*leaf_heuristic_)(State*, int, const DSPOMDP*,
                           POMCPPrior*, int);
    double (*simulate_)(State*, VNode*, const DSPOMDP*,
            POMCPPrior*, int, double (State*, int, const DSPOMDP*, POMCPPrior*, int));

public:
	POMCP(const DSPOMDP* model, POMCPPrior* prior, Belief* belief = NULL);
	virtual ValuedAction Search();
	virtual ValuedAction Search(double timeout);
    virtual ValuedAction simSearch(int num_simulates);

	void reuse(bool r);
	virtual void belief(Belief* b);
	virtual void BeliefUpdate(ACT_TYPE action, OBS_TYPE obs);

	static VNode* CreateVNode(int depth, const State*, POMCPPrior* prior,
		const DSPOMDP* model);
    static double Simulate(State* particle, VNode* root, const DSPOMDP* model,
                           POMCPPrior* prior, int search_depth, double (*leaf_heuristic)(State*, int, const DSPOMDP*, POMCPPrior*, int));
	static double Simulate(State* particle, RandomStreams& streams,
		VNode* vnode, const DSPOMDP* model, POMCPPrior* prior,  double (*leaf_heuristic)(State*, int, const DSPOMDP*, POMCPPrior*, int));
    static std::vector<double> Simulate(std::vector<State*>& particles,
                           VNode* vnode, const DSPOMDP* model, POMCPPrior* prior, int search_depth, std::vector<double> (*leaf_heuristic)(std::vector<State *> &,  int, const DSPOMDP*, POMCPPrior*, int));
    static double Simulate(State* particle, RandomStreams& streams,
                           VNode* vnode, const DSPOMDP* model, POMCPPrior* prior);
    static double Rollout(State* particle, int depth, const DSPOMDP* model,
		POMCPPrior* prior, int search_depth);
	static double Rollout(State* particle, RandomStreams& streams, int depth,
		const DSPOMDP* model, POMCPPrior* prior);
    static std::vector<double> Rollout(std::vector<State *> &particles, int depth, const DSPOMDP *model,
            POMCPPrior *prior, int search_depth);
    static std::vector<double> Rollout_Leader(std::vector<State *> &particles, int depth, const DSPOMDP *model,
                                       POMCPPrior *prior, int search_depth);
    static double Sarsop_heuristic(State* particle, int depth, const DSPOMDP* model,
                          POMCPPrior* prior, int search_depth);
    static std::vector<double> Sarsop_heuristic(std::vector<State *> &, int depth, const DSPOMDP* model,
                                   POMCPPrior* prior, int search_depth);
    static double Value_iteration_heuristic(State* particle, int depth, const DSPOMDP* model,
                          POMCPPrior* prior, int search_depth);
    static std::vector<double> Value_iteration_heuristic(std::vector<State *> &, int depth, const DSPOMDP* model,
                                            POMCPPrior* prior, int search_depth);
	static ValuedAction Evaluate(VNode* root, std::vector<State*>& particles,
		RandomStreams& streams, const DSPOMDP* model, POMCPPrior* prior);
	static ACT_TYPE UpperBoundAction(const VNode* vnode, double explore_constant);
	static ValuedAction OptimalAction(const VNode* vnode);
	static int Count(const VNode* vnode);

    static double
    Check_default_policy_Simulate(State *particle, VNode *vnode, const DSPOMDP *model, POMCPPrior *prior,
                                  int search_depth,
                                  double (*leaf_heuristic)(State *, int, const DSPOMDP *, POMCPPrior *, int));
    static double
    Check_default_policy_Simulate(State *particle, RandomStreams& streams, VNode *vnode, const DSPOMDP *model, POMCPPrior *prior,
                                  double (*leaf_heuristic)(State *, int, const DSPOMDP *, POMCPPrior *, int));


    static std::vector<double>
    Check_default_policy_Simulate(std::vector<State *> &particles, VNode *vnode,
                                         const DSPOMDP *model, POMCPPrior *prior, int search_depth,
                                        std::vector<double>  (*leaf_heuristic)(std::vector<State*>&, int, const DSPOMDP *, POMCPPrior *, int));
    static std::vector<double> normalize(std::vector<double> v);
};

/* =============================================================================
 * DPOMCP class
 * =============================================================================*/

class DPOMCP: public POMCP {
protected:
    double (*simulate_)(State*,  RandomStreams& streams, VNode*, const DSPOMDP*,
                        POMCPPrior*, double (State*, int, const DSPOMDP*, POMCPPrior*, int));

public:
    DPOMCP(const DSPOMDP* model, POMCPPrior* prior, Belief* belief = NULL);

	virtual ValuedAction Search(double timeout);
	static VNode* ConstructTree(std::vector<State*>& particles,
		RandomStreams& streams, const DSPOMDP* model, POMCPPrior* prior,
		History& history,  double (*leaf_heuristic)(State*, int, const DSPOMDP*, POMCPPrior*, int), double timeout);

	virtual void belief(Belief* b);
	virtual void BeliefUpdate(ACT_TYPE action, OBS_TYPE obs);
};



/* =============================================================================
 * LEAFOMCP class
 * =============================================================================*/

class LEAFOMCP: public POMCP {
protected:
    std::vector<double> (*simulate_)(std::vector<State*>&, VNode*, const DSPOMDP*,
                        POMCPPrior*, int, std::vector<double> (std::vector<State*>&, int, const DSPOMDP*, POMCPPrior*, int));
    std::vector<double> (*leaf_heuristic_)(std::vector<State*>&, int, const DSPOMDP*,
                              POMCPPrior*, int);
public:
    LEAFOMCP(const DSPOMDP* model, POMCPPrior* prior, Belief* belief = NULL);

    virtual ValuedAction Search(double timeout);
    virtual ValuedAction simSearch(int num_simulates);
    virtual void belief(Belief* b);
    virtual void BeliefUpdate(ACT_TYPE action, OBS_TYPE obs);
};



} // namespace despot

#endif
