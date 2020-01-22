
#pragma once

#include <stdio.h>
#include <unordered_set>



#include <fs/core/problem.hxx>
#include <fs/core/problem_info.hxx>
#include <fs/core/search/drivers/sbfws/base.hxx>
#include <search/algorithms/lookahead/iw_stats.hxx>
#include <fs/core/utils/printers/vector.hxx>
#include <fs/core/utils/printers/actions.hxx>
#include <lapkt/search/components/open_lists.hxx>
#include <fs/core/utils/config.hxx>
#include <lapkt/novelty/tuples.hxx>
#include <lapkt/novelty/features.hxx>
#include <lapkt/tools/resources_control.hxx>
#include <lapkt/tools/logging.hxx>
#include <fs/core/heuristics/novelty/features.hxx>
#include <fs/core/heuristics/reward.hxx>

// For logging search trees
#include <search/algorithms/lookahead/treelog.hxx>

namespace fs0 { namespace lookahead {

using FSFeatureValueT = lapkt::novelty::FeatureValueT;
typedef lapkt::novelty::Width1Tuple<FSFeatureValueT> Width1Tuple;
typedef lapkt::novelty::Width1TupleHasher<FSFeatureValueT> Width1TupleHasher;
typedef lapkt::novelty::Width2Tuple<FSFeatureValueT> Width2Tuple;
typedef lapkt::novelty::Width2TupleHasher<FSFeatureValueT> Width2TupleHasher;

template <typename StateT, typename ActionType>
class IWNode {
public:
	using ActionT = ActionType;
	using PT = std::shared_ptr<IWNode<StateT, ActionT>>;

	//! The state in this node
	StateT state;

	//! The action that led to this node
	typename ActionT::IdType action;

	//! The parent node
	PT parent;

	//! Generation
	//! GJF: This was marked as accumulated cost when it appears not to be.
	unsigned g;

	//! The novelty  of the state
	unsigned char _w;

    //! Reward
    float R;

	//! The generation order, uniquely identifies the node
	//! NOTE We're assuming we won't generate more than 2^32 ~ 4.2 billion nodes.
	uint32_t _gen_order;

	//! GJF: Used in calculating children's terminal cost change (instantaneous)
	float terminal_cost;

	//! GJF: Used in calculating children's stage cost change (instantaneous)
	float stage_cost;


	IWNode() = default;
	~IWNode() = default;
	IWNode(const IWNode&) = default;
	IWNode(IWNode&&) = delete;
	IWNode& operator=(const IWNode&) = delete;
	IWNode& operator=(IWNode&&) = delete;

	//! Constructor with full copying of the state (expensive)
	IWNode(const StateT& s, unsigned long gen_order) : IWNode(StateT(s), ActionT::invalid_action_id, nullptr, gen_order) {}

	//! Constructor with move of the state (cheaper)
	IWNode(const StateT& s, typename ActionT::IdType _action, PT _parent, uint32_t gen_order) :
		state(s),
		action(_action),
		parent(_parent),
		g(parent ? parent->g+1 : 0),
		_w(std::numeric_limits<unsigned char>::max()),
        R(0.0f),
		_gen_order(gen_order),
		terminal_cost(0.0f),
		stage_cost(0.0f)
	{
		assert(_gen_order < std::numeric_limits<uint32_t>::max()); // Very silly way to detect overflow, in case we ever generate > 4 billion nodes :-)
	}
	//! Constructor with move of the state (cheaper)
	IWNode(StateT&& _state, typename ActionT::IdType _action, PT _parent, uint32_t gen_order) :
		state(std::move(_state)),
		action(_action),
		parent(_parent),
		g(parent ? parent->g+1 : 0),
		_w(std::numeric_limits<unsigned char>::max()),
        R(0.0f),
		_gen_order(gen_order),
		terminal_cost(0.0f),
		stage_cost(0.0f)
	{
		assert(_gen_order < std::numeric_limits<uint32_t>::max()); // Very silly way to detect overflow, in case we ever generate > 4 billion nodes :-)
	}


	bool has_parent() const { return parent != nullptr; }

	//! Print the node into the given stream
	friend std::ostream& operator<<(std::ostream &os, const IWNode<StateT, ActionT>& object) { return object.print(os); }
	std::ostream& print(std::ostream& os) const {
		os << "{@ = " << this;
		os << ", #=" << _gen_order ;
		os << ", s = " << state ;
		os << ", g=" << g ;
        os << ", R=" << R ;
		os << ", w=" << (_w == std::numeric_limits<unsigned char>::max() ? "INF" : std::to_string(_w));

		os << ", act=" << action ;
		os << ", parent = " << (parent ? "#" + std::to_string(parent->_gen_order) : "None");
		return os;
	}

	bool operator==( const IWNode<StateT, ActionT>& o ) const { return state == o.state; }

	std::size_t hash() const { return state.hash(); }
};


template <typename NodeT, typename FeatureSetT, typename NoveltyEvaluatorT>
class LazyEvaluator {
protected:
	//! The set of features used to compute the novelty
	const FeatureSetT& _features;

	//! A single novelty evaluator will be in charge of evaluating all nodes
	std::unique_ptr<NoveltyEvaluatorT> _evaluator;

public:

    typedef typename NoveltyEvaluatorT::ValuationT ValuationT;

	LazyEvaluator(const FeatureSetT& features, NoveltyEvaluatorT* evaluator) :
		_features(features),
		_evaluator(evaluator)
	{}

	~LazyEvaluator() = default;

	//! Returns false iff we want to prune this node during the search
	unsigned evaluate(NodeT& node) {
		if (node.parent) {
			// Important: the novel-based computation works only when the parent has the same novelty type and thus goes against the same novelty tables!!!
			node._w = _evaluator->evaluate(_features.evaluate(node.state), _features.evaluate(node.parent->state));
		} else {
			node._w = _evaluator->evaluate(_features.evaluate(node.state));
		}

		return node._w;
	}

	std::vector<Width1Tuple> reached_tuples() const {
		std::vector<Width1Tuple> tuples;
		_evaluator->mark_tuples_in_novelty1_table(tuples);
		return tuples;
	}

	void reset() {
		_evaluator->reset();
	}

    const FeatureSetT& feature_set() const { return _features; }

};


//! A single IW run (with parametrized max. width) that runs until (independent)
//! satisfaction of each of the provided goal atoms, and computes the set
//! of atoms R that is relevant for the achievement of at least one atom.
//! R is computed treating the actions as a black-box. For this, an atom is considered
//! relevant for a certain goal atom if that atom is true in at least one of the states
//! that lies on the path between the seed node and the first node where that goal atom is
//! satisfied.
template <typename NodeT,
          typename StateModel,
          typename NoveltyEvaluatorT,
		  typename FeatureSetT
>
class IW
{
public:
	using ActionT = typename StateModel::ActionType;
	using StateT = typename StateModel::StateT;

	using ActionIdT = typename StateModel::ActionType::IdType;
    using PlanT =  std::vector<ActionIdT>;
	using NodePT = std::shared_ptr<NodeT>;

	using SimEvaluatorT = LazyEvaluator<NodeT, FeatureSetT, NoveltyEvaluatorT>;

	using FeatureValueT = typename NoveltyEvaluatorT::FeatureValueT;

	using OpenListT = lapkt::SimpleQueue<NodeT>;

	using RewardPT = std::shared_ptr<Reward>;


	// MRJ: IW(1) debugging
	std::vector<NodePT>	_visited;

	struct Config {
		//! Whether to perform a complete run or a partial one, i.e. up until (independent) satisfaction of all goal atoms.
		bool _complete;

		//! The maximum levels of novelty to be considered
		unsigned _max_width;

		//!
		unsigned _max_depth;

		//!
		const fs0::Config& _global_config;

		//! Whether to extract goal-informed relevant sets R
		bool _goal_directed;

		//! Enforce state constraints
		bool _enforce_state_constraints;

		//! Load R set from file
		std::string _R_file;

		//! Log search
		bool _log_search;

		//! BrFS layers
		unsigned _num_brfs_layers;

		//! Pivot on rewards
		bool	 _pivot_on_rewards;

		//! discount factor
		float 	_discount_factor;

		Config(bool complete, unsigned max_width, const fs0::Config& global_config) :
			_complete(complete),
			_max_width(max_width),
			_max_depth(global_config.getOption<int>("lookahead.iw.max_depth",std::numeric_limits<unsigned>::max())),
			_global_config(global_config),
			_goal_directed(global_config.getOption<bool>("lookahead.iw.goal_directed", false)),
			_enforce_state_constraints(global_config.getOption<bool>("lookahead.iw.enforce_state_constraints", true)),
			_R_file(global_config.getOption<std::string>("lookahead.iw.from_file", "")),
			_log_search(global_config.getOption<bool>("lookahead.iw.log", false)),
			_num_brfs_layers(global_config.getOption<int>("lookahead.iw.layers", 0)),
			_pivot_on_rewards(global_config.getOption<bool>("lookahead.iw.pivot_on_rewards", false)),
			_discount_factor(global_config.getOption<float>("lookahead.iw.discount_factor", 1.0))
		{
		}
	};
	//! The search model
	const StateModel& _model;
protected:


	//! The simulation configuration
	Config _config;

    //! Best node found
	NodePT _best_node;

	//!
	std::vector<NodePT> _optimal_paths;

	//! '_unreached' contains the indexes of all those goal atoms that have yet not been reached.
	std::unordered_set<unsigned> _unreached;

	//! Contains the indexes of all those goal atoms that were already reached in the seed state
	std::vector<bool> _in_seed;

	//! A single novelty evaluator will be in charge of evaluating all nodes
	SimEvaluatorT _evaluator;

	//! The general statistics of the search
	IteratedWidthStats& _stats;

	//! Whether to print some useful extra information or not
	bool _verbose;


	// MRJ: Reward Function
	RewardPT	_reward_function;

public:

	//! Constructor
	IW(const StateModel& model, const FeatureSetT& featureset, NoveltyEvaluatorT* evaluator, const IW::Config& config, IteratedWidthStats& stats, bool verbose) :
		_model(model),
		_config(config),
        _best_node(nullptr),
		_optimal_paths(model.num_subgoals()),
		_unreached(),
		_in_seed(),
		_evaluator(featureset, evaluator),
		_stats(stats),
		_verbose(verbose),
		_reward_function(nullptr)
	{
	}

	NodePT get_best_node() const {
		return _best_node;
	}

	void reset() {
		std::vector<NodePT> _(_optimal_paths.size(), nullptr);
		_optimal_paths.swap(_);
        _best_node = nullptr;
		_evaluator.reset();
		_stats.reset();
	}

	~IW() = default;

	// Disallow copy, but allow move
	IW() = delete;
	IW(IW&) = delete;
	IW(const IW&) = delete;
	IW(IW&&) = delete;
	IW& operator=(const IW&) = delete;
	IW& operator=(IW&&) = delete;

	std::vector<NodePT> extract_seed_nodes() {
		std::vector<NodePT> seed_nodes;
		for (unsigned subgoal_idx = 0; subgoal_idx < _optimal_paths.size(); ++subgoal_idx) {
			if (!_in_seed[subgoal_idx] && _optimal_paths[subgoal_idx] != nullptr) {
				seed_nodes.push_back(_optimal_paths[subgoal_idx]);
			}
		}
		return seed_nodes;
	}

	class DeactivateZCC {
		bool _current_setting;
	public:
		DeactivateZCC() {
			_current_setting = fs0::Config::instance().getZeroCrossingControl();
			fs0::Config::instance().setZeroCrossingControl(false);
		}

		~DeactivateZCC() {
			fs0::Config::instance().setZeroCrossingControl(_current_setting);
		}
	};

	//!
	void set_reward_function( RewardPT f ) {
		_reward_function = f;
	}

	RewardPT
	get_reward_function() const {
		return _reward_function;
	}

	bool different_time_step(const State& s1, const State& s2) const {
		unsigned ct_var = ProblemInfo::getInstance().getVariableId("clock_time()");
		float s1_time = fs0::value<float>(s1.getValue(ct_var));
		float s2_time = fs0::value<float>(s2.getValue(ct_var));
		return s1_time != s2_time;
	}

	bool state_after(const State& later, const State& earlier) const {
		unsigned ct_var = ProblemInfo::getInstance().getVariableId("clock_time()");
		float later_time = fs0::value<float>(later.getValue(ct_var));
		float earlier_time = fs0::value<float>(earlier.getValue(ct_var));
		return later_time > earlier_time;
	}

	//! Evaluate reward
	void evaluate_reward( NodePT n ) const {
		if ( _reward_function == nullptr ) {
			n->R = 0.0f;
			return;
		}
		// GJF: Reward function gives absolute value of state, here we use the
		// difference between the new state reward and its successor state's reward.
		if ( n->parent != nullptr ) {
			//LPT_INFO("discount", "Parent reward: " << n->parent->R);
			//LPT_INFO("discount", "Parent state reward: " << _reward_function->evaluate(n->parent->state));
			//float discount_factor = std::pow(_config._discount_factor, n->g);
			float terminal_cost = _reward_function->terminal(n->state);
			float parent_terminal_cost = n->parent->terminal_cost;
			n->terminal_cost = terminal_cost;
			float stage_cost = _reward_function->evaluate(n->state);
			float parent_stage_cost = n->parent->stage_cost;
			n->stage_cost = stage_cost;
			// Update terminal cost difference while maintaining stage cost accumulated.
			n->R += parent_terminal_cost - terminal_cost;
			// Add stage cost.
			if (different_time_step(n->state, n->parent->state)){
				// Since we've changed time steps, the previous timestep's stage cost
				// has stabilised, so we add a new stage cost fresh.
				n->R += parent_stage_cost;
			}
			n->R += stage_cost - parent_stage_cost;
			//LPT_INFO("discount", "Child state reward: " << _reward_function->evaluate(n->state));
			//LPT_INFO("discount", "Child reward increment: " << n->R);
		} else {
			float terminal_cost = _reward_function->terminal(n->state);
			float stage_cost = _reward_function->evaluate(n->state);
			n->stage_cost = stage_cost;
			n->terminal_cost = terminal_cost;
			n->R = terminal_cost + stage_cost;
		}
		if ( n->parent != nullptr )
			n->R += n->parent->R; // accumulate
		return;
	}

    //! Convenience method
	bool solve_model(PlanT& solution) { return search(_model.init(), solution); }

	bool search(const StateT& s, PlanT& plan) {
        _best_node = nullptr; // Make sure we start assuming no solution found
		NodePT top_level = std::make_shared<NodeT>(s, _stats.generated());

		if ( _config._pivot_on_rewards ) {
			LPT_INFO("search", "Pivoting on rewards...");
			NodePT current_best = _best_node;
			if ( _config._num_brfs_layers > 0 ) {
				LPT_INFO("search", "Using the lookahead...");
				unsigned num_app_root = 0;
				for (const auto& a : _model.applicable_actions(s, _config._enforce_state_constraints)) {
					StateT s_a = _model.next( s, a );
					_stats.generation();

		        	run(s_a, _config._max_width, top_level, a);
					LPT_INFO("search", "Finished run " << ++num_app_root << ": max R(s)=" << _best_node->R << " visited: " << _visited.size() );
					std::vector<NodePT> _(_optimal_paths.size(), nullptr);
					_optimal_paths.swap(_);
					_evaluator.reset();
					LPT_INFO("search", "Run finished for action: #" << num_app_root);
				}
				LPT_INFO("search", "Number of applicable actions: " << num_app_root);
			}
			else {
				run(s, _config._max_width, nullptr, (ActionIdT)0);
				LPT_INFO("search", "Finished first run: max R(s)=" << _best_node->R << " visited: " << _visited.size() );
			}
			std::vector<NodePT> _(_optimal_paths.size(), nullptr);
			_optimal_paths.swap(_);
			_evaluator.reset();
			while ( _best_node != current_best ){
				current_best = _best_node;
				for (const auto& a : _model.applicable_actions(current_best->state, _config._enforce_state_constraints)) {
					StateT s_a = _model.next( current_best->state, a );
					_stats.generation();

					run(s_a, _config._max_width, current_best, a);
					LPT_INFO("search", "Finished run: max R(s)=" << _best_node->R << " visited: " << _visited.size() );
					std::vector<NodePT> _(_optimal_paths.size(), nullptr);
					_optimal_paths.swap(_);
					_evaluator.reset();
				}
			}
			return extract_plan( _best_node, plan );
		}

		if ( _config._num_brfs_layers > 0 ) {
			LPT_INFO("search", "Using the lookahead...");
			for (const auto& a : _model.applicable_actions(s, _config._enforce_state_constraints)) {
				StateT s_a = _model.next( s, a );
				_stats.generation();

	        	run(s_a, _config._max_width, top_level, a);
				LPT_INFO("search", "Finished run: max R(s)=" << _best_node->R << " visited: " << _visited.size() );
				//LPT_INFO("search", "Action ID:" << a);
				//LPT_INFO("search", "iw.hxx - LINE 428 - best node :" << _best_node->action << _model.getTask().getGroundActions()[_best_node->action]->getName());
				LPT_INFO("search", "Action:" << _model.getTask().getGroundActions()[a]->getName());
				std::vector<NodePT> _(_optimal_paths.size(), nullptr);
				_optimal_paths.swap(_);
				_evaluator.reset();
			}
		}
		else
			run(s, _config._max_width, nullptr, (ActionIdT)0);
		return extract_plan( _best_node, plan);
	}

    //! Returns true iff there is an actual plan (i.e. because the given solution node is non-null)
    bool extract_plan(const NodePT& solution_node, PlanT& plan) const {
        if (!solution_node) return false;
        assert(plan.empty());

        NodePT node = solution_node;

        while (node->parent) {
            plan.push_back(node->action);
            node = node->parent;
        }

        std::reverse(plan.begin(), plan.end());
        return true;
    }

	bool run(const StateT& seed, unsigned max_width, NodePT top_level, ActionIdT a ) {
		if (_verbose) LPT_INFO("search", "Simulation - Starting IW Simulation");

		std::shared_ptr<DeactivateZCC> zcc_setting = nullptr;
		if (!_config._enforce_state_constraints ) {
			LPT_INFO("search", ":Simulation - Deactivating zero crossing control");
			zcc_setting = std::make_shared<DeactivateZCC>();
		}

		NodePT root;
		if ( top_level == nullptr )
			root = std::make_shared<NodeT>(seed, _stats.generated());
		else
			root = std::make_shared<NodeT>( seed, a, top_level, _stats.generated());

		_stats.generation();
		mark_seed_subgoals(root);

		auto nov =_evaluator.evaluate(*root);
		assert(nov==1 || nov==std::numeric_limits<unsigned char>::max());
		update_novelty_counters_on_generation(nov);

// 		LPT_DEBUG("cout", "Simulation - Seed node: " << *root);

		assert(max_width <= 2); // The current swapping-queues method works only for up to width 2, but is trivial to generalize if necessary
		if (top_level == nullptr ) {
			evaluate_reward(root);
			_stats.set_initial_reward(root->R);
	        _best_node = root;
			_stats.update_best_reward(_best_node->R);
		} else {
			evaluate_reward(root);
			if ( _best_node == nullptr ) {
				_stats.set_initial_reward(root->R);
				_best_node = root;
			}
			update_best_node(root);
		}
		OpenListT open_w1, open_w2;
		OpenListT open_w1_next, open_w2_next; // The queues for the next depth level.

		open_w1.insert(root);

		while (true) {
			while (!open_w1.empty() || !open_w2.empty()) {
				NodePT current = open_w1.empty() ? open_w2.next() : open_w1.next();

				// Expand the node
				update_novelty_counters_on_expansion(current->_w);
				_stats.expansion();
				for (const auto& a : _model.applicable_actions(current->state, _config._enforce_state_constraints)) {
					StateT s_a = _model.next( current->state, a );
					NodePT successor = std::make_shared<NodeT>(std::move(s_a), a, current, _stats.generated());
					_stats.generation();
					evaluate_reward(successor);
					update_best_node(successor);
					unsigned char novelty = _evaluator.evaluate(*successor);
					update_novelty_counters_on_generation(novelty);

					// LPT_INFO("search", "Simulation - Node generated: " << *successor);
					if (_config._log_search )
						_visited.push_back(successor);

					if (process_node(successor)) {  // i.e. all subgoals have been reached before reaching the bound
						report("All subgoals reached");
						return true;
					}

					if (novelty <= max_width && novelty == 1) open_w1_next.insert(successor);
					else if (novelty <= max_width && novelty == 2) open_w2_next.insert(successor);
				}

			}
			// We've processed all nodes in the current depth level.
			open_w1.swap(open_w1_next);
			open_w2.swap(open_w2_next);

			if (open_w1.empty() && open_w2.empty()) break;
		}

		report("State space exhausted");
		return false;
	}

	void update_novelty_counters_on_expansion(unsigned char novelty) {
	}

	void update_novelty_counters_on_generation(unsigned char novelty) {
		if (novelty == 1) _stats.w1_node();
		else if (novelty== 2) _stats.w2_node();
		else _stats.wgt2_node();
	}

	void report(const std::string& result) const {
		if (!_verbose) return;
		LPT_INFO("search", "Simulation - Result: " << result);
		LPT_INFO("search", "Simulation - Num reached subgoals: " << (_model.num_subgoals() - _unreached.size()) << " / " << _model.num_subgoals());
		LPT_INFO("search", "Simulation - Generated nodes with w=1 " << _stats.num_w1_nodes());
		LPT_INFO("search", "Simulation - Generated nodes with w=2 " << _stats.num_w2_nodes());
		LPT_INFO("search", "Simulation - Generated nodes with w>2 " << _stats.num_wgt2_nodes());
		if (! _config._log_search ) return;
		// Dump optimal_paths and visited into JSON document
		dump_search_tree( *this, "iw.lookahead.json");
	}

protected:

	//! Returns true iff all goal atoms have been reached in the IW search
	bool process_node(NodePT& node) {
		if (_config._complete) return process_node_complete(node);

		const StateT& state = node->state;

		// We iterate through the indexes of all those goal atoms that have not yet been reached in the IW search
		// to check if the current node satisfies any of them - and if it does, we mark it appropriately.
		for (auto it = _unreached.begin(); it != _unreached.end(); ) {
			unsigned subgoal_idx = *it;

			if (_model.goal(state, subgoal_idx)) {
                // MRJ: Reward function
				_stats.generation_g_decrease();
// 				node->satisfies_subgoal = true;
// 				_all_paths[subgoal_idx].push_back(node);
				if (!_optimal_paths[subgoal_idx]) _optimal_paths[subgoal_idx] = node;
				it = _unreached.erase(it);
			} else {
				++it;
			}
		}

		// As soon as all nodes have been processed, we return true so that we can stop the search
		return _unreached.empty();
	}

	//! Returns true iff all goal atoms have been reached in the IW search
	bool process_node_complete(NodePT& node) {
		const StateT& state = node->state;

		for (unsigned i = 0; i < _model.num_subgoals(); ++i) {
			if (!_in_seed[i] && _model.goal(state, i)) {
// 				node->satisfies_subgoal = true;
				_stats.generation_g_decrease();
				if (!_optimal_paths[i]) _optimal_paths[i] = node;
				_unreached.erase(i);
			}
		}
 		return _unreached.empty();
		//return false; // return false so we don't interrupt the processing
	}

    void update_best_node( const NodePT& node ) {
				// GJF: Use the accumulated reward to determine the best node.
				// Only use generation to break ties.
				// GJF: We update the reward if the new state is after the old one
				// chronologically.
				if ( ((node->R > _best_node->R
					|| (node->R == _best_node->R && _best_node->g < node->g )) &&
					(! different_time_step(node->state, _best_node->state))) ||
					state_after(node->state, _best_node->state) ) {
						_stats.update_best_reward(node->R);
						_stats.update_depth_best_reward(node->g);
            _best_node = node;
						//LPT_INFO("search","Updated best reward to: " << _best_node->R);
						//LPT_INFO("search","Best g updated to: " << _best_node->g);
		}
    }

	void mark_seed_subgoals(const NodePT& node) {
		std::vector<bool> _(_model.num_subgoals(), false);
		_in_seed.swap(_);
		_unreached.clear();
		for (unsigned i = 0; i < _model.num_subgoals(); ++i) {
			if (_model.goal(node->state, i)) {
				_in_seed[i] = true;
			} else {
				_unreached.insert(i);
			}
		}
	}

// public:
// 	const std::unordered_set<NodePT>& get_relevant_nodes() const { return _visited; }
};

} } // namespaces
