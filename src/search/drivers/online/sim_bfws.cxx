

#include <search/drivers/online/sim_bfws.hxx>
#include <lapkt/novelty/features.hxx>
#include <fs/core/problem_info.hxx>
#include <fs/core/search/drivers/setups.hxx>
#include <fs/core/search/utils.hxx>

#include <fs/core/heuristics/goal_count_signal.hxx>
#include <fs/core/heuristics/error_signal.hxx>
#include <fs/core/heuristics/metric_signal.hxx>

#include <fs/core/search/novelty/fs_novelty.hxx>

#include <fs/core/utils/config.hxx>


namespace fs0 { namespace drivers { namespace online {

SimBFWSDriver::~SimBFWSDriver() {}

void
SimBFWSDriver::prepare(const SimpleStateModel& model, const Config& config, const std::string& out_dir) {
	bfws::FeatureSelector<StateT> selector(ProblemInfo::getInstance());
	_feature_evaluator = std::make_shared<FeatureEvaluatorT>();
	selector.select(*_feature_evaluator);
	create(config, model, _stats);
	//setup_reward_function(config, model.getTask());
	//LPT_INFO("search", "[SimBFWSDriver::prepare()(" << this << ")] Pointer to problem associated with engine "
	//					<< _engine.get() << " is " << &(_engine->_model.getTask()) << " via model " << &model);
}

void
SimBFWSDriver::dispose(/* arguments */) {
	//_engine.reset(nullptr);
	_engine = nullptr;
}

ExitCode
SimBFWSDriver::search() {
	//LPT_INFO("search", "[SimBFWSDriver::search()(" << this << ")] Pointer to problem associated with engine "
	//					<< _engine.get() << " is " << &(_engine->_model.getTask()) << " via model " << &(_engine->_model));
	if ( _engine.get() == nullptr ) {
		throw std::runtime_error("[SimBFWSDriver::search()]: search engine was not prepared!");
	}
	LPT_INFO("search", "Resetting search call statistics cached in driver...");
	reset_results();
	float start_time = aptk::time_used();
	try {
		LPT_INFO("search", "Resetting search engine internal data structures...");
        // MRJ: BFWS doesn't have a reset function, do we need one?
		//_engine->reset();
		LPT_INFO("search", "Search started...");
		solved = _engine->solve_model( plan );
		LPT_INFO("search", "Search finished normally...")
	}
	catch (const std::bad_alloc& ex)
	{
		LPT_INFO("cout", "FAILED TO ALLOCATE MEMORY");
		oom = true;
		dispose(); //needs prepare
	}
	search_time = aptk::time_used() - start_time;
	total_planning_time = aptk::time_used() - start_time;
 	gen_speed = (search_time > 0) ? (float) _stats.generated() / search_time : 0;
	eval_speed = (search_time > 0) ?(float) _stats.expanded() / search_time : 0;
	ExitCode result;
	if (solved) {
		result = ExitCode::PLAN_FOUND;
	} else if (oom) {
		result = ExitCode::OUT_OF_MEMORY;
	} else {
		result = ExitCode::UNSOLVABLE;
	}
	return result;
}

void
SimBFWSDriver::create(const Config& config, const SimpleStateModel& model, bfws::BFWSStats& stats) {
	//using FeatureValueT = typename bfws::IntNoveltyEvaluatorI::FeatureValueT;
    bfws::SBFWSConfig bfws_config(config);


    //bfws::NoveltyFactory<FeatureValueT> factory(model.getTask(), bfws::SBFWSConfig::NoveltyEvaluatorType::Generic, true, max_novelty);
	//auto evaluator = factory.create_evaluator(max_novelty);

	_engine = std::make_unique<EngineT>(model, std::move(*_feature_evaluator), stats, config, bfws_config );
	setup_reward_function(config, model.getTask());
	//LPT_INFO("search", "[SimBFWSDriver::create()(" << this << ")] Pointer to problem associated with engine "
	//					<< _engine.get() << " is " << &(_engine->_model.getTask()) << " via model " << &model);
}

void
SimBFWSDriver::setup_reward_function( const Config& cfg, const Problem& prob ) {
	if ( cfg.getOption<bool>("reward.goal_count", false )) {
		LPT_INFO("search", "Using goal counting reward");
		std::shared_ptr<Reward> r_func = hybrid::GoalCountSignal::create(prob);
		_engine->set_reward_function( r_func );
		return;
	}
	if ( cfg.getOption<bool>("reward.goal_error", false )) {
		LPT_INFO("search", "Using squared goal error reward");
		std::shared_ptr<Reward> r_func = hybrid::SquaredErrorSignal::create_from_goals(prob);
		_engine->set_reward_function( r_func );
		return;
	}
	if ( cfg.getOption<bool>("reward.from_metric", false)) {
		LPT_INFO("search", "Using the specified metric as reward");
		std::shared_ptr<Reward> r_func = hybrid::StateMetricSignal::create(prob);
		_engine->set_reward_function( r_func );
		return;
	}

	throw std::runtime_error("SimBFWSDriver::setup_reward_function() : No reward function has been specified!");
}



ExitCode
SimBFWSDriver::search(const SimpleStateModel& model, const Config& config, const std::string& out_dir, float start_time) {
	bfws::FeatureSelector<StateT> selector(ProblemInfo::getInstance());
    _feature_evaluator = std::make_shared<FeatureEvaluatorT>();
	selector.select(*_feature_evaluator);
	return do_search1(model, config, out_dir, start_time);
}


ExitCode
SimBFWSDriver::do_search1(const SimpleStateModel& model, const Config& config, const std::string& out_dir, float start_time) {
	create(config,  model, _stats);
	Utils::SearchExecution<SimpleStateModel> exec_manager(model);

	EngineOptions opt;
	opt.setOutputDir(out_dir);
	return exec_manager.do_search(*_engine, opt, start_time, _stats);
}

void
SimBFWSDriver::archive_scalar_stats( rapidjson::Document& doc ) {
	EmbeddedDriver::archive_scalar_stats(doc);
	using namespace rapidjson;
    Document::AllocatorType& allocator = doc.GetAllocator();
	doc.AddMember( "expanded", Value(_stats.expanded()).Move(), allocator );
	doc.AddMember( "generated", Value(_stats.generated()).Move(), allocator );
	doc.AddMember( "num_wg1_nodes", Value(_stats.num_wg1_nodes()).Move(), allocator );
	doc.AddMember( "num_wgr1_nodes", Value(_stats.num_wgr1_nodes()).Move(), allocator );
    doc.AddMember( "num_wgr2_nodes", Value(_stats.num_wgr2_nodes()).Move(), allocator );
	doc.AddMember( "num_wgr_wgt2_nodes", Value(_stats.num_wgr_gt2_nodes()).Move(), allocator );
	doc.AddMember( "initial_reward", Value(_stats.initial_reward()).Move(), allocator );
	float selected_reward = _engine->get_best_node() ? _engine->get_best_node()->R : -100000.0;
	doc.AddMember( "max_reward", Value(selected_reward).Move(), allocator );
	unsigned depth_reward = _engine->get_best_node() ? _engine->get_best_node()->g : 0;
	doc.AddMember( "depth_reward", Value(depth_reward).Move(), allocator );
}


} } } // namespaces
