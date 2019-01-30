

#include <search/drivers/online/iterated_width.hxx>
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

IteratedWidthDriver::~IteratedWidthDriver() {}

void
IteratedWidthDriver::prepare(const SimpleStateModel& model, const Config& config, const std::string& out_dir) {
	bfws::FeatureSelector<StateT> selector(ProblemInfo::getInstance());
	_feature_evaluator = std::make_shared<FeatureEvaluatorT>();
	selector.select(*_feature_evaluator);
	create(config, *_feature_evaluator, model, _stats);
	//setup_reward_function(config, model.getTask());
	//LPT_INFO("search", "[IteratedWidthDriver::prepare()(" << this << ")] Pointer to problem associated with engine "
	//					<< _engine.get() << " is " << &(_engine->_model.getTask()) << " via model " << &model);
}

void
IteratedWidthDriver::dispose(/* arguments */) {
	//_engine.reset(nullptr);
	_engine = nullptr;
}

ExitCode
IteratedWidthDriver::search() {
	//LPT_INFO("search", "[IteratedWidthDriver::search()(" << this << ")] Pointer to problem associated with engine "
	//					<< _engine.get() << " is " << &(_engine->_model.getTask()) << " via model " << &(_engine->_model));
	if ( _engine.get() == nullptr ) {
		throw std::runtime_error("[IteratedWidthDriver::search()]: search engine was not prepared!");
	}
	LPT_INFO("search", "Resetting search call statistics cached in driver...");
	reset_results();
	float start_time = aptk::time_used();
	try {
		LPT_INFO("search", "Resetting search engine internal data structures...");
		_engine->reset();
		LPT_INFO("search", "Search started...");
		solved = _engine->solve_model( plan );
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
IteratedWidthDriver::create(const Config& config, const IteratedWidthDriver::FeatureEvaluatorT& featureset, const SimpleStateModel& model, lookahead::IteratedWidthStats& stats) {
	using FeatureValueT = typename bfws::IntNoveltyEvaluatorI::FeatureValueT;

	unsigned max_novelty = config.getOption<int>("width.max");

	bool do_complete_search = config.getOption<bool>("lookahead.iw.complete", false);
	bool verbose = config.getOption<bool>("lookahead.iw.verbose", false);

	typename EngineT::Config cfg( do_complete_search, max_novelty, config);

    bfws::NoveltyFactory<FeatureValueT> factory(model.getTask(), bfws::SBFWSConfig::NoveltyEvaluatorType::Generic, true, max_novelty);
	auto evaluator = factory.create_evaluator(max_novelty);

	_engine = std::make_unique<EngineT>(model, std::move(featureset), evaluator , cfg, stats, verbose );
	setup_reward_function(config, model.getTask());
	//LPT_INFO("search", "[IteratedWidthDriver::create()(" << this << ")] Pointer to problem associated with engine "
	//					<< _engine.get() << " is " << &(_engine->_model.getTask()) << " via model " << &model);
}

void
IteratedWidthDriver::setup_reward_function( const Config& cfg, const Problem& prob ) {
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

	throw std::runtime_error("IteratedWidthDriver::setup_reward_function() : No reward function has been specified!");
}



ExitCode
IteratedWidthDriver::search(const SimpleStateModel& model, const Config& config, const std::string& out_dir, float start_time) {
	bfws::FeatureSelector<StateT> selector(ProblemInfo::getInstance());
	return do_search1(model, selector.select(), config, out_dir, start_time);
}


ExitCode
IteratedWidthDriver::do_search1(const SimpleStateModel& model, const IteratedWidthDriver::FeatureEvaluatorT& featureset, const Config& config, const std::string& out_dir, float start_time) {
	create(config, featureset, model, _stats);
	Utils::SearchExecution<SimpleStateModel> exec_manager(model);
	EngineOptions opt;
	opt.setOutputDir(out_dir);
	return exec_manager.do_search(*_engine,EngineOptions(), start_time, _stats);
}

void
IteratedWidthDriver::archive_scalar_stats( rapidjson::Document& doc ) {
	EmbeddedDriver::archive_scalar_stats(doc);
	using namespace rapidjson;
    Document::AllocatorType& allocator = doc.GetAllocator();
	doc.AddMember( "expanded", Value(_stats.expanded()).Move(), allocator );
	doc.AddMember( "generated", Value(_stats.generated()).Move(), allocator );
	doc.AddMember( "num_w1_nodes", Value(_stats.num_w1_nodes()).Move(), allocator );
	doc.AddMember( "num_w2_nodes", Value(_stats.num_w2_nodes()).Move(), allocator );
	doc.AddMember( "num_wgt2_nodes", Value(_stats.num_wgt2_nodes()).Move(), allocator );
	doc.AddMember( "initial_reward", Value(_stats.initial_reward()).Move(), allocator );
	doc.AddMember( "max_reward", Value(_stats.max_reward()).Move(), allocator );
	doc.AddMember( "max_depth", Value(_stats.depth_max_reward()).Move(), allocator);
}


} } } // namespaces
