

#include <search/drivers/online/iterated_width.hxx>
#include <lapkt/novelty/features.hxx>
#include <problem_info.hxx>
#include <search/drivers/setups.hxx>
#include <search/utils.hxx>


#include <search/novelty/fs_novelty.hxx>

#include <utils/config.hxx>


namespace fs0 { namespace drivers { namespace online {

IteratedWidthDriver::~IteratedWidthDriver() {}

void
IteratedWidthDriver::prepare(const SimpleStateModel& model, const Config& config, const std::string& out_dir) {
	bfws::FeatureSelector<StateT> selector(ProblemInfo::getInstance());
	_feature_evaluator = std::make_shared<FeatureEvaluatorT>();
	selector.select(*_feature_evaluator);
	_engine = create(config, *_feature_evaluator, model, _stats);
}

void
IteratedWidthDriver::dispose(/* arguments */) {
	_engine.reset(nullptr);
}

ExitCode
IteratedWidthDriver::search() {

	if ( _engine == nullptr ) {
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

IteratedWidthDriver::EnginePT
IteratedWidthDriver::create(const Config& config, const IteratedWidthDriver::FeatureEvaluatorT& featureset, const SimpleStateModel& model, lookahead::IteratedWidthStats& stats) {
	using FeatureValueT = typename bfws::IntNoveltyEvaluatorI::FeatureValueT;

	unsigned max_novelty = config.getOption<int>("width.max");

	bool do_complete_search = config.getOption<bool>("lookahead.iw.complete", false);
	bool verbose = config.getOption<bool>("lookahead.iw.verbose", false);

	typename EngineT::Config cfg( do_complete_search, max_novelty, config);

    bfws::NoveltyFactory<FeatureValueT> factory(model.getTask(), bfws::SBFWSConfig::NoveltyEvaluatorType::Generic, true, max_novelty);
	auto evaluator = factory.create_evaluator(max_novelty);

	return EnginePT(new EngineT(model, std::move(featureset), evaluator , cfg, stats, verbose ));
}


ExitCode
IteratedWidthDriver::search(const SimpleStateModel& model, const Config& config, const std::string& out_dir, float start_time) {
	bfws::FeatureSelector<StateT> selector(ProblemInfo::getInstance());

	return do_search1(model, selector.select(), config, out_dir, start_time);
}


ExitCode
IteratedWidthDriver::do_search1(const SimpleStateModel& model, const IteratedWidthDriver::FeatureEvaluatorT& featureset, const Config& config, const std::string& out_dir, float start_time) {
	_engine = create(config, featureset, model, _stats);

	return drivers::Utils::do_search(*_engine, model, out_dir, start_time, _stats);
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
}

} } } // namespaces
