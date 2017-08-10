

#include <lapkt/novelty/features.hxx>
#include <search/drivers/online/iterated_width.hxx>
#include <search/algorithms/iterated_width.hxx>
#include <search/novelty/fs_novelty.hxx>
#include <search/utils.hxx>
#include <problem_info.hxx>
#include <search/drivers/setups.hxx>

#include <search/drivers/sbfws/base.hxx>
#include <search/drivers/sbfws/features/features.hxx>


namespace fs0 { namespace drivers { namespace online {


template <typename FeatureEvaluatorT, typename NoveltyEvaluatorT>
std::unique_ptr<FS0IWAlgorithm<SimpleStateModel, FeatureEvaluatorT, NoveltyEvaluatorT>>
create(const Config& config, FeatureEvaluatorT&& featureset, const SimpleStateModel& model, SearchStats& stats) {
	using FeatureValueT = typename NoveltyEvaluatorT::FeatureValueT;

	using EngineT = FS0IWAlgorithm<SimpleStateModel, FeatureEvaluatorT, NoveltyEvaluatorT>;
	using EnginePT = std::unique_ptr<EngineT>;

	unsigned max_novelty = config.getOption<int>("width.max");
    bfws::NoveltyFactory<FeatureValueT> factory(model.getTask(), bfws::SBFWSConfig::NoveltyEvaluatorType::Generic, true, max_novelty);
	return EnginePT(new EngineT(model, 1, max_novelty, std::move(featureset), factory.create_evaluator(max_novelty), stats));
}


ExitCode
IteratedWidthDriver::search(const SimpleStateModel& model, const Config& config, const std::string& out_dir, float start_time) {
    const StateAtomIndexer& indexer = model.getTask().getStateAtomIndexer();

	if (config.getOption<bool>("width.force_generic_evaluator", false)) {
		bfws::FeatureSelector<StateT> selector(ProblemInfo::getInstance());

		LPT_INFO("cout", "FEATURE EVALUATION: Forced to use GenericFeatureSetEvaluator");
		using FeatureEvaluatorT = lapkt::novelty::GenericFeatureSetEvaluator<StateT>;
		return do_search1<bfws::IntNoveltyEvaluatorI, FeatureEvaluatorT>(model, selector.select(), config, out_dir, start_time);
	}
	if (config.getOption<bool>("width.extra_features", false)) {
		bfws::FeatureSelector<StateT> selector(ProblemInfo::getInstance());

		if (selector.has_extra_features()) {
			LPT_INFO("cout", "FEATURE EVALUATION: Extra Features were found!  Using a GenericFeatureSetEvaluator");
			using FeatureEvaluatorT = lapkt::novelty::GenericFeatureSetEvaluator<StateT>;
			return do_search1<bfws::IntNoveltyEvaluatorI, FeatureEvaluatorT>(model, selector.select(), config, out_dir, start_time);
		}
	}

	if (indexer.is_fully_binary()) { // The state is fully binary
		LPT_INFO("cout", "FEATURE EVALUATION: Using the specialized StraightFeatureSetEvaluator<bool>");
		using FeatureEvaluatorT = lapkt::novelty::StraightFeatureSetEvaluator<bool>;
		return do_search1<bfws::BoolNoveltyEvaluatorI, FeatureEvaluatorT>(model, FeatureEvaluatorT(), config, out_dir, start_time);

	}
	/*
	else if (indexer.is_fully_multivalued()) { // The state is fully multivalued
		LPT_INFO("cout", "FEATURE EVALUATION: Using the specialized StraightFeatureSetEvaluator<object_id>");
		using FeatureEvaluatorT = lapkt::novelty::StraightFeatureSetEvaluator<int>;
		return do_search1<IntNoveltyEvaluatorI, FeatureEvaluatorT>(model, FeatureEvaluatorT(), config, out_dir, start_time);

	}*/
	else { // We have a hybrid state and cannot thus apply optimizations
		LPT_INFO("cout", "FEATURE EVALUATION: Using a generic IntegerFeatureEvaluator");
		using FeatureEvaluatorT = bfws::IntegerFeatureEvaluator;
		return do_search1<bfws::IntNoveltyEvaluatorI, FeatureEvaluatorT>(model, FeatureEvaluatorT(), config, out_dir, start_time);
	}
}


template <typename NoveltyEvaluatorT, typename FeatureEvaluatorT>
ExitCode
IteratedWidthDriver::do_search1(const SimpleStateModel& model, FeatureEvaluatorT&& featureset, const Config& config, const std::string& out_dir, float start_time) {
	auto engine = create<FeatureEvaluatorT, NoveltyEvaluatorT>(config, std::move(featureset), model, _stats);

	return drivers::Utils::do_search(*engine, model, out_dir, start_time, _stats);
}





} } } // namespaces
