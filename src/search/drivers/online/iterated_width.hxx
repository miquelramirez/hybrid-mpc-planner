
#pragma once

#include <search/drivers/sbfws/stats.hxx>
#include <models/simple_state_model.hxx>
#include <search/algorithms/iterated_width.hxx>

namespace fs0 { class Config; }

namespace fs0 { namespace drivers { namespace online {

//! A creator for an IW algorithm
class IteratedWidthDriver : public EmbeddedDriver {
public:
    using StateT = typename SimpleStateModel::StateT;


	ExitCode search(const SimpleStateModel& problem, const Config& config, const std::string& out_dir, float start_time) override;

protected:
	bfws::BFWSStats _stats;

    template <typename NoveltyEvaluatorT, typename FeatureEvaluatorT>
    ExitCode
    do_search1(const SimpleStateModel& model, FeatureEvaluatorT&& featureset, const Config& config, const std::string& out_dir, float start_time);
};


} } } // namespaces
