
#pragma once

#include <search/algorithms/lookahead/iw.hxx>
#include <search/drivers/base.hxx>

#include <models/simple_state_model.hxx>
#include <search/drivers/sbfws/stats.hxx>
#include <search/drivers/sbfws/features/features.hxx>

namespace fs0 { class Config; }

namespace fs0 { namespace drivers { namespace online {

//! A creator for an online IW algorithm
class IteratedWidthDriver : public EmbeddedDriver {
public:
    typedef typename SimpleStateModel::StateT
        StateT; // State type
    typedef lookahead::IWNode<SimpleStateModel::StateT,GroundAction>
        NodePT; // Node pointer type
    typedef lapkt::novelty::GenericFeatureSetEvaluator<StateT>
        FeatureEvaluatorT; // Feature evaluator
    typedef lookahead::IW<NodePT, SimpleStateModel, bfws::IntNoveltyEvaluatorI, FeatureEvaluatorT>
        EngineT; // Engine type
    typedef	std::unique_ptr<EngineT>
        EnginePT; // Pointer type

    virtual void prepare(const SimpleStateModel& problem, const Config& config, const std::string& out_dir) override;

    virtual void dispose() override;

    virtual ExitCode search() override;

	virtual ExitCode search(const SimpleStateModel& problem, const Config& config, const std::string& out_dir, float start_time) override;

    virtual ~IteratedWidthDriver();
protected:
	bfws::BFWSStats _stats;

    ExitCode
    do_search1(const SimpleStateModel& model, const FeatureEvaluatorT& featureset, const Config& config, const std::string& out_dir, float start_time);

    EnginePT
    create(const Config& config, const FeatureEvaluatorT& featureset, const SimpleStateModel& model, bfws::BFWSStats& stats);

    EnginePT                                _engine;
    std::shared_ptr<FeatureEvaluatorT>      _feature_evaluator;
};


} } } // namespaces
