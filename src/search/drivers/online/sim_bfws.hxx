
#pragma once

#include <search/algorithms/lookahead/sbfws.hxx>
#include <fs/core/search/drivers/base.hxx>

#include <fs/core/models/simple_state_model.hxx>
#include <fs/core/search/drivers/sbfws/mv_iw_run.hxx>
#include <fs/core/search/drivers/sbfws/features/features.hxx>

namespace fs0 { class Config; }

namespace fs0 { namespace drivers { namespace online {



//! A creator for an online IW algorithm
class SimBFWSDriver : public EmbeddedDriver {
public:
    typedef typename SimpleStateModel::StateT
        StateT; // State type
    typedef lookahead::SBFWSNode<SimpleStateModel::StateT,GroundAction>
        NodePT; // Node pointer type
    typedef lapkt::novelty::GenericFeatureSetEvaluator<StateT>
        FeatureEvaluatorT; // Feature evaluator
    typedef lookahead::SBFWS<SimpleStateModel, FeatureEvaluatorT, bfws::IntNoveltyEvaluatorI, bfws::MultiValuedIWRun, bfws::MultiValuedIWRunNode >
        EngineT; // Engine type
    typedef	std::unique_ptr<EngineT>
        EnginePT; // Pointer type

    virtual void prepare(const SimpleStateModel& problem, const Config& config, const std::string& out_dir) override;

    virtual void dispose() override;

    virtual ExitCode search() override;

	virtual ExitCode search(const SimpleStateModel& problem, const Config& config, const std::string& out_dir, float start_time) override;

    virtual void archive_scalar_stats( rapidjson::Document& doc ) override;

    virtual ~SimBFWSDriver();
    EnginePT                                _engine;
protected:
	bfws::BFWSStats _stats;

    ExitCode
    do_search1(const SimpleStateModel& model, const Config& config, const std::string& out_dir, float start_time);

    void
    create(const Config& config,  const SimpleStateModel& model, bfws::BFWSStats& stats);

    void
    setup_reward_function( const  Config& cfg, const Problem& prob );


    std::shared_ptr<FeatureEvaluatorT>      _feature_evaluator;
};


} } } // namespaces
