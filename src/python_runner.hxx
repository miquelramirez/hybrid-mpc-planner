#pragma once

#include <memory>

#include <fs_types.hxx>
#include <problem.hxx>
#include <fstrips/language_info.hxx>
#include <models/simple_state_model.hxx>
#include <search/drivers/base.hxx>
#include <search/runner.hxx>
#include <search/options.hxx>
#include <utils/config.hxx>
#include <dynamics/hybrid_plan.hxx>
// This include will dinamically point to the adequate per-instance automatically generated file
#include <boost/python.hpp>
#include <lib/rapidjson/document.h>

#include <map>

namespace bp = boost::python;

namespace fs0 {
    class Problem;

}

namespace fs0 { namespace drivers {

class SingletonLock;

class PythonRunner {

public:
    friend class SingletonLock; // To help with the management of singletons

    //! The type of the concrete instance generator function
	typedef std::function<void (const rapidjson::Document&, const std::string&)> ProblemGeneratorType;


    PythonRunner();
    PythonRunner( const PythonRunner& runner );
    ~PythonRunner();

    //! Loads up the data
    void        setup();
    void        set_initial_state( bp::dict& state );
    void        solve();

    //! Properties

    //! plan - read only, contains the last plan computed
    bp::list   get_plan() { return _plan; }
    //! setup_time - read only, time to setup the planner (in seconds)
    double      get_setup_time( ) { return _setup_time; }
    //! search_time - read only, time spent searching for a plan
    double      get_search_time() { return _search_time; }
    //! plan duration - read only, plan duration in time units
    double      get_plan_duration() { return _native_plan.get_duration(); }
    //! result - read only, string describing result of last call to planner
    std::string get_result() { return _result; }
    //! time out - time alloted for search (only used by anytime planners)
    double      get_timeout() { return _timeout; }
    void        set_timeout( double t) { _timeout = t; }
    //! driver - select search engine to be used
    std::string get_search_driver() { return _options.getDriver(); }
    void        set_search_driver( std::string s ) { _options.setDriver(s); }
    //! data_dir - path to directory where the planner is to find its data
    std::string get_data_dir() { return _options.getDataDir(); }
    void        set_data_dir( std::string data) { _options.setDataDir(data); }
    //! config - path to the config file to be used to setup the planner
    std::string get_config() { return _options.getDefaultConfigurationFilename(); }
    void        set_config( std::string cfg) { _options.setDefaultConfigurationFilename(cfg); }
    //! output_dir - path where the planner is going to leave its output
    std::string get_output_dir( )  { return _options.getOutputDir(); }
    void        set_output_dir( std::string dir ) { _options.setOutputDir(dir); }
    //! user options
    std::string get_user_option( std::string s ) { return _options.getUserOption(s); }
    void        set_user_option( std::string name, std::string value ) { _options.setUserOption( name, value ); }
    //! delta_max - maximum duration of intervals and motions
    double      get_delta_max( ) { return _time_step; }
    void        set_delta_max( double t) { _time_step = t; }
    //! delta_min - minimum duration of intervals and motions
    double      get_delta_min( ) { return _control_eps; }
    void        set_delta_min( double t) { _control_eps = t; }
    //! horizon - maximum duration of plans (by default is set to "infty")
    double      get_horizon( ) { return _time_horizon; }
    void        set_horizon( double t) { _time_horizon = t; }
    //! budget - maximum number of states to be generated during search
    unsigned    get_budget( ) { return _budget; }
    void        set_budget( unsigned B) { _budget = B; }
    //! simulate_plan - simulates the plan found (useful for visualization and debugging)
    bp::list    simulate_plan( double duration, double step_size );
    double      get_simulation_time() { return _simulation_time; }

    //! verify_plan - verifies the plan found (useful for debugging purposes)
    bool        get_verify_plan( ) { return _verify_plan; }
    void        set_verify_plan( bool flag) { _verify_plan = flag; }

protected:

    void        export_plan();
    void        clear_plan();
    void        solve_with_hybrid_planner();
    void        solve_with_ompl_planner();

    void        index_state_variables();

    void        report_stats(const Problem& problem, const std::string& out_dir);
    void        update(Config& cfg);
private:


    bp::list                                _plan;
    dynamics::HybridPlan                    _native_plan;
    EngineOptions                           _options;
    double                                  _setup_time;
    double                                  _search_time;
    double                                  _simulation_time;
    std::string                             _result;
    unsigned                                _timeout;
    double                                  _time_step;
    double                                  _control_eps;
    double                                  _time_horizon;
    unsigned                                _budget;
	bool		                            _simulate_plan;
	bool		                            _verify_plan;
    std::unique_ptr<ProblemInfo>            _problem_info;
    std::unique_ptr<fstrips::LanguageInfo>  _lang_info;
    std::unique_ptr<Problem>                _problem;
    std::map< std::string, VariableIdx >    _var_index;
    std::unique_ptr<Config>                 _instance_config;
    EmbeddedDriver*                         _current_driver;
    std::shared_ptr<State>                  _state;
    std::shared_ptr<SimpleStateModel>       _state_model;
};

}} // namespace
