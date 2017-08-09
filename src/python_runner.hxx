#pragma once

#include <memory>

#include <fs_types.hxx>
#include <problem.hxx>
#include <search/runner.hxx>
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

class PythonRunner {

public:

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
    double      get_plan_duration() { return _plan_duration; }
    //! result - read only, string describing result of last call to planner
    std::string get_result() { return _result; }
    //! time out - time alloted for search (only used by anytime planners)
    double      get_timeout() { return _timeout; }
    void        set_timeout( double t) { _timeout = t; }
    //! data_dir - path to directory where the planner is to find its data
    std::string get_data_dir() { return _data_dir; }
    void        set_data_dir( std::string data) { _data_dir = data; }
    //! config - path to the config file to be used to setup the planner
    std::string get_config() { return _config; }
    void        set_config( std::string cfg) { _config = cfg; }
    //! output_dir - path where the planner is going to leave its output
    std::string get_output_dir( )  { return _output_dir; }
    void        set_output_dir( std::string dir ) { _output_dir = dir; }
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
    //! retry_when_failed - if planner fails to find a plan, give it another
    //! opportunity (used with randomised search algorithms)
    bool        get_retry_when_failed( ) { return _retry_when_failed; }
    void        set_retry_when_failed( bool retry_flag) { _retry_when_failed = retry_flag; }
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

private:


    bp::list                        _plan;
    fs0::dynamics::HybridPlan       _native_plan;
    double                          _plan_duration;
    double                          _setup_time;
    double                          _search_time;
    double                          _simulation_time;
    std::string                     _result;
    unsigned                        _timeout;
    std::string                     _data_dir;
	std::string                     _config;
	std::string                     _output_dir;
    double                          _time_step;
    double                          _control_eps;
    double                          _time_horizon;
    unsigned                        _budget;
    bool                            _retry_when_failed;
	bool		                    _simulate_plan;
	bool		                                 _verify_plan;
    std::unique_ptr<fs0::ProblemInfo>            _problem_info;
    std::unique_ptr<fs0::Problem>                _problem;
    std::map< std::string, fs0::VariableIdx >    _var_index;
    std::unique_ptr<fs0::Config>                _instance_config;
};
