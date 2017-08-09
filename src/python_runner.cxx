#include <python_runner.hxx>
#include <components.hxx>
#include <problem.hxx>
#include <utils/loader.hxx>
#include <search/search.hxx>

#include <search/runner.hxx>
#include <utils/config.hxx>
#include <utils/logging.hxx>
#include <lapkt/tools/logging.hxx>
#include <lapkt/tools/resources_control.hxx>

#include <search/result.hxx>

#include <search/engines/registry.hxx>
#include <actions/hybrid_checker.hxx>
#include <utils/printers/printers.hxx>
#include <utils/config.hxx>
#include <utils/logging.hxx>

// MRJ: Deactivated for now
// #include <search/ompl/default_engine.hxx>

namespace fs0 { namespace drivers {

class SingletonLock {
    PythonRunner& _runner;

public:
    SingletonLock( PythonRunner& r )
        : _runner(r) {
            LanguageInfo::instance( std::move(_runner._lang_info ));
            ProblemInfo::setInstance( std::move(_runner._problem_info));
            Problem::setInstance( std::move(_runner._problem) );
            Config::setAsGlobal( std::move(_runner._instance_config) );
        }

    ~SingletonLock() {
        _runner._problem_info = ProblemInfo::claimOwnership();
        _runner._lang_info = fstrips::LanguageInfo::claimOwnership();
        _runner._problem = Problem::claimOwnership();
        _runner._instance_config = Config::claimOwnership();
    }
};

PythonRunner::PythonRunner() :
    _plan_duration( 0.0 ),
    _setup_time( 0.0 ),
    _search_time( 0.0 ),
    _simulation_time( 0.0 ),

    _result( aptk::human_readable_result(aptk::HybridSearchAlgorithmResult::NeedsToRunFirst) ),
    _timeout( 10 ),
    _data_dir( "data" ),
    _config( "./config.json"),
    _output_dir( "." ),
    _time_step( 1.0 ),
    _control_eps( 0.01 ),
    _time_horizon( 100 ),
    _budget(std::numeric_limits<unsigned>::max()),
    _retry_when_failed(false),
    _simulate_plan(false),
    _verify_plan( false ),
    _current_driver( nullptr ) {

}

PythonRunner::PythonRunner( const PythonRunner& other ) {
    _setup_time = other._setup_time;
    _search_time = other._search_time;
    _result = other._result;
    _timeout = other._timeout;
    _data_dir = other._data_dir;
    _config = other._config;
    _output_dir = other._output_dir;
    _time_step = other._time_step;
    _control_eps = other._control_eps;
    _time_horizon = other._time_horizon;
    _budget = other._budget;
    _retry_when_failed = other._retry_when_failed;
    _simulate_plan = other._simulate_plan;
    _verify_plan = other._verify_plan;
    _problem = nullptr;
    _instance_config = nullptr;
}

PythonRunner::~PythonRunner() {

}

void
PythonRunner::setup() {
    /* code */
    float t0 = aptk::time_used();

    lapkt::tools::Logger::init(_options.getOutputDir() + "/logs");

    _instance_config = std::unique_ptr<Config>(new Config(_config));
    Config::setAsGlobal( std::move(_instance_config) );

    LPT_INFO("main", "Generating the problem (" << get_data_dir() << ")... ");
    auto data = Loader::loadJSONObject( get_data_dir() + "/problem.json");

    //! This will generate the problem and set it as the global singleton instance
    generate(data, get_data_dir());


    Config& config = Config::instance();

    LPT_INFO("main", "Problem instance loaded:" << std::endl << Problem::getInstance());

    LPT_INFO("main", "Applying options to planner configuration:");

    double horizon = get_horizon();
    LPT_INFO("main", "\tHorizon: " << horizon << " secs" );
    config.setHorizonTime( horizon );

    double time_step = get_delta_max();
    LPT_INFO("main", "\tTime Step: " << time_step << " secs" );
    config.setTimeStep( time_step );

    double control_eps = get_delta_min();
    LPT_INFO("main", "\tControl Epsilon: " << control_eps << " secs");
    // MRJ: Not used yet
    // config.setControlEpsilon( control_eps );

    LPT_INFO("main", "\tLookahead Budget: " << get_budget() << " secs");
    // MRJ: Not used yet
    // config.setBudget( get_budget() );

    LPT_INFO("main", "Planner configuration: " << std::endl << config);

    LPT_INFO("main", "Indexing state variables..." );
    index_state_variables();
    _setup_time = aptk::time_used() - t0;

    // Singleton management: note that we're not using the Lock class because
    // the pointers are initialised during this method
    _lang_info = fstrips::LanguageInfo::claimOwnership();
    _problem_info = ProblemInfo::claimOwnership();
    _problem = Problem::claimOwnership();
    _instance_config = Config::claimOwnership();
}

void
PythonRunner::index_state_variables() {
    for ( fs0::VariableIdx x = 0; x < Problem::getInstance().getProblemInfo().getNumVariables(); x++ )
        _var_index[ Problem::getInstance().getProblemInfo().getVariableName(x) ] = x;
}

void
PythonRunner::set_initial_state( bp::dict& new_state ) {
    ProblemInfo::setInstance( std::move(_problem_info));
    Problem::setInstance( std::move(_problem) );
    Config::setAsGlobal( std::move(_instance_config) );
    State::ptr s = std::make_shared<State>(*Problem::getInstance().getInitialState());
    const fs0::ProblemInfo& info = Problem::getInstance().getProblemInfo();
    const bp::list& entries = new_state.items();
    for ( unsigned k = 0; k < bp::len(entries); k++ ) {
        std::string var_name = bp::extract<std::string>(entries[k][0]);
        auto it = _var_index.find(var_name);
        if (it == _var_index.end()) {
            throw std::runtime_error( "Error: PythonRunner::set_initial_state : unknown variable found in state: " + var_name );
        }
        VariableIdx x = it->second;
        ObjectIdx value;
        if ( info.getVariableGenericType( x ) == ObjectType::NUMBER )
            value = bp::extract<float>(entries[k][1]);
        else if ( info.getVariableGenericType( x ) == ObjectType::OBJECT) {
            std::string obj_name = bp::extract<std::string>( bp::str(entries[k][1]).encode("utf-8"));

            value = info.getObjectId(obj_name);
        }

        else if ( info.getVariableGenericType( x ) == ObjectType::INT)
            value = bp::extract<int>(entries[k][1]);
        else if ( info.getVariableGenericType( x ) == ObjectType::BOOL)
            value = bp::extract<int>(entries[k][1]);
        s->setValue( x, value );
    }
    _problem = Problem::claimOwnership();
    _problem->setInitialState( s );
    _problem_info = ProblemInfo::claimOwnership();
    _instance_config = Config::claimOwnership();
}


void
PythonRunner::solve() {
    ProblemInfo::setInstance( std::move(_problem_info ));
    Problem::setInstance( std::move(_problem) );
    Config::setAsGlobal( std::move(_instance_config) );
    float t0 = aptk::time_used();
    Config& config = Config::instance();

    if (    config.getModelTag() == "hybrid"
            || config.getModelTag() == "ompl" ) {

        if ( config.getModelTag() == "hybrid" ) {
            solve_with_hybrid_planner();
        }
        else
            solve_with_ompl_planner();
    }
    else {
        std::string msg = "Run-time Error: Unsupported planning model requested: '";
        msg += config.getModelTag();
        msg += "''";
        LPT_INFO( "main", msg );
        throw std::runtime_error(msg);
    }
    _search_time = aptk::time_used() - t0;
    _problem_info = ProblemInfo::claimOwnership();
    _problem = Problem::claimOwnership();
    _instance_config = Config::claimOwnership();
}


void
PythonRunner::export_plan( ) {
    double      timing;
    unsigned    act_index;
    const Problem& p = Problem::getInstance();
    _plan = bp::list();
    for ( auto entry : _native_plan ) {
        std::tie( timing, act_index) = entry;
        if (act_index >= p.getGroundActions().size()) continue;
        bp::tuple py_entry = bp::make_tuple(timing, p.getGroundActions()[act_index]->getFullName());
        _plan.append(py_entry);
    }
}

void
PythonRunner::solve_with_hybrid_planner() {
    Config& config = Config::instance();
    LPT_INFO( "main", "Starting Hybrid Planner...." << std::endl );
    fs0::HybridStateModel model(Problem::getInstance());
    auto creator = fs0::engines::EngineRegistry::instance().get(config.getEngineTag());
    auto engine = creator->create(config, model);

    if ( engine.get() == nullptr ) {
        std::string msg = "Run-time Error: Failed to instantiate search engine '";
        msg += config.getEngineTag();
        msg += "'\n for planning model '";
        msg += config.getModelTag();
        msg += "'";
        LPT_INFO( "main", msg )
        throw std::runtime_error(msg);
    }
    _native_plan.clear();
	float t0 = aptk::time_used();
	bool solved = engine->solve_model( _native_plan );
	float total_time = aptk::time_used() - t0;
	float accum_time = total_time;
    bool valid = false;
    double validation_time = 0.0;
    _result = aptk::human_readable_result(engine->outcome);
    LPT_INFO( "main", "Result: " <<  _result << std::endl );
    while ( Config::instance().getReTryWhenFailed() &&
            (engine->outcome == aptk::HybridSearchAlgorithmResult::DeadEnd
            || engine->outcome == aptk::HybridSearchAlgorithmResult::TerminalNonGoal
            || engine->outcome == aptk::HybridSearchAlgorithmResult::ExhaustedSearchSpace) ) {

        double Dt = Config::instance().getTimeStep();
        // 1/1000th time units is max precision
        if ( Dt <= Config::instance().getControlEpsilon() ) break;

        LPT_INFO( "main", "Search failed, trying with smaller time step");
        double nextDt = Dt / 2.0;
        LPT_INFO( "main", "Current Delta Time: " << Dt << " next Delta Time: " << nextDt );
        Config::instance().setTimeStep( nextDt );

        t0 = aptk::time_used();
        solved = engine->solve_model( _native_plan );
        total_time = aptk::time_used() - t0;
        accum_time += total_time;
        _result = aptk::human_readable_result(engine->outcome);
        LPT_INFO( "main", "Result: " << _result << std::endl );
    }

    std::ofstream json_out( get_output_dir() + "/results.json" );
	std::string eval_speed = (total_time > 0) ? std::to_string((float) engine->generated / total_time) : "0";
	json_out << "{" << std::endl;
    json_out << "\t\"state_variables\": " << Problem::getInstance().getProblemInfo().getNumVariables() << "," << std::endl;
    json_out << "\t\"instant_actions\": " << Problem::getInstance().getGroundActions().size() << "," << std::endl;
    json_out << "\t\"processes\": " << Problem::getInstance().getGroundProcesses().size() << "," << std::endl;
	json_out << "\t\"search_time\": " << accum_time << "," << std::endl;
	json_out << "\t\"generated\": " << engine->generated << "," << std::endl;
	json_out << "\t\"expanded\": " << engine->expanded << "," << std::endl;
	json_out << "\t\"eval_per_second\": " << eval_speed << "," << std::endl;
    json_out << "\t\"min_search_delta_t\": " << engine->model.smallest_time_step() << "," << std::endl;
    json_out << "\t\"search_delta_t\": " << Config::instance().getTimeStep()  << "," << std::endl;
    json_out << "\t\"lookahead_delta_t\": " << Config::instance().getTimeStep() << "," << std::endl;


	json_out << "\t\"solved\": " << "\"" << aptk::human_readable_result(engine->outcome)  << "\"" << "," << std::endl;
	json_out << "\t\"valid\": " << ( valid ? "true" : "false" ) << "," << std::endl;
    json_out << "\t\"validation_time\": " << validation_time << "," << std::endl;
	json_out << "\t\"plan_length\": " << _native_plan.size() << "," << std::endl;
    json_out << "\t\"plan_duration\": " << engine->plan_duration << "," << std::endl;
    _plan_duration = engine->plan_duration;
	json_out << "\t\"plan\": ";
	if ( solved ) {
		HybridPlanPrinter::printPlanJSON( _native_plan, Problem::getInstance(), json_out);

        export_plan();
    }
	else {
		if ( _native_plan.size() > 0 ) {
			HybridPlanPrinter::printPlanJSON( _native_plan, Problem::getInstance(), json_out);
            export_plan();
		}
		else {
            json_out << "null";
            clear_plan();
        }

	}

	json_out << "," << std::endl;
	json_out << "\t\"metric\": ";
	json_out << "null";
	json_out << std::endl;
	json_out << "}" << std::endl;
	json_out.close();

}


void
PythonRunner::clear_plan() {
    _plan = bp::list();
}

void
PythonRunner::solve_with_ompl_planner() {
    throw std::runtime_error("OMPL Support is currently deactivated!");
    /* MRJ: Deactivated for now
    float timer = 0.0;
    Config& config = Config::instance();
    LPT_INFO( "main", "Starting OMPL search engine...");
    LPT_INFO( "main", "Time out set to: " << get_timeout() << " secs");
    auto theEngine = fs0::engines::ompl::DefaultEngine(config, Problem::getInstance());

    float t0 = aptk::time_used();
    theEngine.solve(get_timeout());
    timer = aptk::time_used() - t0;
    LPT_INFO( "main", "Elapsed Time: " << timer << " secs" << std::endl);
    LPT_INFO( "main", "Result: " << aptk::human_readable_result(theEngine._outcome) << std::endl );

    double validation_time = 0.0;
    bool valid = false;
    bool solved =  theEngine._outcome == aptk::HybridSearchAlgorithmResult::FoundSolution
                    && theEngine.isExactSolution();

    std::ofstream json_out( get_output_dir() + "/results.json" );
    json_out << "{" << std::endl;
    json_out << "\t\"state_variables\": " << Problem::getInstance().getProblemInfo().getNumVariables() << "," << std::endl;
    json_out << "\t\"instant_actions\": " << Problem::getInstance().getGroundActions().size() << "," << std::endl;
    json_out << "\t\"processes\": " << Problem::getInstance().getGroundProcesses().size() << "," << std::endl;
    json_out << "\t\"search_time\": " << timer << "," << std::endl;
    theEngine.printJSON( json_out );
    json_out << "\t\"search_delta_t\": " << Config::instance().getTimeStep()  << "," << std::endl;
    _result = aptk::human_readable_result(theEngine._outcome);
    json_out << "\t\"solved\": " << "\"" << _result << "\"" << "," << std::endl;
    json_out << "\t\"valid\": " << ( valid ? "true" : "false" ) << "," << std::endl;
    json_out << "\t\"validation_time\": " << validation_time << "," << std::endl;
    json_out << "\t\"plan_length\": " << theEngine._plan.size() << "," << std::endl;
    json_out << "\t\"plan_duration\": " << theEngine._plan_duration << "," << std::endl;
    _plan_duration = theEngine._plan_duration;
    json_out << "\t\"plan\": ";
    _native_plan = theEngine._plan;
    if ( solved ) {
        HybridPlanPrinter::printPlanJSON( _native_plan, *_problem, json_out);
        export_plan();
    }
    else {
        if ( theEngine._plan.size() > 0 ) {
            HybridPlanPrinter::printPlanJSON( _native_plan, *_problem, json_out);
            export_plan();
        }
        else {
            json_out << "null";
            clear_plan();
        }
    }

    json_out << "," << std::endl;
    json_out << "\t\"metric\": ";
    if ( _problem->metric() == nullptr )
        json_out << "null";
    else {
        json_out << "\"Not Implemented!\"";
    }
    json_out << std::endl;
    json_out << "}" << std::endl;
    json_out.close();
    */

}

bp::list
PythonRunner::simulate_plan( double duration, double step_size ) {
    ProblemInfo::setInstance( std::move(_problem_info ));
    Problem::setInstance( std::move(_problem) );
    Config::setAsGlobal( std::move(_instance_config) );
    float t0 = aptk::time_used();


    double sim_duration = std::min( duration, _plan_duration );
    LPT_INFO( "main", "Simulating plan for " << sim_duration << " time units");

    std::vector< State::ptr > trace;
    HybridChecker::simulatePlan( Problem::getInstance(),  _native_plan, *Problem::getInstance().getInitialState(), sim_duration, step_size, trace );
    LPT_INFO( "main", "Final simulation state: " << *trace.back() );

    bp::list py_trace;
    const ProblemInfo& info = ProblemInfo::getInstance();

    for ( unsigned k = 0; k < trace.size(); k++ ) {
        State::ptr s_k = trace[k];
        bp::dict py_s_k;
        for ( unsigned x = 0; x < info.getNumVariables(); x++ ) {
            ObjectIdx value = s_k->getValue(x);
            if ( info.getVariableGenericType( x ) == ObjectType::NUMBER )
                py_s_k[info.getVariableName(x)] = (float)value;
            else if ( info.getVariableGenericType( x ) == ObjectType::OBJECT) {
                py_s_k[info.getVariableName(x)] = info.getObjectName( x, value );
            }

            else if ( info.getVariableGenericType( x ) == ObjectType::INT) {
                py_s_k[info.getVariableName(x)] = (int)value;
            }
            else if ( info.getVariableGenericType( x ) == ObjectType::BOOL)
                py_s_k[info.getVariableName(x)] = (bool)(int)value;
        }
        py_trace.append(py_s_k);
    }
    _simulation_time = aptk::time_used() - t0;
    _problem_info = ProblemInfo::claimOwnership();
    _problem = Problem::claimOwnership();
    _instance_config = Config::claimOwnership();
    return py_trace;
}

}}
