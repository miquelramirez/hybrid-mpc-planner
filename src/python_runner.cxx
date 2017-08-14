#include <python_runner.hxx>
#include <components.hxx>
#include <problem.hxx>
#include <utils/loader.hxx>
#include <utils/config.hxx>
#include <lapkt/tools/logging.hxx>
#include <lapkt/tools/resources_control.hxx>
#include <utils/printers/printers.hxx>
#include <languages/fstrips/language.hxx>
#include <languages/fstrips/operations.hxx>
#include <search/drivers/setups.hxx>
#include <search/drivers/online/registry.hxx>

#include <locale>
#include <codecvt>

// MRJ: Deactivated for now
// #include <search/ompl/default_engine.hxx>

namespace fs0 { namespace drivers {

class SingletonLock {
    PythonRunner& _runner;

public:
    SingletonLock( PythonRunner& r )
        : _runner(r) {
            fstrips::LanguageInfo::setInstance( std::move(_runner._lang_info ));
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
    _setup_time( 0.0 ),
    _search_time( 0.0 ),
    _simulation_time( 0.0 ),
    _timeout( 10 ),
    _time_step( 1.0 ),
    _control_eps( 0.01 ),
    _time_horizon( 100 ),
    _budget(std::numeric_limits<unsigned>::max()),
    _simulate_plan(false),
    _verify_plan( false ),
    _current_driver( nullptr ),
    _state(nullptr),
    _state_model(nullptr) {

}

PythonRunner::PythonRunner( const PythonRunner& other ) {
    _setup_time = other._setup_time;
    _search_time = other._search_time;
    _result = other._result;
    _timeout = other._timeout;
    _time_step = other._time_step;
    _control_eps = other._control_eps;
    _time_horizon = other._time_horizon;
    _budget = other._budget;
    _simulate_plan = other._simulate_plan;
    _verify_plan = other._verify_plan;
    _problem = nullptr;
    _instance_config = nullptr;
    _current_driver = nullptr;
    _options = other._options;
    _state = nullptr;
    _state_model = nullptr;
}

PythonRunner::~PythonRunner() {

}

void PythonRunner::report_stats(const Problem& problem, const std::string& out_dir) {
	const ProblemInfo& info = ProblemInfo::getInstance();
	const AtomIndex& tuple_index = problem.get_tuple_index();
	unsigned n_actions = problem.getGroundActions().size();
	std::ofstream json_out( out_dir + "/problem_stats.json" );
	json_out << "{" << std::endl;

	unsigned num_goal_atoms = fs::all_atoms(*problem.getGoalConditions()).size();
	unsigned num_sc_atoms = 0;
	for ( auto sc : problem.getStateConstraints() ) {
		num_sc_atoms += fs::all_atoms(*sc).size();
	}

	LPT_INFO("cout", "Number of objects: " << info.num_objects());
	LPT_INFO("cout", "Number of state variables: " << info.getNumVariables());
	LPT_INFO("cout", "Number of problem atoms: " << tuple_index.size());
	LPT_INFO("cout", "Number of action schemata: " << problem.getActionData().size());
	LPT_INFO("cout", "Number of (perhaps partially) ground actions: " << n_actions);
	LPT_INFO("cout", "Number of goal atoms: " << num_goal_atoms);
	LPT_INFO("cout", "Number of state constraint atoms: " << num_sc_atoms);


	json_out << "\t\"num_objects\": " << info.num_objects() << "," << std::endl;
	json_out << "\t\"num_state_variables\": " << info.getNumVariables() << "," << std::endl;
	json_out << "\t\"num_atoms\": " << tuple_index.size() << "," << std::endl;
	json_out << "\t\"num_action_schema\": " << problem.getActionData().size() << "," << std::endl;
	json_out << "\t\"num_grounded_actions\": " << n_actions << "," << std::endl;
	json_out << "\t\"num_goal_atoms\": " << num_goal_atoms << "," << std::endl;
	json_out << "\t\"num_state_constraint_atoms\": " << num_sc_atoms;
	json_out << std::endl << "}" << std::endl;
	json_out.close();
}

void
PythonRunner::update( Config& config ) {
    LPT_INFO("main", "Applying options to planner configuration:");

    double horizon = get_horizon();
    LPT_INFO("main", "\tHorizon: " << horizon << " secs" );
    config.setHorizonTime( horizon );

    double time_step = get_delta_max();
    LPT_INFO("main", "\tTime Step: " << time_step << " secs" );
    config.setDiscretizationStep( time_step );

    double control_eps = get_delta_min();
    LPT_INFO("main", "\tControl Epsilon: " << control_eps << " secs");
    // MRJ: Not used yet
    // config.setControlEpsilon( control_eps );

    LPT_INFO("main", "\tLookahead Budget: " << get_budget() << " secs");
    // MRJ: Not used yet
    // config.setBudget( get_budget() );

    LPT_INFO("main", "Planner configuration: " << std::endl << config);
}

void
PythonRunner::setup() {
    /* code */
    float t0 = aptk::time_used();

    lapkt::tools::Logger::init(_options.getOutputDir() + "/logs");
    Config::init(_options.getDriver(), _options.getUserOptions(), _options.getDefaultConfigurationFilename());

    // MRJ: The following two lines make up for the method Config::init()
    _instance_config = std::unique_ptr<Config>(new Config(_options.getDriver(), _options.getUserOptions(), _options.getDefaultConfigurationFilename()));
    Config::setAsGlobal( std::move(_instance_config) );

    LPT_INFO("main", "[PythonRunner::setup] Generating the problem (" << _options.getDataDir() << ")... ");
    //! This will generate the problem and set it as the global singleton instance
    const std::string problem_spec = _options.getDataDir() + "/problem.json";
    auto data = Loader::loadJSONObject( problem_spec);
    LPT_INFO("main", "[PythonRunner::setup] Loaded JSON specification from '" << problem_spec << "'... ");
    auto problem = generate(data, _options.getDataDir());
    LPT_INFO("main", "[PythonRunner::setup] Activated problem model... ");
    Config& config = Config::instance();

    LPT_INFO("main", "[PythonRunner::setup] Problem instance loaded" );
    report_stats( *problem, _options.getOutputDir() );
    update( config );

    LPT_INFO("main", "[PythonRunner::setup] Grounding Actions....");
    _state_model = std::make_shared<SimpleStateModel>(drivers::GroundingSetup::fully_ground_simple_model(*problem));

    LPT_INFO("main", "[PythonRunner::setup] Indexing state variables..." );
    index_state_variables();

    LPT_INFO("main", "[PythonRunner::setup] Preparing Search Engine....");
    _current_driver = online::EngineRegistry::instance().get(_options.getDriver());
    _current_driver->prepare(*_state_model, config, _options.getOutputDir());

    _setup_time = aptk::time_used() - t0;
    LPT_INFO("main", "[PythonRunner::setup] Finished!" );
    // Singleton management: note that we're not using the Lock class because
    // the pointers are initialised during this method
    _lang_info = fstrips::LanguageInfo::claimOwnership();
    _problem_info = ProblemInfo::claimOwnership();
    _problem = Problem::claimOwnership();
    _instance_config = Config::claimOwnership();
}

void
PythonRunner::index_state_variables() {
    for ( fs0::VariableIdx x = 0; x < ProblemInfo::getInstance().getNumVariables(); x++ )
        _var_index[ ProblemInfo::getInstance().getVariableName(x) ] = x;
}

void
PythonRunner::set_initial_state( bp::dict& new_state ) {
    if ( _problem == nullptr ) {
        throw std::runtime_error("[PythonRunner::set_initial_state] Error: before setting states it is necessary to setup the planner");
    }
    SingletonLock lock(*this);
    _state = std::make_shared<State>(Problem::getInstance().getInitialState());
    const ProblemInfo& info = ProblemInfo::getInstance();
    const bp::list& entries = new_state.items();

    std::vector<Atom> facts;

    for ( unsigned k = 0; k < bp::len(entries); k++ ) {
        std::string var_name = bp::extract<std::string>(bp::str(entries[k][0]).encode("utf-8"));


        auto it = _var_index.find(var_name);
        if (it == _var_index.end()) {
            throw std::runtime_error( "Error: PythonRunner::set_initial_state : unknown variable found in state: " + var_name );
        }
        VariableIdx var = it->second;
        object_id value;

        type_id var_type = info.sv_type(var);

		if (var_type == type_id::bool_t) {
            bool tmp = bp::extract<int>(entries[k][1]);
			value =  make_object(tmp);
		} else if (var_type == type_id::float_t) {
			// MRJ: We're using the specialization so the floating point number
			// is stored correctly via type punning
            float tmp = bp::extract<float>(entries[k][1]);
		    value =  make_object(tmp);
		}
		else if (var_type == type_id::int_t) {
            int tmp = bp::extract<int>(entries[k][1]);
		    value =  make_object(type_id::int_t, tmp );
		}
		else if (var_type == type_id::object_t) {
            std::string obj_name = bp::extract<std::string>( bp::str(entries[k][1]).encode("utf-8"));
			value =  info.get_object_id(obj_name);
		}
		else {
			throw std::runtime_error("PythonRunner::set_initial_state() : Cannot load state variable '" + info.getVariableName(var)
									 + "' of type '" + fstrips::LanguageInfo::instance().get_typename(var) + "'");
		}
        facts.push_back( Atom( var, value ));
    }
    _state->accumulate(facts);
    LPT_INFO("search", "Initial state set:" << *_state );
}


void
PythonRunner::solve() {
    // MRJ: Note that we need to set the initial state before "locking in"
    // the singletons
    _problem->setInitialState( *_state );
    SingletonLock lock(*this);
    float t0 = aptk::time_used();
    //Config& config = Config::instance();
    //ExitCode code = _current_driver->search(*_state_model, config, _options.getOutputDir(), 0.0f);

    ExitCode code = _current_driver->search();
    _current_driver->archive_results_JSON( "results.json" );
    _native_plan.interpret_plan( _current_driver->plan );
    export_plan();
    _search_time = aptk::time_used() - t0;
}


void
PythonRunner::export_plan( ) {

    double                  timing;
    const GroundAction*     act;
    _plan = bp::list();
    for ( auto entry : _native_plan.get_control_events() ) {
        std::tie( timing, act) = entry;
        bp::tuple py_entry = bp::make_tuple(timing, act->getName());
        _plan.append(py_entry);
    }
}

void
PythonRunner::solve_with_hybrid_planner() {
    throw std::runtime_error("Deprecated!");
    /*
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
    */
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
    SingletonLock lock(*this);
    float t0 = aptk::time_used();


    double sim_duration = std::min( duration, (double)_native_plan.get_duration() );
    LPT_INFO( "main", "Simulating plan for " << sim_duration << " time units");
    LPT_INFO( "main", "Simulating with dt = " << step_size << " time units");

    _native_plan.simulate( step_size, sim_duration );
    LPT_INFO( "main", "Final simulation state: " << *_native_plan.trajectory().back() );

    bp::list py_trace;
    const ProblemInfo& info = ProblemInfo::getInstance();

    for ( unsigned k = 0; k < _native_plan.trajectory().size(); k++ ) {
        auto s_k = _native_plan.trajectory()[k];
        bp::dict py_s_k;
        for ( unsigned x = 0; x < info.getNumVariables(); x++ ) {
            object_id value = s_k->getValue(x);
            type_id var_type = info.sv_type(x);

    		if (var_type == type_id::bool_t) {
                py_s_k[info.getVariableName(x)] = fs0::value<bool>(value);
    		} else if (var_type == type_id::float_t) {
                py_s_k[info.getVariableName(x)] = fs0::value<float>(value);
    		}
    		else if (var_type == type_id::int_t) {
                py_s_k[info.getVariableName(x)] = fs0::value<int>(value);
    		}
    		else if (var_type == type_id::object_t) {
                py_s_k[info.getVariableName(x)] = info.object_name(value);
    		}
        }
        py_trace.append(py_s_k);
    }
    _simulation_time = aptk::time_used() - t0;
    return py_trace;
}

}}
