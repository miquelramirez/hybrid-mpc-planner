#include <python_runner.hxx>
#include <fs/core/problem.hxx>
#include <fs/core/utils/loader.hxx>
#include <fs/core/utils/config.hxx>
#include <lapkt/tools/logging.hxx>
#include <lapkt/tools/resources_control.hxx>
#include <fs/core/utils/printers/printers.hxx>
#include <fs/core/languages/fstrips/language.hxx>
#include <fs/core/languages/fstrips/operations.hxx>
#include <fs/core/search/drivers/setups.hxx>
#include <search/drivers/online/registry.hxx>
#include <cstring>
#include <rapidjson/document.h>
#include <fs/core/fstrips/loader.hxx>
#include <fs/core/utils/loader.hxx>
#include <fs/core/utils/component_factory.hxx>

#include <dlfcn.h> // For run-time symbol loading in Linux

#include <locale>
#include <codecvt>
// For Modern C++ friendly loading
#include <boost/function_types/components.hpp>
#include <boost/function_types/function_pointer.hpp>


fs0::Problem* generate(const rapidjson::Document& data, const std::string& data_dir) {
	fs0::BaseComponentFactory factory;

	LPT_INFO( "main", "[components::generate] Loading language info...")
	fs0::fstrips::LanguageJsonLoader::loadLanguageInfo(data);

	LPT_INFO( "main", "[components::generate] Loading problem info...")
	fs0::Loader::loadProblemInfo(data, data_dir, factory);

    //MRJ: @TODO: this needs to be loaded at run-time
    //std::unique_ptr<External> external = std::unique_ptr<External>(new External(info, data_dir));
    //external->registerComponents();
    //info.set_external(std::move(external));

	LPT_INFO( "main", "[components::generate] Loading problem...");
	return fs0::Loader::loadProblem(data);
}

namespace fs0 { namespace drivers {

class SingletonLock {
    PythonRunner& _runner;

public:
    SingletonLock( PythonRunner& r )
        : _runner(r) {
            lapkt::tools::Logger::set_instance( std::move(_runner._logger));
			LogicalComponentRegistry::set_instance( std::move( _runner._registry ));
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
		_runner._registry = LogicalComponentRegistry::claim_ownership();
        _runner._logger = lapkt::tools::Logger::claim_ownership();
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
    _state_model(nullptr),
	_external_dll_handle(nullptr) {

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
	_external_dll_handle = nullptr;
}

PythonRunner::~PythonRunner() {
	std::cout << "[PythonRunner::Destructor] destroying external symbols" << std::endl;
	if ( _external_dll_handle == nullptr )
		return;

	ExternalI* ex = _problem_info->release_external();
	_external_destructor(ex);
	dlclose(_external_dll_handle);
	std::cout << "[PythonRunner::Destructor] all done!" << std::endl;
}

void
PythonRunner::report_stats(const Problem& problem, const std::string& out_dir) {
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

	LPT_INFO("main", "Number of objects: " << info.num_objects());
	LPT_INFO("main", "Number of state variables: " << info.getNumVariables());
	LPT_INFO("main", "Number of problem atoms: " << tuple_index.size());
	LPT_INFO("main", "Number of action schemata: " << problem.getActionData().size());
	LPT_INFO("main", "Number of (perhaps partially) ground actions: " << n_actions);
	LPT_INFO("main", "Number of goal atoms: " << num_goal_atoms);
	LPT_INFO("main", "Number of state constraint atoms: " << num_sc_atoms);


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
PythonRunner::load_external_symbols( ProblemInfo& info ) {
	if (_external_dll_name.empty()) return;
	if ( _external_dll_handle != nullptr )
		throw std::runtime_error("[PythonRunner::load_external_symbols] : Already associated with an external dll!");
	_external_dll_handle = dlopen( _external_dll_name.c_str(), RTLD_LAZY );

	if ( _external_dll_handle == nullptr )
		throw std::runtime_error("[PythonRunner::load_external_symbols] : Could not open external dll: " + _external_dll_name );
	// MRJ: see discussion on https://stackoverflow.com/questions/4770968/storing-function-pointer-in-stdfunction

	dlerror(); // Clear error state
	ExternalCreatorSignature func_ptr;
	*reinterpret_cast<void**>(&func_ptr) = dlsym(_external_dll_handle, "create_instance");
	//func_ptr = reinterpret_cast<ExternalCreatorSignature>(dlsym(_external_dll_handle, "create_instance"));
	_external_creator = func_ptr;
	const char *dlsym_error = dlerror();
	if (dlsym_error != nullptr) {
		dlclose(_external_dll_handle);
		throw std::runtime_error("[PythonRunner::load_external_symbols] : Cannot load symbol 'create_instance' " + std::string(dlsym_error));
	}
	dlerror(); // Clear error state
	ExternalDestructorSignature des_func_ptr;
	*reinterpret_cast<void**>(&des_func_ptr) = dlsym(_external_dll_handle, "destroy_instance");
	//func_ptr = reinterpret_cast<ExternalCreatorSignature>(dlsym(_external_dll_handle, "create_instance"));
	_external_destructor = des_func_ptr;
	dlsym_error = dlerror();
	if (dlsym_error != nullptr) {
		dlclose(_external_dll_handle);
		throw std::runtime_error("[PythonRunner::load_external_symbols] : Cannot load symbol 'destroy_instance' " + std::string(dlsym_error));
	}
	// and finally we're ready
	std::unique_ptr<ExternalI> external = std::unique_ptr<ExternalI>(_external_creator(info, _options.getDataDir()));
	LPT_INFO("main", "[PythonRunner::load_external_symbols] : Registering external components from library '"<< _external_dll_name << "'");
	external->registerComponents();
	info.set_external(std::move(external));

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
    if (_current_driver != nullptr )
        throw std::runtime_error("[PythonRunner::setup] was called twice on the same object" ) ;
    /* code */
    float t0 = aptk::time_used();

    _logger = std::make_unique<lapkt::tools::Logger>(_options.getOutputDir() + "/logs");
    lapkt::tools::Logger::set_instance(std::move(_logger));
    //lapkt::tools::Logger::init(_options.getOutputDir() + "/logs");

    // MRJ: The following two lines make up for the method Config::init()
    _instance_config = std::unique_ptr<Config>(new Config(_options.getDriver(), _options.getUserOptions(), _options.getDefaultConfigurationFilename()));
    Config::setAsGlobal( std::move(_instance_config) );

	fs0::LogicalComponentRegistry::set_instance( std::make_unique<fs0::LogicalComponentRegistry>());

    LPT_INFO("main", "[PythonRunner::setup] Generating the problem (" << _options.getDataDir() << ")... ");
    //! This will generate the problem and set it as the global singleton instance
    const std::string problem_spec = _options.getDataDir() + "/problem.json";
    auto data = Loader::loadJSONObject( problem_spec);
    LPT_INFO("main", "[PythonRunner::setup] Loaded JSON specification from '" << problem_spec << "'... ");

    fs0::BaseComponentFactory factory;

    LPT_INFO( "main", "[PythonRunner::setup] Loading language info...")
    fs0::fstrips::LanguageJsonLoader::loadLanguageInfo(data);

    LPT_INFO( "main", "[PythonRunner::setup] Loading problem info...")
    auto& info = fs0::Loader::loadProblemInfo(data, _options.getDataDir(), factory);

	//MRJ: placement of this function matters - depends on ProblemInfo being setup
	load_external_symbols(info);

    LPT_INFO( "main", "[PythonRunner::setup] Loading problem...");
    auto problem = fs0::Loader::loadProblem(data);

    LPT_INFO("main", "[PythonRunner::setup] Activated problem model... ");
    Config& config = Config::instance();

    LPT_INFO("main", "[PythonRunner::setup] Problem instance loaded" );
    report_stats( *problem, _options.getOutputDir() );
    update( config );

    LPT_INFO("main", "[PythonRunner::setup] Grounding Actions....");
    _problem = Problem::claimOwnership();
    _state_model = std::make_shared<SimpleStateModel>(drivers::GroundingSetup::fully_ground_simple_model(*_problem));
	Problem::setInstance(std::move(_problem));
    LPT_INFO("main", "[PythonRunner::setup] Indexing state variables..." );
    index_state_variables();
    std::string option_value = Config::instance().getOption<bool>("dynamics.decompose_ode", false) ? "yes" : "no";
    LPT_INFO( "main", "[PythonRunner::setup] Decomposing ODEs?: " << option_value);
    LPT_INFO("main", "[PythonRunner::setup] Preparing Search Engine....");
    _current_driver = _available_engines.get(_options.getDriver());
    _current_driver->prepare(*_state_model, config, _options.getOutputDir());
    _setup_time = aptk::time_used() - t0;
    LPT_INFO("main", "[PythonRunner::setup] Finished!" );
    // Singleton management: note that we're not using the Lock class because
    // the pointers are initialised during this method
    _lang_info = fstrips::LanguageInfo::claimOwnership();
    _problem_info = ProblemInfo::claimOwnership();
	_problem = Problem::claimOwnership();
    _instance_config = Config::claimOwnership();
    _logger = lapkt::tools::Logger::claim_ownership();
	_registry = LogicalComponentRegistry::claim_ownership();
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

bp::dict
PythonRunner::get_initial_state() {
    if ( _problem == nullptr ) {
        throw std::runtime_error("[PythonRunner::get_initial_state] Error: before setting states it is necessary to setup the planner");
    }
    SingletonLock lock(*this);
    if ( _state == nullptr )
        throw std::runtime_error("[PythonRunner::get_initial_state] Error: No initial state was set");
    const ProblemInfo& info = ProblemInfo::getInstance();
    return decode_state( *_state, info );
}

void
PythonRunner::set_null_plan() {
	_problem->setInitialState( *_state );
	SingletonLock lock(*this);
	float t0 = aptk::time_used();
	_current_driver->search();
	std::vector<const fs0::GroundAction*> empty;
	_native_plan.interpret_plan( empty );
	_plan = bp::list();
	_search_time = aptk::time_used() - t0;
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
    /*ExitCode code =*/ _current_driver->search();
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

bp::dict
PythonRunner::decode_state( const State& s, const ProblemInfo& info ) {
    bp::dict py_s;
    for ( unsigned x = 0; x < info.getNumVariables(); x++ ) {
        object_id value = s.getValue(x);
        type_id var_type = info.sv_type(x);

        if (var_type == type_id::bool_t) {
            py_s[info.getVariableName(x)] = fs0::value<bool>(value);
        } else if (var_type == type_id::float_t) {
            py_s[info.getVariableName(x)] = fs0::value<float>(value);
        }
        else if (var_type == type_id::int_t) {
            py_s[info.getVariableName(x)] = fs0::value<int>(value);
        }
        else if (var_type == type_id::object_t) {
            py_s[info.getVariableName(x)] = info.object_name(value);
        }
    }
    return py_s;
}

bp::list
PythonRunner::simulate_plan( double duration, double step_size ) {
    SingletonLock lock(*this);
    float t0 = aptk::time_used();


    double sim_duration = duration;//std::min( duration, (double)_native_plan.get_duration() );
    LPT_INFO( "main", "Simulating plan for " << sim_duration << " time units");
    LPT_INFO( "main", "Simulating with dt = " << step_size << " time units");

    _native_plan.simulate( step_size, sim_duration );
    LPT_INFO( "main", "Final simulation state: " << *_native_plan.trajectory().back() );

    bp::list py_trace;
    const ProblemInfo& info = ProblemInfo::getInstance();

    for ( unsigned k = 0; k < _native_plan.trajectory().size(); k++ ) {
        auto s_k = _native_plan.trajectory()[k];
        bp::dict py_s_k = decode_state( *s_k, info );
        py_trace.append(py_s_k);
    }
    _simulation_time = aptk::time_used() - t0;
    return py_trace;
}

}}
