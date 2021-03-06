#include <python_runner.hxx>
using namespace boost::python;
using namespace fs0::drivers;

BOOST_PYTHON_MODULE( libfs_planner )
{
    class_<PythonRunner>("HybridPlanner")
    .def( init<  >() )
    .def( "setup", &PythonRunner::setup )
    .def( "set_initial_state", &PythonRunner::set_initial_state )
    .def( "get_initial_state", &PythonRunner::get_initial_state )
    .def( "solve", &PythonRunner::solve )
    .def( "set_null_plan", &PythonRunner::set_null_plan)
    .def( "simulate_plan", &PythonRunner::simulate_plan)
    .def( "get_user_option", &PythonRunner::get_user_option )
    .def( "set_user_option", &PythonRunner::set_user_option )
    //! Read only properties
    .add_property( "plan", &PythonRunner::get_plan )
    .add_property( "plan_duration", &PythonRunner::get_plan_duration )
    .add_property( "search_time", &PythonRunner::get_search_time )
    .add_property( "setup_time", &PythonRunner::get_setup_time )
    .add_property( "simulation_time", &PythonRunner::get_simulation_time )
    .add_property( "result", &PythonRunner::get_result )
    //! Read write properties
    .add_property( "timeout", &PythonRunner::get_timeout, &PythonRunner::set_timeout)
    .add_property( "data_dir", &PythonRunner::get_data_dir, &PythonRunner::set_data_dir)
    .add_property( "config", &PythonRunner::get_config, &PythonRunner::set_config)
    .add_property( "output_dir", &PythonRunner::get_output_dir, &PythonRunner::set_output_dir)
    .add_property( "search_driver", &PythonRunner::get_search_driver, &PythonRunner::set_search_driver)
    .add_property( "delta_max", &PythonRunner::get_delta_max, &PythonRunner::set_delta_max)
    .add_property( "delta_min", &PythonRunner::get_delta_min, &PythonRunner::set_delta_min)
    .add_property( "horizon", &PythonRunner::get_horizon, &PythonRunner::set_horizon)
    .add_property( "budget", &PythonRunner::get_budget, &PythonRunner::set_budget)
    .add_property( "verify_plan", &PythonRunner::get_verify_plan, &PythonRunner::set_verify_plan)
    .add_property( "external_lib", &PythonRunner::get_external_lib, &PythonRunner::set_external_lib)

    ; //! Note the semi colon!
}
