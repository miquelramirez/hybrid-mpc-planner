
#include <fs/core/problem.hxx>
#include <fs/core/search/drivers/breadth_first_search.hxx>
#include <fs/core/search/drivers/sbfws/sbfws.hxx>
#include <fs/core/actions/grounding.hxx>
#include <fs/core/problem_info.hxx>
#include <fs/core/models/simple_state_model.hxx>

#include <search/drivers/online/registry.hxx>
#include <search/drivers/online/iterated_width.hxx>
#include <search/drivers/online/sim_bfws.hxx>
// using namespace fs0::gecode;

namespace fs0 { namespace drivers { namespace online {

EngineRegistry::EngineRegistry() {
	// We register the pre-configured search drivers on the instantiation of the singleton
	add("iw",  new IteratedWidthDriver());
	add("sbfws",  new SimBFWSDriver());
}

EngineRegistry::~EngineRegistry() {
	for (const auto elem:_creators) delete elem.second;
}

void EngineRegistry::add(const std::string& engine_name, EmbeddedDriver* creator) {
auto res = _creators.insert(std::make_pair(engine_name, creator));
	if (!res.second) throw new std::runtime_error("Duplicate registration of engine creator for symbol " + engine_name);
}


EmbeddedDriver* EngineRegistry::get(const std::string& engine_name) {
	auto it = _creators.find(engine_name);
	if (it == _creators.end()) throw std::runtime_error("No engine creator has been registered for given engine name '" + engine_name + "'");
	return it->second;
}


} } }// namespaces
