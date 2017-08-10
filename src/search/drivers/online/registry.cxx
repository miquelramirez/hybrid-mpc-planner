
#include <problem.hxx>
#include <search/drivers/online/registry.hxx>
#include <search/drivers/online/iterated_width.hxx>
#include <search/drivers/breadth_first_search.hxx>
#include <search/drivers/sbfws/sbfws.hxx>
#include <actions/grounding.hxx>
#include <problem_info.hxx>
#include <models/simple_state_model.hxx>


// using namespace fs0::gecode;

namespace fs0 { namespace drivers { namespace online {



EngineRegistry& EngineRegistry::instance() {
	static EngineRegistry theInstance;
	return theInstance;
}

EngineRegistry::EngineRegistry() {
	// We register the pre-configured search drivers on the instantiation of the singleton
	add("iw",  new IteratedWidthDriver());
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
