
#include <components.hxx>
#include <utils/loader.hxx>
#include <utils/component_factory.hxx>
#include <fstrips/loader.hxx>
#include <lapkt/tools/logging.hxx>

fs0::Problem* generate(const rapidjson::Document& data, const std::string& data_dir) {
	fs0::BaseComponentFactory factory;
	LPT_INFO( "main", "[components::generate] Loading language info...")
	fs0::fstrips::LanguageJsonLoader::loadLanguageInfo(data);
	LPT_INFO( "main", "[components::generate] Loading problem info...")
	fs0::Loader::loadProblemInfo(data, data_dir, factory);
	LPT_INFO( "main", "[components::generate] Loading problem...");
	return fs0::Loader::loadProblem(data);
}
