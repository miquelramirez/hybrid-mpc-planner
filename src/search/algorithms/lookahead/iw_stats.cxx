#include <search/algorithms/lookahead/iw_stats.hxx>
#include <ios>
#include <iomanip>
#include <sstream>

namespace fs0 { namespace lookahead {

    std::string
    IteratedWidthStats::_if_computed(unsigned val) {
    	return val < std::numeric_limits<unsigned>::max() ?  std::to_string(val) : "N/A";
    }

    std::string
    IteratedWidthStats::_avg(unsigned val, unsigned den) {
    	if (den == 0) return "N/A";
    	std::stringstream ss;
    	ss << std::fixed << std::setprecision(2) << val / (float) den;
    	return ss.str();
    }

    std::vector<IteratedWidthStats::DataPointT>
    IteratedWidthStats::dump() const {
    	std::vector<IteratedWidthStats::DataPointT> data = {
    		std::make_tuple("expanded", "Expansions", std::to_string(expanded())),
    		std::make_tuple("generated", "Generations", std::to_string(generated())),
    		std::make_tuple("evaluated", "Evaluations", std::to_string(evaluated())),

    		std::make_tuple("_num_w1_nodes", "w_{F}(n)=1", std::to_string(_num_w1_nodes)),
    		std::make_tuple("_num_w2_nodes", "w_{F}(n)=2", std::to_string(_num_w2_nodes)),
    		std::make_tuple("_num_wgt2_nodes", "w_{F}(n)>2", std::to_string(_num_wgt2_nodes)),

    		std::make_tuple("_num_expanded_g_decrease", "Expansions with #g decrease", std::to_string(_num_expanded_g_decrease)),
    		std::make_tuple("_num_generated_g_decrease", "Generations with #g decrease", std::to_string(_num_generated_g_decrease)),
            std::make_tuple("_initial_reward", "r(s0)", std::to_string(_initial_reward)),
            std::make_tuple("_max_reward", "max r(s)", std::to_string(_max_reward)),
            std::make_tuple("_max_depth", "max g(s)", std::to_string(_max_depth))
    	};
    	return data;
    }



} }
