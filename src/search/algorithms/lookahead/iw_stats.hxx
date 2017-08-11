#pragma once

#include <limits>

#include <tuple>
#include <vector>


namespace fs0 { namespace lookahead {

    class IteratedWidthStats {
    public:
    	IteratedWidthStats() = default;

    	void expansion() { ++_expanded; }
    	void generation() { ++_generated; }

        void w1_node() { ++_num_w1_nodes; }
        void w2_node() { ++_num_w2_nodes; }
        void wgt2_node() { ++_num_wgt2_nodes; }

    	void expansion_g_decrease() { ++_num_expanded_g_decrease; }
    	void generation_g_decrease() { ++_num_generated_g_decrease; }

    	unsigned long num_w1_nodes() const { return _num_w1_nodes; }
    	unsigned long num_w2_nodes() const { return _num_w2_nodes; }
    	unsigned long num_wgt2_nodes() const { return _num_wgt2_nodes; }

    	unsigned long expanded() const { return _expanded; }
        unsigned long evaluated() const { return _expanded; }
    	unsigned long generated() const { return _generated; }


    	void set_initial_reward(float r) { _initial_reward = r; }
    	void reward(float r) {
    		_max_reward = std::max(r, _max_reward);
    	}

        float initial_reward() const { return _initial_reward; }
        float max_reward() const { return _max_reward; }

        void reset() {
            _expanded = 0;
        	_generated = 0;

        	_num_w1_nodes = 0; // The number of nodes with w_{F} = 1 that have been processed.
        	_num_w2_nodes = 0; // The number of nodes with w_{F} = 2 that have been processed.
            _num_wgt2_nodes = 0; // The number of nodes with w_{F} > 2 that have been processed.
        	_num_expanded_g_decrease = 0; // The number of nodes with a decrease in #g that are expanded
        	_num_generated_g_decrease = 0; // The number of nodes with a decrease in #g that are expanded

            _initial_reward = 0.0f;
            _max_reward = std::numeric_limits<float>::min();
        }

        using DataPointT = std::tuple<std::string, std::string, std::string>;
    	std::vector<DataPointT> dump() const;

    protected:

    	static std::string _if_computed(unsigned val);
    	static std::string _avg(unsigned val, unsigned den);
        
    	unsigned long _expanded = 0;
    	unsigned long _generated = 0;

    	unsigned long _num_w1_nodes = 0; // The number of nodes with w_{F} = 1 that have been processed.
    	unsigned long _num_w2_nodes = 0; // The number of nodes with w_{F} = 2 that have been processed.
        unsigned long _num_wgt2_nodes = 0; // The number of nodes with w_{F} > 2 that have been processed.
    	unsigned long _num_expanded_g_decrease = 0; // The number of nodes with a decrease in #g that are expanded
    	unsigned long _num_generated_g_decrease = 0; // The number of nodes with a decrease in #g that are expanded

        float   _initial_reward = 0.0f;
        float   _max_reward = std::numeric_limits<float>::min();
    };


}}
