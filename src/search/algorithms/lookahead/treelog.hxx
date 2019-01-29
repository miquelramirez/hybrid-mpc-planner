#pragma once

#include <fs/core/problem_info.hxx>
#include <fs/core/utils/archive/json.hxx>

namespace fs0 { namespace lookahead {

    template <typename NodePT>
	void archive_node( rapidjson::Value& obj, rapidjson::Document::AllocatorType& allocator, NodePT node ) {
		using namespace rapidjson;
		JSONArchive::store(obj, allocator, node->state);
		Value v(node->_gen_order);
		obj.AddMember( "gen_order", v.Move(), allocator);
		v = Value(node->R);
		obj.AddMember( "reward", v.Move(), allocator);
	}

    template <typename Engine>
    void dump_search_tree( Engine& search_engine, std::string filename ) {
        using namespace rapidjson;

        const ProblemInfo& info = ProblemInfo::getInstance();
		Document trace;
		Document::AllocatorType& allocator = trace.GetAllocator();
		trace.SetObject();
		Value domainName;
		domainName.SetString(StringRef(info.getDomainName().c_str()));
		trace.AddMember("domain", domainName.Move(), allocator );
		Value instanceName;
		instanceName.SetString(StringRef(info.getInstanceName().c_str()));
		trace.AddMember("instance", instanceName.Move(), allocator );
		Value visits(kArrayType);
        {
            for ( auto n : search_engine._visited ) {
                Value state(kObjectType);
				archive_node( state, allocator, n );
				visits.PushBack(state.Move(), allocator);
            }
        }
        trace.AddMember("visited", visits, allocator);
		Value selected_path(kArrayType);
		{
			typename Engine::NodePT node = search_engine.get_best_node();
            if ( node != nullptr ) {
    			while (node->has_parent()) {
    				Value state(kObjectType);
    				archive_node( state, allocator, node );
    				selected_path.PushBack( state.Move(),allocator);
    				node = node->parent;
    			}

    			Value s0(kObjectType);
    			archive_node( s0, allocator, node );
    			selected_path.PushBack(s0.Move(),allocator);
            }
		}
		trace.AddMember("selected_path", selected_path, allocator );

		FILE* fp = fopen( filename.c_str(), "wb"); // non-Windows use "w"
		char writeBuffer[65536];
		FileWriteStream os(fp, writeBuffer, sizeof(writeBuffer));
		Writer<FileWriteStream> writer(os);
		trace.Accept(writer);
		fclose(fp);
    }

}}
