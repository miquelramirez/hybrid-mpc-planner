#pragma once

#include <cstring>
#include <lib/rapidjson/document.h>
#include <utils/loader.hxx>
#include <utils/component_factory.hxx>
#include <fstrips/loader.hxx>


namespace fs0 { class Problem; }

/* Generate the whole planning problem */
fs0::Problem* generate(const rapidjson::Document& data, const std::string& data_dir);
