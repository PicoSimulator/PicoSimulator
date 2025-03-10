#pragma once

#include <nlohmann/json.hpp>

typedef bool (*schema_validate_fn)(const json &json);

bool schema_validate_config(const json &json);
bool schema_validate_env(const json &json);
