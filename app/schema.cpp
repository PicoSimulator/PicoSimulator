#include "schema.hpp"

#include <iostream>
#include <string>

bool schema_validate_config(const json &json)
{
    if (!json.is_object()) {
        std::cerr << "Config top level must be an object\n";
        return false;
    }
    std::vector<std::string> known_keys = {
        "environments",
        "libraries"
    };
    for (auto &[key, _] : json.items()) {
        const auto it = std::find(known_keys.begin(), known_keys.end(), key);
        if (it == known_keys.end()) {
            std::cerr << "Unknown key \"" << key << "\" in \"config\"\n";
        }
    }
    {
        if (!json.contains("environments")) {
            std::cerr << "Config top level must contain \"environments\" array\n";
            return false;
        }
        auto &envs = json["environments"];
        if (!envs.is_array()) {
            std::cerr << "\"config.environments\" must be an array!\n";
            return false;
        }
        for (auto &env : json["environments"]) {

        }
    }
}