#pragma once
#include <string>
#include <map>
#include <vector>

#include "../includes/json.hpp"

struct Metadata {
    std::string city;
    std::map<std::string, std::vector<double>> coordinates;

    bool isEmpty() const {
        return city.empty() && coordinates.empty(); 
    }
};

// This mactro defines to_json and from_json functions for Metadata
NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE(Metadata, city, coordinates)