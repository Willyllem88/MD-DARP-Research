#pragma once

#include "../core/DARPMD_ProblemInstance.h"
#include <iostream>
#include <vector>
#include <string>

/**
 * @class InstanceAnalyzer
 * @brief Analyzes a DARPMD problem instance and reports statistical metrics.
 * * References for parameter definitions:
 * - Cordeau, J. F., & Laporte, G. (2003). A tabu search heuristic for the static 
 * multi-vehicle dial-a-ride problem. Transportation Research Part B: Methodological.
 * - Ropke, S., & Pisinger, D. (2006). An adaptive large neighborhood search 
 * heuristic for the pickup and delivery problem with time windows.
 */
class InstanceAnalyzer {
public:
    explicit InstanceAnalyzer(const DARPMD_ProblemInstance& instance);

    // Main method to print the report to console
    void printReport() const;

private:
    const DARPMD_ProblemInstance& data;

    // Helper structs for stats
    struct StatResult {
        double min;
        double max;
        double avg;
        double sum;
    };

    // Helper calculation methods
    StatResult calculateTimeWindowStats() const;
    StatResult calculateDemandStats() const;
    StatResult calculateRideTimeStats() const;
    double calculateGlobalCapacityTightness() const;
    
    // Formatting helper
    void printHeader(const std::string& title) const;
    void printStatLine(const std::string& label, const StatResult& stat) const;
};