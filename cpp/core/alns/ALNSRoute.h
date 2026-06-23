#pragma once

#include <vector> 
#include <map>

// Internal representation of a single vehicle route
struct ALNSRoute {
    int vehicleId;
    std::vector<int> sequence; // Sequence of Node IDs (Depot -> ... -> Depot)
    
    // Evaluation metrics
    double distanceCost = 0.0;
    double timeWindowViolation = 0.0;
    double vehicleMaxRouteTimeViolation = 0.0;
    double loadViolation = 0.0;
    double rideTimeViolation = 0.0;
    double totalCost = 0.0;
    bool isFeasible = false;

    // Vectors indexed by node ID (not by position in the sequence)
    std::vector<double> loads;
    std::vector<double> A;   // Arrival times
    std::vector<double> W;   // Waiting times
    std::vector<double> B;   // Beginning of service times
    std::vector<double> D;   // Departure times
    std::vector<int> id2pos; // Map from Node ID to position in the sequence for quick access

    inline void initializeNodeArrays(int maxNodeId) {
        int size = maxNodeId + 1;
        loads.assign(size, 0.0);
        A.assign(size, -1.0);
        W.assign(size, 0.0);
        B.assign(size, -1.0);
        D.assign(size, -1.0);
        id2pos.assign(size, -1);
    }

    // Get node metrics by node ID
    inline double getA(int nodeId) const { return A[nodeId]; }
    inline double getB(int nodeId) const { return B[nodeId]; }
    inline double getW(int nodeId) const { return W[nodeId]; }
    inline double getD(int nodeId) const { return D[nodeId]; }
    inline double getLoad(int nodeId) const { return loads[nodeId]; }

    // Get node metrics by position in the sequence
    inline double getAByPos(int pos) const { return A[sequence[pos]]; }
    inline double getBByPos(int pos) const { return B[sequence[pos]]; }
    inline double getWByPos(int pos) const { return W[sequence[pos]]; }
    inline double getDByPos(int pos) const { return D[sequence[pos]]; }
    inline double getLoadByPos(int pos) const { return loads[sequence[pos]]; }

    // Set node metrics by node ID
    inline void setA(int nodeId, double value) { A[nodeId] = value; }
    inline void setB(int nodeId, double value) { B[nodeId] = value; }
    inline void setW(int nodeId, double value) { W[nodeId] = value; }
    inline void setD(int nodeId, double value) { D[nodeId] = value; }
    inline void setLoad(int nodeId, double value) { loads[nodeId] = value; }

    // Set node metrics by position in the sequence
    inline void setAByPos(int pos, double value) { A[sequence[pos]] = value; }
    inline void setBByPos(int pos, double value) { B[sequence[pos]] = value; }
    inline void setWByPos(int pos, double value) { W[sequence[pos]] = value; }
    inline void setDByPos(int pos, double value) { D[sequence[pos]] = value; }
    inline void setLoadByPos(int pos, double value) { loads[sequence[pos]] = value; }

    // Utility functions
    int getPosition(int nodeId) const { return id2pos[nodeId]; }
    int getNodeAtPosition(int pos) const { return sequence[pos]; }
    int getRouteSize() const { return sequence.size(); }
    bool containsNode(int nodeId) const { return id2pos[nodeId] != -1; }
};

struct RouteSequenceHash {
    std::size_t operator()(const std::vector<int>& seq) const {
        std::size_t hash = 0;
        for (int nodeId : seq) {
            hash ^= std::hash<int>{}(nodeId) + 0x9e3779b9 + (hash << 6) + (hash >> 2);
        }
        return hash;
    }
};