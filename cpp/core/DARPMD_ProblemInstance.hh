#pragma once

#include <vector>
#include <string>
#include <map>
#include <tuple>
#include <utility>

// We use the nlohmann/json library for JSON parsing
// https://github.com/nlohmann/json
#include "json.hpp" 
using json = nlohmann::json;

class DARPMD_ProblemInstance {
public:
    DARPMD_ProblemInstance() = default;
    // Carga desde el JSON generado por tu Python
    bool loadFromJSON(const std::string& filename);
    
    void displayInfo() const;

    // Getters eficientes (ejemplo)
    double getTravelTime(int i, int j) const;
    double getCost(int i, int j, int k) const;

    // Datos públicos (o privados con getters, según prefieras)
    // Usamos double para precisión
    int N_requests; // Número de peticiones
    int max_node_id; // Para dimensionar vectores

    // Sets
    //TODO: use std::unordered_set if order is not important
    std::vector<int> P; // Pickups
    std::vector<int> D; // Deliveries
    std::vector<int> K; // Vehicles
    
    // Mappings de vehículos
    //TODO: use std::unordered_set if order is not important
    std::map<int, int> StartNode; // k -> start_node
    std::map<int, int> EndNode;   // k -> end_node
    
    // Parámetros por nodo (indexados por ID de nodo para acceso O(1))
    // Asumimos que los IDs de nodo son enteros razonables (0 a 1000, no millones)
    std::vector<double> service_time; // d_i
    std::vector<double> demand;       // q_i
    std::vector<double> time_window_start; // e_i
    std::vector<double> time_window_end;   // l_i

    // Parámetros por vehículo
    std::map<int, double> capacity; // Q_k
    std::map<int, double> max_route_time; // T_k

    // Globales
    double max_ride_time; // L

    // Matrices: Usamos vectores planos o 2D para velocidad.
    // Matriz de tiempos (t_ij): vector de vectores
    std::vector<std::vector<double>> t_ij;

    // Costos (c_ijk): Dado que depende de K, y K puede no ser secuencial, 
    // podemos usar un mapa o un vector 3D si re-indexamos. 
    // Para simplificar la migración manteniendo velocidad:
    // map<{i,j}, base_cost> y factores por vehículo, o map completo.
    // Aquí mantendré tu lógica pero optimizada:
    std::map<std::tuple<int, int, int>, double> c_ijk;
};
