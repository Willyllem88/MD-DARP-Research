#ifndef DARPMD_RESULTINSTANCE_H
#define DARPMD_RESULTINSTANCE_H

#include <vector>
#include <string>
#include <map>
#include <iostream>
#include <fstream>
#include <iomanip>

// Estructura para representar la visita a un nodo específico
struct RouteStep {
    int nodeId;             // ID del nodo (0, 1, ..., N)
    std::string type;       // "DepotStart", "DepotEnd", "Pickup", "Delivery"
    double arrivalTime;     // Valor de la variable u
    double startServiceTime;// Cuándo empieza realmente el servicio
    double loadAfter;       // Carga del vehículo tras visitar el nodo (variable w)
    double distanceTraveled;// Distancia acumulada hasta este punto (opcional)
};

// Estructura para representar la ruta completa de un vehículo
struct VehicleRoute {
    int vehicleId;
    double routeCost;       // Coste específico de esta ruta (si se calcula)
    double routeDuration;   // Tiempo total usado
    std::vector<RouteStep> steps; // Secuencia ordenada de visitas

    bool isEmpty() const { return steps.size() <= 2; } // Si solo tiene Start y End
};

// Clase principal para almacenar el resultado global
class DARPMD_ResultInstance {
public:
    // --- Metadatos de la Solución ---
    double objectiveValue;
    std::string solverStatus; // "Optimal", "Feasible", "Infeasible"
    double solutionTimeSec;   // Tiempo de cómputo

    // --- Datos de las Rutas ---
    // Mapa: ID del Vehículo -> Objeto Ruta
    std::map<int, VehicleRoute> routes;

    // --- Constructor ---
    DARPMD_ResultInstance();

    // --- Métodos de Gestión ---
    void addRoute(int vehicleId, const VehicleRoute& route);

    // --- Métodos de Visualización ---
    void displaySummary() const;
    
    // --- Métodos de Exportación ---
    
    // Guarda en un formato JSON simple (ideal para parsers externos)
    void saveToJSON(const std::string& filename) const;

    // Guarda en un formato de texto legible (tipo reporte)
    void saveToTxt(const std::string& filename) const;
};

#endif // DARPMD_RESULTINSTANCE_H

// TODO list
// - agregar metricas de tiempo de servicio, pintar mejor
// - traducir la doc al ingles