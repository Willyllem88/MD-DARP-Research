#pragma once

#include "ALNSRoute.h"
#include <vector>
#include <map>
#include <unordered_map>
#include <unordered_set>

class RoutePool {
public:
    RoutePool() = default;
    ~RoutePool() = default;

    // Añade la ruta si no está duplicada
    void addRoute(const ALNSRoute& route);

    // Obtiene todas las rutas (útil para que el solver las lea)
    const std::map<int, std::vector<ALNSRoute>>& getRoutes() const { return routePool; }

    // Firma de la función de purga.
    // Podría recibir parámetros como límite de tamaño, umbral de coste, o iteraciones sin mejora.
    void purgeColumns();

    // Limpia todo el pool
    void clear();

private:
    // Key: VehicleID, Value: List of routes
    std::map<int, std::vector<ALNSRoute>> routePool;
    
    // Para evitar duplicados eficientemente
    std::unordered_map<int, std::unordered_set<std::vector<int>, RouteSequenceHash>> seenRoutes;
};