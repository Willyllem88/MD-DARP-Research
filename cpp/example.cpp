#include <ilcplex/ilocplex.h>
#include <vector>
#include <iostream>

// Macro estándar para usar el namespace de CPLEX sin escribir 'Ilo' todo el rato en clases base
ILOSTLBEGIN

int main() {
    // 1. GESTIÓN DEL ENTORNO
    // IloEnv gestiona la memoria y el ciclo de vida de los objetos de CPLEX
    IloEnv env;

    try {
        // ---------------------------------------------------------
        // 2. DEFINICIÓN DE DATOS (Matriz de distancias asimétrica o simétrica)
        // ---------------------------------------------------------
        const int n = 5; // Número de ciudades (0 a 4)
        
        // Ejemplo de matriz de distancias (dummy)
        // Diagonal es 0, valores arbitrarios para el ejemplo
        double distMatrix[n][n] = {
            {0,  10, 15, 20, 10},
            {10, 0,  35, 25, 10},
            {15, 35, 0,  30, 10},
            {20, 25, 30, 0,  10},
            {10, 10, 10, 10, 0}
        };

        // ---------------------------------------------------------
        // 3. CREACIÓN DEL MODELO Y VARIABLES
        // ---------------------------------------------------------
        IloModel model(env);

        // Variables de decisión x[i][j]: 1 si vamos de i a j, 0 si no
        IloArray<IloNumVarArray> x(env, n);
        for (int i = 0; i < n; i++) {
            x[i] = IloNumVarArray(env, n, 0, 1, ILOBOOL);
            // Opcional: poner nombres a variables para debugging (consume memoria)
            /*
            for(int j=0; j<n; j++){
                char name[20];
                sprintf(name, "x_%d_%d", i, j);
                x[i][j].setName(name);
            }
            */
        }

        // Variables auxiliares u[i] para MTZ (orden de visita)
        // u[i] indica la posición de la ciudad i en el tour.
        // Rango: 1..n para nodos 1..n-1 (el nodo 0 es fijo)
        IloNumVarArray u(env, n, 0, n, ILOFLOAT);

        // ---------------------------------------------------------
        // 4. FUNCIÓN OBJETIVO
        // ---------------------------------------------------------
        IloExpr objExpr(env);
        for (int i = 0; i < n; i++) {
            for (int j = 0; j < n; j++) {
                if (i != j) {
                    objExpr += distMatrix[i][j] * x[i][j];
                }
            }
        }
        model.add(IloMinimize(env, objExpr));
        objExpr.end(); // Liberar memoria de la expresión una vez añadida

        // ---------------------------------------------------------
        // 5. RESTRICCIONES
        // ---------------------------------------------------------

        // Restricción 1: Salir de cada ciudad exactamente una vez
        for (int i = 0; i < n; i++) {
            IloExpr rowExpr(env);
            for (int j = 0; j < n; j++) {
                if (i != j) rowExpr += x[i][j];
            }
            model.add(rowExpr == 1);
            rowExpr.end();
        }

        // Restricción 2: Entrar a cada ciudad exactamente una vez
        for (int j = 0; j < n; j++) {
            IloExpr colExpr(env);
            for (int i = 0; i < n; i++) {
                if (i != j) colExpr += x[i][j];
            }
            model.add(colExpr == 1);
            colExpr.end();
        }

        // Restricción 3: Eliminación de Subtours (MTZ)
        // u_i - u_j + n * x_ij <= n - 1  para i, j != 0
        for (int i = 1; i < n; i++) {
            for (int j = 1; j < n; j++) {
                if (i != j) {
                    model.add(u[i] - u[j] + n * x[i][j] <= n - 1);
                }
            }
        }

        // ---------------------------------------------------------
        // 6. RESOLUCIÓN
        // ---------------------------------------------------------
        IloCplex cplex(model);
        
        // Opcional: Desactivar output del log de cplex en consola
        // cplex.setOut(env.getNullStream());

        if (cplex.solve()) {
            std::cout << "--------------------------------------------" << std::endl;
            std::cout << "Estado de la solucion: " << cplex.getStatus() << std::endl;
            std::cout << "Valor Objetivo (Distancia Minima): " << cplex.getObjValue() << std::endl;
            std::cout << "--------------------------------------------" << std::endl;

            // ---------------------------------------------------------
            // 7. IMPRIMIR RESULTADOS
            // ---------------------------------------------------------
            std::cout << "Ruta encontrada (aristas activas):" << std::endl;
            for (int i = 0; i < n; i++) {
                for (int j = 0; j < n; j++) {
                    // Verificamos si la variable es > 0.5 (tolerancia numérica)
                    // cplex.getValue(var) obtiene el valor de la solución
                    if (i != j && cplex.getValue(x[i][j]) > 0.5) {
                        std::cout << "Ciudad " << i << " -> Ciudad " << j 
                                  << " (u[" << i << "]=" << cplex.getValue(u[i]) 
                                  << ", u[" << j << "]=" << cplex.getValue(u[j]) << ")" 
                                  << std::endl;
                    }
                }
            }
        } else {
            std::cout << "No se encontro solucion factible." << std::endl;
        }
    }
    catch (IloException& e) {
        std::cerr << "Excepcion de Concert: " << e << std::endl;
    }
    catch (...) {
        std::cerr << "Error desconocido" << std::endl;
    }

    // 8. LIMPIEZA FINAL
    env.end();
    return 0;
}