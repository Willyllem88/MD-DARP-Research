#!/bin/bash

# 1. Leer los parámetros fijos que siempre envía irace
CONFIG_ID=$1
INSTANCE_ID=$2
SEED=$3
INSTANCE=$4

# Desplazar los 4 primeros argumentos para procesar solo los de ALNS
shift 4

# 2. Inicializar variables
MAX_ITER=""
COOLING=""
DESTROY=""
W_PARAM=""

# 3. Leer los parámetros generados por irace
while [[ "$#" -gt 0 ]]; do
    case $1 in
        --maxIterations) MAX_ITER="$2"; shift 2 ;;
        --coolingRate) COOLING="$2"; shift 2 ;;
        --destroyFraction) DESTROY="$2"; shift 2 ;;
        --w) W_PARAM="$2"; shift 2 ;;
        *) shift ;; # Ignorar argumentos desconocidos por seguridad
    esac
done

# 4. Ejecutar solver pasando todos los parámetros seguidos a --alnsParams
/home/guillem/TFG-Guillem/build/cpp/core/darpmd_run \
    -i "$INSTANCE" \
    -m ALNS \
    -s "$SEED" \
    --alnsParams "$MAX_ITER" "$COOLING" "$DESTROY" "$W_PARAM"

# 5. Leer resultado del JSON con jq
COST=$(jq '.summary.objective' /home/guillem/TFG-Guillem/tuning/solution_report.json)

# 6. Imprimir SOLO el coste para irace
echo "$COST"