# Cordeau DARP → Multi-Depot JSON Converter

Small utility to convert **Cordeau DARP benchmark instances** into my **Multi-Depot JSON format**. This tool converts instances from the [Cordeau DARP benchmark](http://neumann.hec.ca/chairedistributique/data/darp/).

## What it Does

* Reads standard Cordeau `.txt` DARP instances
* Builds pickup, delivery, and per-vehicle depot nodes
* Generates:

  * Request definitions
  * Vehicle definitions
  * Node list
  * Travel time matrix (`matrix_t`)
  * Vehicle-specific cost matrix (`matrix_c`)
* Outputs a structured `.json` file

## Requirements

* Python 3.9+
* No external dependencies (standard library only)

## How to Run

```bash
python main.py --load instances/a2-16.txt --output instances/a2-16.json
```

### Arguments

* `--load` : Path to the Cordeau instance file
* `--output` : Path to the generated JSON file

## Notes

* Distances are Euclidean and rounded to 3 decimals
* Each vehicle gets its own start and end depot nodes
* Arc generation follows the strict ( $A_k$ ) definition (no self-loops, no entering sources, no leaving sinks). Where $P$ is the set of pickup nodes, $D$ is the set of delivery nodes, and $s_k$, $e_k$ are the start and end depot nodes for vehicle $k.

$$
  A_k = \left\{ (i,j)\; \middle|\;
  \begin{array}{l}
  k \in K,\\
  i,j \in P \cup D \cup \{s_k,e_k\},\\
  i \neq j, \, i \neq e_k, \, j \neq s_k
  \end{array}
  \right\}
$$
