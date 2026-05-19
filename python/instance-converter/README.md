# Cordeau DARP → Multi-Depot JSON Converter

Small utility to convert **Cordeau DARP benchmark instances** into my **Multi-Depot JSON format**. This tool converts instances from the [Cordeau DARP benchmark](http://neumann.hec.ca/chairedistributique/data/darp/).

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
