#include "DARPMD_ProblemInstance.hh"

int main() {
    DARPMD_ProblemInstance instance;
    if (!instance.loadFromJSON("./generated_instance.json")) {
        return 1; // Error loading instance
    }
    instance.displayInfo();
    return 0;
}