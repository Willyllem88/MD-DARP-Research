#pragma once
#include <iostream>
#include <string>

class Logger {
public:
    explicit Logger(bool verbose = false) : verbose(verbose) {}

    void setVerbose(bool v) { verbose = v; }

    void log(const std::string& msg) const {
        if (verbose) {
            std::cout << msg << std::endl;
        }
    }

private:
    bool verbose;
};