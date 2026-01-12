# Makefile básico para tu proyecto DARP

CXX = g++
CXXFLAGS = -std=c++17 -Wall -O2
TARGET = darp
SOURCES = main.cpp DARPMD_ProblemInstance.cpp
HEADERS = DARPMD_ProblemInstance.hh json.hpp

# Reglas
all: $(TARGET)

$(TARGET): $(SOURCES) $(HEADERS)
	$(CXX) $(CXXFLAGS) $(SOURCES) -o $(TARGET)

clean:
	rm -f $(TARGET)
