# --- CPLEX ROUTES ---
CPLEX_ROOT = /opt/ibm/ILOG/CPLEX_Studio2211
CPLEX_LIB  = $(CPLEX_ROOT)/cplex/lib/x86-64_linux/static_pic
CONCERT_LIB = $(CPLEX_ROOT)/concert/lib/x86-64_linux/static_pic

# --- COMPILER CONFIG ---
CXX      = g++
CXXFLAGS = -O3 -m64 -fPIC -fno-strict-aliasing -fexceptions -DNDEBUG -DIL_STD
INCLUDES = -I$(CPLEX_ROOT)/cplex/include -I$(CPLEX_ROOT)/concert/include
LDFLAGS  = -L$(CPLEX_LIB) -L$(CONCERT_LIB) -lilocplex -lcplex -lconcert -lm -lpthread -ldl

# --- FILES ---
TARGET = tsp_mtz
SRC    = tsp_mtz.cpp

# --- RULES ---
all: $(TARGET)

$(TARGET): $(SRC)
	$(CXX) $(CXXFLAGS) $(INCLUDES) -o $(TARGET) $(SRC) $(LDFLAGS)

clean:
	rm -f $(TARGET)