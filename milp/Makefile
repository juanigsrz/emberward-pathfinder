# Makefile for Integer Programming Solver
# Requires Gurobi to be installed and GUROBI_HOME set

CXX = g++
CXXFLAGS = -std=c++17 -O2 -Wall -Wextra

# Gurobi settings - adjust path as needed
# GUROBI_HOME = 
GUROBI_INC = $(GUROBI_HOME)/include
GUROBI_LIB = $(GUROBI_HOME)/lib

# Get Gurobi version dynamically
GUROBI_VERSION = 120

TARGET = integer_programming
SOURCES = integer_programming.cpp
HEADERS = integer_programming.hpp

INCLUDES = -I$(GUROBI_INC)
LIBS = -L$(GUROBI_LIB) -lgurobi_c++ -lgurobi$(GUROBI_VERSION) -lm -lpthread

$(TARGET): $(SOURCES) $(HEADERS)
	$(CXX) $(CXXFLAGS) $(INCLUDES) -o $(TARGET) $(SOURCES) $(LIBS)

clean:
	rm -f $(TARGET)

test: $(TARGET)
	@echo "Testing integer programming solver..."
	./$(TARGET)

.PHONY: clean test