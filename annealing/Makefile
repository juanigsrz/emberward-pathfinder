CXX = g++
CXXFLAGS = -std=c++17 -O2 -Wall -Wextra
TARGET = solver
SOURCES = solver.cpp

$(TARGET): $(SOURCES) solver.hpp
	$(CXX) $(CXXFLAGS) -o $(TARGET) $(SOURCES)

clean:
	rm -f $(TARGET)

.PHONY: clean