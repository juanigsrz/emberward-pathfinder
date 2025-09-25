CXX = g++
CXXFLAGS = -std=c++17 -O2 -Wall -Wextra
TARGET = genetic_algorithm
SOURCES = genetic_algorithm.cpp
HEADERS = genetic_algorithm.hpp

$(TARGET): $(SOURCES) $(HEADERS)
	$(CXX) $(CXXFLAGS) -o $(TARGET) $(SOURCES)

clean:
	rm -f $(TARGET)

test: $(TARGET)
	@echo "Testing genetic algorithm..."
	./$(TARGET) ../maps/basic.txt

.PHONY: clean test